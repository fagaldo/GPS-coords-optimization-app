package polsl.gps.gpsoptimization

import android.os.Build
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.math.*

/**
 * Klasa `GPSMovingAvgFuzzySmoothing` implementuje algorytm wygładzania danych GPS przy użyciu rozmytej średniej ruchomej.
 * Odpowiedzialna za wygładzanie współrzędnych oraz grupowanie punktów na podstawie ich podobieństwa.
 *
 * @param latitudes Tablica szerokości geograficznych.
 * @param longitudes Tablica długości geograficznych.
 * @param threshold Próg podobieństwa współrzędnych do grupowania punktów.
 * @param timeStamps Lista znaczników czasowych.
 */
class GPSMovingAvgFuzzySmoothing(
    private val latitudes: DoubleArray,
    private val longitudes: DoubleArray,
    private val threshold: Double,
    private val timeStamps: MutableList<Long>
) {
    /**
     * Tablica przechowująca wygładzone szerokości geograficzne GPS.
     */
    private val smoothedLatitudes = DoubleArray(latitudes.size)

    /**
     * Tablica przechowująca wygładzone długości geograficzne GPS.
     */
    private val smoothedLongitudes = DoubleArray(longitudes.size)

    /**
     * Grupy punktów podobnych.
     */
    private val groups = mutableMapOf<Int, MutableList<Int>>()

    /**
     * Mapa indeksów grup.
     */
    private var groupIndexMap = mutableMapOf<Int, Int>()

    /**
     * Maksymalny rozmiar grupy.
     */
    private var maxGroupSize: Int = 10

    /**
     * Wygładza współrzędne GPS oraz grupuje punkty na podstawie ich podobieństwa.
     */
    @RequiresApi(Build.VERSION_CODES.N)
    fun smoothAndEvaluateAndGroup() {
        createWindows(1000)
        for (i in latitudes.indices) {
            val smoothedCoordinate = movingFuzzyAverage(i)
            smoothedLatitudes[i] = smoothedCoordinate.first
            smoothedLongitudes[i] = smoothedCoordinate.second
            val groupId = groupIndexMap[i]
            if (groupId != null) {
                groups.computeIfAbsent(groupId) { mutableListOf() }.add(i)
            }
        }
    }

    /**
     * Tworzy okna czasowe do grupowania punktów na podstawie ich podobieństwa.
     *
     * @param timeThreshold Próg czasowy dla grupowania.
     * @return Lista identyfikatorów okien.
     */
    private fun createWindows(timeThreshold: Long): List<Int> {
        val windows = mutableListOf<Int>()
        val indexesInWindows = mutableListOf<Int>()
        val ungroupedIndexes = mutableListOf<Int>()

        var incrementRate = 0.001
        var incrementTimeRate: Long = 10
        var currentGroupIndex = 0
        val groupSizes = mutableMapOf<Int, Int>()

        for (i in 0 until latitudes.size) {
            for (j in 0 until latitudes.size) {
                if (i != j && isSimilar(latitudes[i], longitudes[i], latitudes[j], longitudes[j], threshold)
                    && isSimilarTime(timeStamps[i], timeStamps[j], 3000)) {

                    val groupI = groupIndexMap[i]
                    val groupJ = groupIndexMap[j]

                    if (groupI == null && groupJ == null) {
                        groupIndexMap[i] = currentGroupIndex
                        groupIndexMap[j] = currentGroupIndex
                        indexesInWindows.add(i)
                        indexesInWindows.add(j)
                        windows.add(currentGroupIndex)
                        groupSizes[currentGroupIndex] = 2
                        currentGroupIndex++
                    } else if (groupI != null && groupJ == null) {
                        if (groupSizes[groupI]!! < maxGroupSize) {
                            groupIndexMap[j] = groupI
                            indexesInWindows.add(j)
                            groupSizes[groupI] = groupSizes[groupI]!! + 1
                        }
                    } else if (groupI == null && groupJ != null) {
                        if (groupSizes[groupJ]!! < maxGroupSize) {
                            groupIndexMap[i] = groupJ
                            indexesInWindows.add(i)
                            groupSizes[groupJ] = groupSizes[groupJ]!! + 1
                        }
                    }
                }
            }
        }

        ungroupedIndexes.addAll((latitudes.indices).filter { !indexesInWindows.contains(it) })

        var diff = 5.0
        var oldDiff = diff
        var timeDiff: Long = 2000
        var oldTimeDiff = timeDiff
        var iteration = 0

        while (ungroupedIndexes.isNotEmpty()) {
            val iterator = ungroupedIndexes.iterator()
            val foundIndex = mutableListOf<Int>()

            while (iterator.hasNext()) {
                val itIndex = iterator.next()
                var isAssigned = false

                for (tmp in indexesInWindows) {
                    if (iteration % 1000 == 0)
                        maxGroupSize++
                    var group: Int? = null
                    for (tmp1 in indexesInWindows) {
                        diff = similarityVal(latitudes[itIndex], longitudes[itIndex], latitudes[tmp1], longitudes[tmp1])
                        if (diff < oldDiff && groupSizes[groupIndexMap[tmp1]]!! < maxGroupSize) {
                            oldDiff = diff
                            group = groupIndexMap[tmp1]
                        }
                    }
                    if (group != null && groupSizes[group]!! < maxGroupSize) {
                        groupIndexMap[itIndex] = group
                        indexesInWindows.add(itIndex)
                        foundIndex.add(itIndex)
                        groupSizes[group] = groupSizes[group]!! + 1
                        iterator.remove()
                        isAssigned = true
                        break
                    }

                    if (!isAssigned) {
                        for (tmp2 in indexesInWindows) {
                            timeDiff = similarityTimeVal(timeStamps[itIndex], timeStamps[tmp2])
                            if (timeDiff < oldTimeDiff && groupSizes[groupIndexMap[tmp2]]!! < maxGroupSize) {
                                oldTimeDiff = timeDiff
                                group = groupIndexMap[tmp2]
                            }
                        }
                        if (group != null && (groupSizes[group]!! < maxGroupSize)) {
                            groupIndexMap[itIndex] = group
                            indexesInWindows.add(itIndex)
                            foundIndex.add(itIndex)
                            groupSizes[group] = groupSizes[group]!! + 1
                            iterator.remove()
                            isAssigned = true
                            break
                        }
                    }
                }

                oldDiff = 5.0
                oldTimeDiff = 2000 + incrementTimeRate
                if (oldTimeDiff > 10000) {
                    oldTimeDiff = 2000
                    incrementTimeRate = 100
                }
            }
            iteration++
            incrementRate += 0.001
            incrementTimeRate += 100
        }
        return windows
    }

    /**
     * Oblicza różnicę czasu między dwoma punktami.
     *
     * @param time1 Czas pierwszego punktu.
     * @param time2 Czas drugiego punktu.
     * @return Różnica czasu jako wartość Long.
     */
    private fun similarityTimeVal(time1: Long, time2: Long): Long {
        return abs(time1 - time2)
    }

    /**
     * Oblicza odległość między dwoma punktami GPS.
     *
     * @param lat1 Szerokość geograficzna pierwszego punktu.
     * @param lon1 Długość geograficzna pierwszego punktu.
     * @param lat2 Szerokość geograficzna drugiego punktu.
     * @param lon2 Długość geograficzna drugiego punktu.
     * @return Odległość między punktami w kilometrach.
     */
    private fun similarityVal(
        lat1: Double, lon1: Double,
        lat2: Double, lon2: Double
    ): Double {
        val R = 6371.0 // Średni promień Ziemi w kilometrach

        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c
    }

    /**
     * Sprawdza, czy dwa punkty GPS są podobne pod względem czasu.
     *
     * @param time1 Czas pierwszego punktu.
     * @param time2 Czas drugiego punktu.
     * @param threshold Próg podobieństwa czasowego.
     * @return True, jeśli różnica czasu jest mniejsza od progu.
     */
    private fun isSimilarTime(time1: Long, time2: Long, threshold: Long): Boolean {
        return abs(time1 - time2) < threshold
    }

    /**
     * Sprawdza, czy dwa punkty GPS są podobne pod względem współrzędnych.
     *
     * @param lat1 Szerokość geograficzna pierwszego punktu.
     * @param lon1 Długość geograficzna pierwszego punktu.
     * @param lat2 Szerokość geograficzna drugiego punktu.
     * @param lon2 Długość geograficzna drugiego punktu.
     * @param threshold Próg podobieństwa współrzędnych.
     * @return True, jeśli punkty są podobne pod względem współrzędnych.
     */
    private fun isSimilar(
        lat1: Double, lon1: Double,
        lat2: Double, lon2: Double,
        threshold: Double
    ): Boolean {
        val R = 6371.0 // Średni promień Ziemi w kilometrach

        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c < threshold
    }

    /**
     * Wylicza rozmytą średnią ruchomą dla punktu GPS.
     *
     * @param index Indeks punktu, dla którego wyliczamy średnią.
     * @return Wygładzona współrzędna GPS jako para (szerokość, długość).
     */
    private fun movingFuzzyAverage(index: Int): Pair<Double, Double> {
        var sumLat = 0.0
        var sumLon = 0.0
        var weightSum = 0.0
        val groupToIterate = groupIndexMap[index]

        for ((i, group) in groupIndexMap) {
            if (group == groupToIterate) {
                val sr = calculateMedianCenter(group)
                val ab = calculateOutlierDistanceForGroup(group, sr)
                val weight = membershipFunction(latitudes[i], longitudes[i], sr.latitude, sr.longitude, ab.first, ab.second)
                sumLat += weight * latitudes[i]
                sumLon += weight * longitudes[i]
                weightSum += weight
            }
        }
        val smoothedLatitude = sumLat / weightSum
        val smoothedLongitude = sumLon / weightSum
        return Pair(smoothedLatitude, smoothedLongitude)
    }

    /**
     * Oblicza funkcję przynależności (membership function) do rozmytej średniej ruchomej.
     *
     * @param x Wartość szerokości geograficznej.
     * @param y Wartość długości geograficznej.
     * @param s Mediana szerokości geograficznej.
     * @param r Mediana długości geograficznej.
     * @param a Odległość od wartości mediany w szerokości geograficznej.
     * @param b Odległość od wartości mediany w długości geograficznej.
     * @return Wartość funkcji przynależności.
     */
    private fun membershipFunction(x: Double, y: Double, s: Double, r: Double, a: Double, b: Double): Double {
        if (a == 0.0 || b == 0.0)
            return 1.0
        val distanceX = abs(x - s)
        val distanceY = abs(y - r)
        val weightX = exp(-(distanceX / a).pow(2))
        val weightY = exp(-(distanceY / b).pow(2))
        return weightX * weightY
    }

    /**
     * Oblicza medianę współrzędnych dla grupy punktów GPS.
     *
     * @param groupId Identyfikator grupy.
     * @return Mediana współrzędnych GPS jako LatLng.
     */
    private fun calculateMedianCenter(groupId: Int): LatLng {
        val groupIndices = groupIndexMap.filter { it.value == groupId }.keys.toList()
        val medianLatitude = calculateMedian(latitudes, groupIndices)
        val medianLongitude = calculateMedian(longitudes, groupIndices)
        return LatLng(medianLatitude, medianLongitude)
    }

    /**
     * Oblicza medianę wartości dla zadanej grupy indeksów.
     *
     * @param array Tablica wartości.
     * @param indices Lista indeksów w tablicy.
     * @return Mediana wartości.
     */
    private fun calculateMedian(array: DoubleArray, indices: List<Int>): Double {
        val sortedValues = indices.map { array[it] }.sorted()
        val medianIndex = indices.size / 2
        return if (indices.size % 2 == 0) {
            (sortedValues[medianIndex - 1] + sortedValues[medianIndex]) / 2.0
        } else {
            sortedValues[medianIndex]
        }
    }

    /**
     * Oblicza maksymalną odległość od mediany w grupie punktów GPS.
     *
     * @param groupId Identyfikator grupy.
     * @param median Mediana współrzędnych GPS dla grupy.
     * @return Para maksymalnych odległości dla szerokości i długości geograficznej.
     */
    private fun calculateOutlierDistanceForGroup(groupId: Int, median: LatLng): Pair<Double, Double> {
        val groupIndices = groupIndexMap.filter { it.value == groupId }.keys.toList()
        var maxDistanceLat = 0.0
        var maxDistanceLon = 0.0

        for (index in groupIndices) {
            val distance = calculateDistance(latitudes[index], median.latitude, longitudes[index], median.longitude)
            if (distance.first > maxDistanceLat) {
                maxDistanceLat = distance.first
            }
            if (distance.second > maxDistanceLon) {
                maxDistanceLon = distance.second
            }
        }
        return Pair(maxDistanceLat, maxDistanceLon)
    }

    /**
     * Oblicza odległość między dwoma punktami GPS w kategoriach szerokości i długości geograficznej.
     *
     * @param lat1 Szerokość geograficzna pierwszego punktu.
     * @param lat2 Szerokość geograficzna drugiego punktu.
     * @param lon1 Długość geograficzna pierwszego punktu.
     * @param lon2 Długość geograficzna drugiego punktu.
     * @return Para odległości dla szerokości i długości geograficznej.
     */
    private fun calculateDistance(lat1: Double, lat2: Double, lon1: Double, lon2: Double): Pair<Double, Double> {
        val dLat = abs(lat2 - lat1)
        val dLon = abs(lon2 - lon1)
        return Pair(dLat, dLon)
    }

    /**
     * Zwraca listę wygładzonych współrzędnych GPS.
     *
     * @return Lista wygładzonych współrzędnych GPS jako obiekty LatLng.
     */
    fun getSmoothedLatLngList(): List<LatLng> {
        val smoothedLatLngList = mutableListOf<LatLng>()
        for (i in smoothedLatitudes.indices) {
            smoothedLatLngList.add(LatLng(smoothedLatitudes[i], smoothedLongitudes[i]))
        }
        return smoothedLatLngList
    }

    /**
     * Zwraca mapę grup punktów GPS.
     *
     * @return Mapa grup punktów GPS.
     */
    fun getGroups(): Map<Int, List<Int>> {
        return groups
    }
}