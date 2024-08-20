package polsl.gps.gpsoptimization

import android.os.Build
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.collections.mutableMapOf
import kotlin.math.*

/**
 * Klasa GpsLowessSmoothing implementuje algorytm LOWESS do wygładzania danych GPS.
 * Odpowiedzialna za grupowanie punktów na podstawie ich podobieństwa i wygładzanie współrzędnych.
 *
 * @param latitudes Tablica szerokości geograficznych.
 * @param longitudes Tablica długości geograficznych.
 * @param bandwidth Parametr szerokości pasma dla algorytmu LOWESS.
 * @param timeStamps Lista znaczników czasowych.
 * @param threshold Próg podobieństwa geograficznego do grupowania punktów.
 */
class GpsLowessSmoothing(
    private val latitudes: DoubleArray,
    private val longitudes: DoubleArray,
    private var bandwidth: Double,
    private val timeStamps: MutableList<Long>,
    private val threshold: Double
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
     * Mapa indeksów okien.
     */
    private var windowIndexMap = mutableMapOf<Int, Int>()

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
            val smoothedCoordinate = loessSmooth(i)
            smoothedLatitudes[i] = smoothedCoordinate.first
            smoothedLongitudes[i] = smoothedCoordinate.second
            val groupId = windowIndexMap[i]
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

                    val groupI = windowIndexMap[i]
                    val groupJ = windowIndexMap[j]

                    if (groupI == null && groupJ == null) {
                        windowIndexMap[i] = currentGroupIndex
                        windowIndexMap[j] = currentGroupIndex
                        indexesInWindows.add(i)
                        indexesInWindows.add(j)
                        windows.add(currentGroupIndex)
                        groupSizes[currentGroupIndex] = 2
                        currentGroupIndex++
                    } else if (groupI != null && groupJ == null) {
                        if (groupSizes[groupI]!! < maxGroupSize) {
                            windowIndexMap[j] = groupI
                            indexesInWindows.add(j)
                            groupSizes[groupI] = groupSizes[groupI]!! + 1
                        }
                    } else if (groupI == null && groupJ != null) {
                        if (groupSizes[groupJ]!! < maxGroupSize) {
                            windowIndexMap[i] = groupJ
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
                        if (diff < oldDiff && groupSizes[windowIndexMap[tmp1]]!! < maxGroupSize) {
                            oldDiff = diff
                            group = windowIndexMap[tmp1]
                        }
                    }
                    if (group != null && groupSizes[group]!! < maxGroupSize) {
                        windowIndexMap[itIndex] = group
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
                            if (timeDiff < oldTimeDiff && groupSizes[windowIndexMap[tmp2]]!! < maxGroupSize) {
                                oldTimeDiff = timeDiff
                                group = windowIndexMap[tmp2]
                            }
                        }
                        if (group != null && (groupSizes[group]!! < maxGroupSize)) {
                            windowIndexMap[itIndex] = group
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
     * Wygładza współrzędne GPS przy użyciu algorytmu LOESS.
     *
     * @param index Indeks punktu do wygładzenia.
     * @return Wygładzona współrzędna GPS jako para (szerokość, długość).
     */
    private fun loessSmooth(index: Int): Pair<Double, Double> {
        val xi = latitudes[index]
        val weights = calculateWeights(xi, index)

        var sumLat = 0.0
        var sumLon = 0.0
        var sumWeights = 0.0
        val groupToIterate = windowIndexMap[index] // Numer grupy, którą chcesz przetworzyć

        for ((i, group) in windowIndexMap) {
            if (group == groupToIterate) {
                val weight = weights[i]
                sumLat += weight * latitudes[i]
                sumLon += weight * longitudes[i]
                sumWeights += weight
            }
        }

        val smoothedLatitude = sumLat / sumWeights
        val smoothedLongitude = sumLon / sumWeights
        return Pair(smoothedLatitude, smoothedLongitude)
    }

    /**
     * Oblicza wagi dla danego punktu w algorytmie LOESS.
     *
     * @param xi Współrzędna punktu, dla którego obliczamy wagi.
     * @param index Indeks punktu.
     * @return Tablica wag.
     */
    private fun calculateWeights(xi: Double, index: Int): DoubleArray {
        val weights = DoubleArray(latitudes.size)
        val groupToIterate = windowIndexMap[index]
        for ((i, group) in windowIndexMap) {
            if (group == groupToIterate) {
                val diff = abs(latitudes[i] - xi)
                val weight = tricube(diff / bandwidth) // funkcja wagowa tricube
                weights[i] = weight
            }
        }

        // Normalizacja wag, aby ich suma wynosiła 1
        val sumWeights = weights.sum()
        for (i in weights.indices) {
            weights[i] /= sumWeights
        }

        return weights
    }

    /**
     * Funkcja oblicza funkcję wagową tricube.
     *
     * @param x Wartość wejściowa.
     * @return Wartość funkcji tricube.
     */
    private fun tricube(x: Double): Double {
        val absX = abs(x)
        return if (absX < 1) {
            (1 - absX * absX * absX) * (1 - absX * absX * absX) * (1 - absX * absX * absX)
        } else {
            0.0
        }
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
     * Zwraca mapę grup punktów GPS.
     *
     * @return Mapa grup punktów GPS.
     */
    fun getGroups(): Map<Int, List<Int>> {
        return groups
    }
}