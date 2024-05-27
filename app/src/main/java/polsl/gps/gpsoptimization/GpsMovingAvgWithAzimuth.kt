package polsl.gps.gpsoptimization

import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.math.*

class GpsMovingAvgWithAzimuth(
    private val latitudes: DoubleArray, private val longitudes: DoubleArray,
    private val threshold: Double, private val trueLatitudes: DoubleArray,
    private val trueLongitudes: DoubleArray, private val timeStamps: MutableList<Long>,
    private val azimuths: MutableList<Float>
) {
    private val smoothedLatitudes = DoubleArray(latitudes.size)
    private val smoothedLongitudes = DoubleArray(longitudes.size)
    private val errors = DoubleArray(latitudes.size)
    private val groups = mutableMapOf<Int, MutableList<Int>>()
    private var groupIndexMap = mutableMapOf<Int, Int>()

    @RequiresApi(Build.VERSION_CODES.N)
    fun smoothAndEvaluateAndGroup() {
        createWindows(1000)
        for (i in latitudes.indices) {
            val smoothedCoordinate = movingAverage(i)
            smoothedLatitudes[i] = smoothedCoordinate.first
            smoothedLongitudes[i] = smoothedCoordinate.second
            val groupId = groupIndexMap[i]
            errors[i] = calculateMAE(smoothedCoordinate.first, smoothedCoordinate.second, groupId)
            if (groupId != null) {
                groups.computeIfAbsent(groupId) { mutableListOf() }.add(i)
            }
        }
    }
    private fun createWindows(timeThreshold: Long): List<Int>{
        val windows = mutableListOf<Int>()
        val indexesInWindows = mutableListOf<Int>()
        val ungroupedIndexes = mutableListOf<Int>()
        var incrementRate = 0.01
        var incrementTimeRate: Long = 10

        var windowSize = 1 // Rozmiar aktualnego okna
        var currentGroupIndex = 0
        for (i in 0 until latitudes.size) {
            for (j in 0 until latitudes.size) {
                Log.d("Porownanie", "Porownuje i: $i z j: $j")
                if (isSimilar(latitudes[i], longitudes[i], latitudes[j], longitudes[j], threshold)
                    && isSimilarTime(timeStamps[i], timeStamps[j], 2000) && i!=j) {
                    Log.d("Podobienstwo", "Znaleziona podobna para: ($i, $j)")

                    // Sprawdzamy, czy którykolwiek z indeksów jest już w jakiejś grupie
                    val groupI = groupIndexMap[i]
                    val groupJ = groupIndexMap[j]

                    if (groupI == null && groupJ == null) {
                        // Żaden z punktów nie jest jeszcze w grupie, tworzymy nową grupę
                        groupIndexMap[i] = currentGroupIndex
                        groupIndexMap[j] = currentGroupIndex
                        indexesInWindows.add(i)
                        indexesInWindows.add(j)
                        windowSize += 2
                        windows.add(windowSize)
                        Log.d("Tworze grupe", "Index: $i, $j, Grupa: $currentGroupIndex")
                        currentGroupIndex++
                    } else if (groupI != null && groupJ == null) {
                        // i jest w grupie, dodajemy j do tej samej grupy
                        groupIndexMap[j] = groupI
                        indexesInWindows.add(j)
                        windowSize++
                        windows.add(windowSize)
                        Log.d("Dodaje do grupy", "Index: $j, Grupa: $groupI")
                    } else if (groupI == null && groupJ != null) {
                        // j jest w grupie, dodajemy i do tej samej grupy
                        groupIndexMap[i] = groupJ
                        indexesInWindows.add(i)
                        windowSize++
                        windows.add(windowSize)
                        Log.d("Dodaje do grupy", "Index: $i, Grupa: $groupJ")
                    } else {
                        // Oba punkty są już w grupach
                        Log.d("Oba punkty w grupach", "Index: $i, Grupa: $groupI, Index: $j, Grupa: $groupJ")
                    }
                }
            }
        }

// Dodaj wszystkie nieprzypisane indeksy do ungroupedIndexes
        ungroupedIndexes.addAll((latitudes.indices).filter { !indexesInWindows.contains(it) })

        var diff = 5.0
        var oldDiff = diff

        while (ungroupedIndexes.isNotEmpty()) {
            Log.d("Oprozniamy", ungroupedIndexes.toString())
            Log.d("Time thresh", (timeThreshold + incrementTimeRate).toString())

            val iterator = ungroupedIndexes.iterator()
            while (iterator.hasNext()) {
                val itIndex = iterator.next()
                var isAssigned = false

                for (tmp in indexesInWindows) {
                    // Sprawdzenie podobieństwa geograficznego i czasowego
                    if (isSimilar(latitudes[itIndex], longitudes[itIndex], latitudes[tmp], longitudes[tmp], threshold + incrementRate)
                        && isSimilarTime(timeStamps[itIndex], timeStamps[tmp], 1000)) {

                        // Obliczenie różnicy dla znalezionego podobnego indeksu
                        diff = similarityVal(latitudes[itIndex], longitudes[itIndex], latitudes[tmp], longitudes[tmp])
                        if (diff < oldDiff) {
                            oldDiff = diff
                            Log.d("DorzucamG", "na podstawie geografii dla $itIndex")
                            val group = groupIndexMap[tmp]
                            if (group != null) {
                                groupIndexMap[itIndex] = group
                                indexesInWindows.add(itIndex) // Dodaj do indeksów w oknach
                                iterator.remove() // Usuń przetworzony indeks z ungroupedIndexes
                                isAssigned = true
                                break
                            }
                        }
                    } else if (!isAssigned && isSimilarTime(timeStamps[itIndex], timeStamps[tmp], timeThreshold + incrementTimeRate)) {
                        // Dodanie na podstawie czasu, jeśli nie znaleziono na podstawie podobieństwa geograficznego
                        Log.d("Dorzucam", "na podstawie czasu dla $itIndex")
                        val group = groupIndexMap[tmp]
                        if (group != null) {
                            groupIndexMap[itIndex] = group
                            indexesInWindows.add(itIndex) // Dodaj do indeksów w oknach
                            iterator.remove() // Usuń przetworzony indeks z ungroupedIndexes
                            isAssigned = true
                            break
                        }
                    }
                }

                oldDiff = 5.0 // Resetowanie oldDiff po przetworzeniu każdego indeksu
            }

            // Zwiększanie wartości progów
            incrementRate += incrementRate
            incrementTimeRate += incrementTimeRate

            Log.d("XD", incrementRate.toString())
        }



        return windows
    }

    private fun movingAverage(index: Int): Pair<Double, Double> {
        var weightedSumLat = 0.0
        var weightedSumLon = 0.0
        val groupToIterate = groupIndexMap[index]
        val weights = calculateWeights(index)

        var totalWeight = 0.0
        for ((i, group) in groupIndexMap) {
            if (group == groupToIterate) {
                weightedSumLat += latitudes[i] * weights[i]
                weightedSumLon += longitudes[i] * weights[i]
                totalWeight += weights[i]
            }
        }
        val smoothedLatitude = weightedSumLat / totalWeight
        val smoothedLongitude = weightedSumLon / totalWeight
        Log.d("koord:", "$smoothedLatitude" + " $smoothedLongitude" +" $index")
        return Pair(smoothedLatitude, smoothedLongitude)
    }
    private fun calculateWeights(index: Int): DoubleArray {
        var angleDifference: Float
        val weights = DoubleArray(latitudes.size)
        val groupToIterate = groupIndexMap[index]
        for ((i, group) in groupIndexMap) {
            if (group == groupToIterate) {
                angleDifference = if(i == latitudes.size - 1)
                    abs(calculateAngleDifference(azimuths[i], azimuths[i-1]))
                else
                    abs(calculateAngleDifference(azimuths[i], azimuths[i+1]))
                weights[i] = angleDifference.toDouble() // Użyj różnicy kątów jako wagi
            }
        }
        // Normalize weights to sum to 1
        val sumWeights = weights.sum()
        for (i in weights.indices) {
            weights[i] /= sumWeights
        }
        return weights
    }
    // Funkcja pomocnicza do obliczenia różnicy kątów
    private fun calculateAngleDifference(angle1: Float, angle2: Float): Float {
        return abs(angle1 - angle2)
    }

    private fun isSimilarTime(time1: Long, time2: Long, threshold: Long): Boolean{
        return abs(time1- time2) < threshold
    }
    private fun isSimilar(
        lat1: Double, lon1: Double,
        lat2: Double, lon2: Double,
        threshold: Double
    ): Boolean {
        val R = 6371.0

        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c < threshold
    }
    private fun similarityVal(
        lat1: Double, lon1: Double,
        lat2: Double, lon2: Double
    ): Double
    {
        val R = 6371.0

        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c
    }

    fun getSmoothedLatLngList(): List<LatLng> {
        val smoothedLatLngList = mutableListOf<LatLng>()
        for (i in smoothedLatitudes.indices) {
            smoothedLatLngList.add(LatLng(smoothedLatitudes[i], smoothedLongitudes[i]))
            Log.d("koord: ", LatLng(smoothedLatitudes[i], smoothedLongitudes[i]).toString())
        }
        return smoothedLatLngList
    }

    fun getGroups(): Map<Int, List<Int>> {
        return groups
    }

    private fun calculateMAE(smoothedLatitude: Double, smoothedLongitude: Double, index: Int?): Double {
        if (index != null && index >= 0 && index < trueLatitudes.size && index < trueLongitudes.size) {
            val trueLatitude = trueLatitudes[index]
            val trueLongitude = trueLongitudes[index]
            return Math.abs(smoothedLatitude - trueLatitude) + Math.abs(smoothedLongitude - trueLongitude)
        } else {
            Log.d("Index", "Invalid index: $index")
            // Możesz zwrócić wartość domyślną lub NaN lub wykonać inne działania w przypadku błędnego indeksu
            return Double.NaN // NaN oznacza "Not a Number"
        }
    }
    fun getMAE(): DoubleArray {
        return errors
    }
}