package polsl.gps.gpsoptimization

import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.math.*

class GPSMovingAvgFuzzySmoothing(
    private val latitudes: DoubleArray,
    private val longitudes: DoubleArray,
    private val threshold: Double,
    private val timeStamps: MutableList<Long>
) {
    private val smoothedLatitudes = DoubleArray(latitudes.size)
    private val smoothedLongitudes = DoubleArray(longitudes.size)
    private val groups = mutableMapOf<Int, MutableList<Int>>()
    private var groupIndexMap = mutableMapOf<Int, Int>()
    private var maxGroupSize: Int = 10

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
                Log.d("Porownanie", "Porownuje i: $i z j: $j")
                if (i != j && isSimilar(latitudes[i], longitudes[i], latitudes[j], longitudes[j], threshold)
                    && isSimilarTime(timeStamps[i], timeStamps[j], 3000)) {
                    Log.d("Podobienstwo", "Znaleziona podobna para: ($i, $j)")

                    val groupI = groupIndexMap[i]
                    val groupJ = groupIndexMap[j]

                    if (groupI == null && groupJ == null) {
                        groupIndexMap[i] = currentGroupIndex
                        groupIndexMap[j] = currentGroupIndex
                        indexesInWindows.add(i)
                        indexesInWindows.add(j)
                        windows.add(currentGroupIndex)
                        groupSizes[currentGroupIndex] = 2
                        Log.d("Tworze grupe", "Index: $i, $j, Grupa: $currentGroupIndex")
                        currentGroupIndex++
                    } else if (groupI != null && groupJ == null) {
                        if (groupSizes[groupI]!! < maxGroupSize) {
                            groupIndexMap[j] = groupI
                            indexesInWindows.add(j)
                            groupSizes[groupI] = groupSizes[groupI]!! + 1
                            Log.d("Dodaje do grupy", "Index: $j, Grupa: $groupI")
                        }
                    } else if (groupI == null && groupJ != null) {
                        if (groupSizes[groupJ]!! < maxGroupSize) {
                            groupIndexMap[i] = groupJ
                            indexesInWindows.add(i)
                            groupSizes[groupJ] = groupSizes[groupJ]!! + 1
                            Log.d("Dodaje do grupy", "Index: $i, Grupa: $groupJ")
                        }
                    } else {
                        Log.d("Oba punkty w grupach", "Index: $i, Grupa: $groupI, Index: $j, Grupa: $groupJ")
                    }
                }
            }
        }

        ungroupedIndexes.addAll((latitudes.indices).filter { !indexesInWindows.contains(it) })
        Log.d("Nieprzypisane indeksy", ungroupedIndexes.toString())

        var diff = 5.0
        var oldDiff = diff
        var timeDiff:Long = 2000
        var oldTimeDiff = timeDiff
        var iteration = 0
        while (ungroupedIndexes.isNotEmpty()) {
            Log.d("Oprozniamy", ungroupedIndexes.toString())
            Log.d("Time thresh", (timeThreshold + incrementTimeRate).toString())

            val iterator = ungroupedIndexes.iterator()
            val foundIndex = mutableListOf<Int>()

            while (iterator.hasNext()) {
                val itIndex = iterator.next()
                var isAssigned = false

                for (tmp in indexesInWindows) {
                    if(iteration % 1000 == 0)
                        maxGroupSize++
                    var group: Int? = null
                    for(tmp1 in indexesInWindows)
                    {
                        diff = similarityVal(latitudes[itIndex], longitudes[itIndex], latitudes[tmp1], longitudes[tmp1])
                        if(diff < oldDiff && groupSizes[groupIndexMap[tmp1]]!! < maxGroupSize) {
                            oldDiff = diff
                            group = groupIndexMap[tmp1]
                        }
                    }
                    Log.d("Obliczony diffG", oldDiff.toString())
                    if (group != null && groupSizes[group]!! < maxGroupSize) {
                        Log.d("DorzucamG", "na podstawie geografii dla $itIndex do grupy $group")
                        groupIndexMap[itIndex] = group
                        indexesInWindows.add(itIndex)
                        foundIndex.add(itIndex)
                        groupSizes[group] = groupSizes[group]!! + 1
                        iterator.remove()
                        isAssigned = true
                        break
                    }

                    if (!isAssigned) {
                        for(tmp2 in indexesInWindows)
                        {
                            timeDiff = similarityTimeVal(timeStamps[itIndex], timeStamps[tmp2])
                            if(timeDiff < oldTimeDiff && groupSizes[groupIndexMap[tmp2]]!! < maxGroupSize) {
                                oldTimeDiff = timeDiff
                                group = groupIndexMap[tmp2]
                            }
                        }
                        Log.d("Obliczony diffT", oldTimeDiff.toString())
                        if (group != null && (groupSizes[group]!! < maxGroupSize)) {
                            Log.d("Dorzucam", "na podstawie czasu dla $itIndex do grupy $group granica czas była $oldTimeDiff")
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
                if(oldTimeDiff > 10000) {
                    oldTimeDiff = 2000
                    incrementTimeRate = 100
                }
            }
            iteration ++
            incrementRate += 0.001
            incrementTimeRate += 100

            Log.d("Aktualny incrementRate", incrementRate.toString())
            Log.d("Aktualny incrementTime", incrementTimeRate.toString())
        }
        groupSizes.forEach { (group, size) ->
            Log.d("Liczebność grupy", "Grupa: $group, Liczebność: $size")
        }
        return windows
    }
    private fun similarityTimeVal(time1: Long, time2: Long): Long{
        return abs(time1- time2)
    }

    private fun similarityVal(
    lat1: Double, lon1: Double,
    lat2: Double, lon2: Double
    ): Double
    {
        val R = 6371.0 // Średni promień Ziemi w kilometrach

        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c
    }
    private fun isSimilarTime(time1: Long, time2: Long, threshold: Long): Boolean{
        return abs(time1- time2) < threshold
    }
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
    private fun membershipFunction(x: Double, y: Double, s: Double, r: Double, a: Double, b: Double): Double {
        if (a == 0.0|| b == 0.0)
            return 1.0
        val distanceX = abs(x - s)
        val distanceY = abs(y - r)
        val weightX = exp(-(distanceX / a).pow(2))
        val weightY = exp(-(distanceY / b).pow(2))
        return weightX * weightY
    }
    private fun calculateMedianCenter(groupId: Int): LatLng {
        val groupIndices = groupIndexMap.filter { it.value == groupId }.keys.toList()
        val medianLatitude = calculateMedian(latitudes, groupIndices)
        val medianLongitude = calculateMedian(longitudes, groupIndices)
        return LatLng(medianLatitude, medianLongitude)
    }

    private fun calculateMedian(array: DoubleArray, indices: List<Int>): Double {
        val sortedValues = indices.map { array[it] }.sorted()
        val medianIndex = indices.size / 2
        return if (indices.size % 2 == 0) {
            (sortedValues[medianIndex - 1] + sortedValues[medianIndex]) / 2.0
        } else {
            sortedValues[medianIndex]
        }
    }
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
    private fun calculateDistance(lat1: Double, lat2: Double, lon1: Double, lon2:Double): Pair<Double, Double> {
        val dLat = abs(lat2 - lat1)
        val dLon = abs(lon2 - lon1)
        return Pair(dLat, dLon)
    }
    private fun movingFuzzyAverage(index: Int): Pair<Double, Double>{
        var sumLat = 0.0
        var sumLon = 0.0
        var weightSum = 0.0
        val groupToIterate = groupIndexMap[index]

        for ((i, group) in groupIndexMap) {
            if (group == groupToIterate) {
                val sr = calculateMedianCenter(group)
                Log.d("mediana", sr.latitude.toString() + " " + sr.longitude.toString())
                val ab = calculateOutlierDistanceForGroup(group, sr)
                Log.d("outlier", ab.first.toString() + " " + ab.second.toString())
                val weight = membershipFunction(latitudes[i], longitudes[i], sr.latitude, sr.longitude, ab.first, ab.second) // Domyślne wartości a i b
                Log.d("Obliczone wagi dla ele ", weight.toString() + " element " + index + "grupa" + group)
                Log.d("Koordynaty tego elemu: ", latitudes[i].toString() + " " + longitudes[i].toString())
                sumLat += weight * latitudes[i]
                sumLon += weight * longitudes[i]
                weightSum += weight
            }
        }
        val smoothedLatitude = sumLat / weightSum
        val smoothedLongitude = sumLon / weightSum
        return Pair(smoothedLatitude, smoothedLongitude)
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
}