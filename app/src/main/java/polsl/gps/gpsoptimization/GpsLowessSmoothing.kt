package polsl.gps.gpsoptimization
import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.collections.mutableMapOf
import kotlin.math.*

class GpsLowessSmoothing(private val latitudes: DoubleArray, private val longitudes: DoubleArray, private var bandwidth: Double, private val trueLatitudes: DoubleArray,
                         private val trueLongitudes: DoubleArray, private val timeStamps: MutableList<Long>, private val threshold: Double) {
    private val smoothedLatitudes = DoubleArray(latitudes.size)
    private val smoothedLongitudes = DoubleArray(longitudes.size)
    private val groups = mutableMapOf<Int, MutableList<Int>>()
    private val errors = DoubleArray(latitudes.size)
    private var windowIndexMap = mutableMapOf<Int, Int>()
    private var maxGroupSize: Int = 6
    @RequiresApi(Build.VERSION_CODES.N)
    fun smoothAndEvaluateAndGroup() {
        //val windowSize = calculateWindowSize(latitudes, longitudes)
        createWindows(1000)
        for (i in latitudes.indices) {
            Log.d("WINDOWS SIZES", " indeksy: $windowIndexMap")
            val smoothedCoordinate = loessSmooth(i)
            smoothedLatitudes[i] = smoothedCoordinate.first
            smoothedLongitudes[i] = smoothedCoordinate.second
            Log.d("index grupy", windowIndexMap[i].toString())
            val groupId = windowIndexMap[i]
            Log.d("koorydanty LOWESS:", (i.toString() + " " + smoothedLatitudes[i].toString() + smoothedLongitudes[i].toString() + "  $i" + groupId.toString()))
            // Ewaluacja MAE
            errors[i] = calculateMAE(smoothedCoordinate.first, smoothedCoordinate.second, groupId)
            if (groupId != null) {
                groups.computeIfAbsent(groupId) { mutableListOf() }.add(i)
            }
        }
        Log.d("GRUPY", groups.toString())
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

                    val groupI = windowIndexMap[i]
                    val groupJ = windowIndexMap[j]

                    if (groupI == null && groupJ == null) {
                        windowIndexMap[i] = currentGroupIndex
                        windowIndexMap[j] = currentGroupIndex
                        indexesInWindows.add(i)
                        indexesInWindows.add(j)
                        windows.add(currentGroupIndex)
                        groupSizes[currentGroupIndex] = 2
                        Log.d("Tworze grupe", "Index: $i, $j, Grupa: $currentGroupIndex")
                        currentGroupIndex++
                    } else if (groupI != null && groupJ == null) {
                        if (groupSizes[groupI]!! < maxGroupSize) {
                            windowIndexMap[j] = groupI
                            indexesInWindows.add(j)
                            groupSizes[groupI] = groupSizes[groupI]!! + 1
                            Log.d("Dodaje do grupy", "Index: $j, Grupa: $groupI")
                        }
                    } else if (groupI == null && groupJ != null) {
                        if (groupSizes[groupJ]!! < maxGroupSize) {
                            windowIndexMap[i] = groupJ
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
        var iteracja = 0
        while (ungroupedIndexes.isNotEmpty()) {
            Log.d("Oprozniamy", ungroupedIndexes.toString())
            Log.d("Time thresh", (timeThreshold + incrementTimeRate).toString())

            val iterator = ungroupedIndexes.iterator()
            val foundIndex = mutableListOf<Int>()

            while (iterator.hasNext()) {
                val itIndex = iterator.next()
                var isAssigned = false

                for (tmp in indexesInWindows) {
                    if(iteracja % 1000 == 0)
                        maxGroupSize++
                    var group: Int? = null
                    for(tmp1 in indexesInWindows)
                    {
                        diff = similarityVal(latitudes[itIndex], longitudes[itIndex], latitudes[tmp1], longitudes[tmp1])
                        if(diff < oldDiff && groupSizes[windowIndexMap[tmp1]]!! < maxGroupSize) {
                            oldDiff = diff
                            group = windowIndexMap[tmp1]
                        }
                    }
                    Log.d("Obliczony diffG", oldDiff.toString())
                    if (group != null && groupSizes[group]!! < maxGroupSize) {
                        Log.d("DorzucamG", "na podstawie geografii dla $itIndex do grupy $group")
                        windowIndexMap[itIndex] = group
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
                            if(timeDiff < oldTimeDiff && groupSizes[windowIndexMap[tmp2]]!! < maxGroupSize) {
                                oldTimeDiff = timeDiff
                                group = windowIndexMap[tmp2]
                            }
                        }
                        Log.d("Obliczony diffT", oldTimeDiff.toString())
                        if (group != null && (groupSizes[group]!! < maxGroupSize)) {
                            Log.d("Dorzucam", "na podstawie czasu dla $itIndex do grupy $group granica czas była $oldTimeDiff")
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
                if(oldTimeDiff > 10000) {
                    oldTimeDiff = 2000
                    incrementTimeRate = 100
                }
            }
            iteracja ++
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
//        val latDiff = lat1 - lat2
//        val lonDiff = lon1 - lon2
//        val distanceSquared = latDiff * latDiff + lonDiff * lonDiff
//        return sqrt(distanceSquared)
        val R = 6371.0 // Średni promień Ziemi w kilometrach

        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c
    }

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
        Log.d("Zwracam PAre: ", Pair(smoothedLatitude, smoothedLongitude).toString() + index.toString())
        return Pair(smoothedLatitude, smoothedLongitude)
    }

    private fun calculateWeights(xi: Double, index: Int): DoubleArray {
        val weights = DoubleArray(latitudes.size)
        val groupToIterate = windowIndexMap[index]
        for ((i, group) in windowIndexMap) {
            if (group == groupToIterate) {
                val diff = abs(latitudes[i] - xi)
                val weight = tricube(diff / bandwidth) // tricube weight function
                weights[i] = weight
            }
        }

        // Normalize weights to sum to 1
        val sumWeights = weights.sum()
        for (i in weights.indices) {
            weights[i] /= sumWeights
        }

        return weights
    }

    private fun tricube(x: Double): Double {
        val absX = abs(x)
        return if (absX < 1) {
            (1 - absX * absX * absX) * (1 - absX * absX * absX) * (1 - absX * absX * absX)
        } else {
            0.0
        }
    }
    fun getSmoothedLatLngList(): List<LatLng> {
        val smoothedLatLngList = mutableListOf<LatLng>()
        for (i in smoothedLatitudes.indices) {
            smoothedLatLngList.add(LatLng(smoothedLatitudes[i], smoothedLongitudes[i]))
        }
        return smoothedLatLngList
    }
    private fun isSimilarTime(time1: Long, time2: Long, threshold: Long): Boolean{
        return abs(time1- time2) < threshold
    }
    private fun isSimilar(
        lat1: Double, lon1: Double,
        lat2: Double, lon2: Double,
        threshold: Double
    ): Boolean {
//        val latDiff = lat1 - lat2
//        val lonDiff = lon1 - lon2
//        val distanceSquared = latDiff * latDiff + lonDiff * lonDiff
//        return distanceSquared < threshold * threshold
        val R = 6371.0 // Średni promień Ziemi w kilometrach

        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))

        //Log.d("Odleglosc: ", "Między $lat1, $lat2" +"  "+ (R * c).toString())

        return R * c < threshold
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
    fun getGroups(): Map<Int, List<Int>> {
        return groups
    }

    fun getMAE(): DoubleArray {
        return errors
    }
}
