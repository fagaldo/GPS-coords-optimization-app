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
        var windowIndex = 0
        var lastIndex: Int
        var index = 0
        val ungroupedIndexes = mutableListOf<Int>()
        var incrementRate = 0.001
        var incrementTimeRate: Long = 1000
        while (index < latitudes.size) {
            var windowSize = 1 // Rozmiar aktualnego okna
            while (index < latitudes.size - 1 &&
                isSimilar(latitudes[index], longitudes[index], latitudes[index + 1], longitudes[index + 1], threshold)) {
                if(index !in indexesInWindows) {
                    windowSize++
                    indexesInWindows.add(index)
                    groupIndexMap[index] = windowIndex
                }
                index++
            }
            if(index == 0){
                lastIndex = 1
                index = 1
                Log.d("index", 0.toString())
                if(isSimilar(latitudes[index-1], longitudes[index-1], latitudes[lastIndex], longitudes[lastIndex], threshold) && lastIndex !in indexesInWindows)
                {
                    groupIndexMap[lastIndex] = windowIndex
                    indexesInWindows.add(lastIndex)
                }
                //index = 0
            }
            else{
                lastIndex = index
                if(isSimilar(latitudes[index-1], longitudes[index-1], latitudes[lastIndex], longitudes[lastIndex], threshold) && lastIndex !in indexesInWindows)
                {
                    groupIndexMap[lastIndex] = windowIndex
                    indexesInWindows.add(lastIndex)
                }
            }

            // Sprawdzamy, czy kolejne punkty nie są zbyt odległe pod względem indeksu i spełniają warunek podobieństwa
            while (index < latitudes.size - 1)
            {   index++
                if(isSimilar(latitudes[lastIndex], longitudes[lastIndex], latitudes[index], longitudes[index], threshold) && index !in indexesInWindows) {
                    windowSize++
                    indexesInWindows.add(index)
                    groupIndexMap[index] = windowIndex

                }
            }
            index = lastIndex
            // Dodajemy rozmiar aktualnego okna do listy okien, jeśli jest większy niż 1
            if (windowSize > 1) {
                windows.add(windowSize)
                windowIndex ++
            }

            index++
        }
        ungroupedIndexes.addAll((latitudes.indices).filter { !indexesInWindows.contains(it) })
        var diff = 10.0
        var oldDiff = diff
        while(ungroupedIndexes.size>0) {
            val foundIndex: MutableList<Int> = mutableListOf()
            Log.d("Oprozniamy", ungroupedIndexes.toString())
            ungroupedIndexes.forEach {
                for(tmp in indexesInWindows)
                {
                    // Przeszukanie indexesInWindows w poszukiwaniu najbliższego indeksu
                    if (isSimilar(
                            latitudes[it],
                            longitudes[it],
                            latitudes[tmp],
                            longitudes[tmp],
                            (threshold + incrementRate)
                        )
                    )
                    // Jeśli znaleziono najbliższy indeks
                    {
                        diff = similarityVal(latitudes[it],
                            longitudes[it],
                            latitudes[tmp],
                            longitudes[tmp])
                        if(diff<oldDiff)
                        {
                            oldDiff = diff
                            val group = groupIndexMap[tmp]
                            if (group != null) {
                                groupIndexMap[it] = group
                                foundIndex.add(it)
                            }
                        }

                    }
                    else if(it !in foundIndex && isSimilarTime(timeStamps[it], timeStamps[tmp], timeThreshold + incrementTimeRate)){
                        //Log.d("Dorzucam", "na podstawie czasu dla $it")
                        val group = groupIndexMap[tmp]
                        if (group != null) {
                            groupIndexMap[it] = group
                            foundIndex.add(it)
                        }
                    }
                }
                oldDiff = 1.0
            }
            ungroupedIndexes.removeAll(foundIndex)
            incrementRate+=incrementRate
            incrementTimeRate+=incrementTimeRate
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