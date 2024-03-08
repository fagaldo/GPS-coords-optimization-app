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
    private val trueLatitudes: DoubleArray,
    private val trueLongitudes: DoubleArray
) {
    private val smoothedLatitudes = DoubleArray(latitudes.size)
    private val smoothedLongitudes = DoubleArray(longitudes.size)
    private val errors = DoubleArray(latitudes.size)
    private val groups = mutableMapOf<Int, MutableList<Int>>()
    private var groupIndexMap = mutableMapOf<Int, Int>()

    @RequiresApi(Build.VERSION_CODES.N)
    fun smoothAndEvaluateAndGroup() {
        createWindows(latitudes, longitudes, threshold, groupIndexMap)
        for (i in latitudes.indices) {
            val smoothedCoordinate = movingFuzzyAverage(i, groupIndexMap)
            smoothedLatitudes[i] = smoothedCoordinate.first
            smoothedLongitudes[i] = smoothedCoordinate.second
            var groupId = groupIndexMap[i]
            errors[i] = calculateMAE(smoothedCoordinate.first, smoothedCoordinate.second, groupId)
            if (groupId != null) {
                groups.computeIfAbsent(groupId) { mutableListOf() }.add(i)
            }
        }
    }

    private fun createWindows(latitudes: DoubleArray, longitudes: DoubleArray, threshold: Double, windowIndexMap: MutableMap<Int, Int>): List<Int>{
        val windows = mutableListOf<Int>()
        val indexesInWindows = mutableListOf<Int>()
        var windowIndex = 0
        var lastIndex = 0
        var index = 0
        while (index < latitudes.size) {
            var windowSize = 1 // Rozmiar aktualnego okna

            // Sprawdzamy kolejne współrzędne, dopóki są podobne
            while (index < latitudes.size - 1 &&
                isSimilar(latitudes[index], longitudes[index], latitudes[index + 1], longitudes[index + 1], threshold)) {
                if(index !in indexesInWindows) {
                    windowSize++
                    indexesInWindows.add(index)
                    windowIndexMap[index] = windowIndex
                }

                index++
            }
            lastIndex = index
            if(isSimilar(latitudes[index-1], longitudes[index-1], latitudes[lastIndex], longitudes[lastIndex], threshold) && lastIndex !in indexesInWindows)
                windowIndexMap[lastIndex] = windowIndex
            // Sprawdzamy, czy kolejne punkty nie są zbyt odległe pod względem indeksu i spełniają warunek podobieństwa
            while (index < latitudes.size - 1)
            {   index++
                if(isSimilar(latitudes[lastIndex], longitudes[lastIndex], latitudes[index], longitudes[index], threshold) && index !in indexesInWindows) {
                    windowSize++
                    indexesInWindows.add(index)
                    windowIndexMap[index] = windowIndex

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

        return windows
    }
    private fun isSimilar(
        lat1: Double, lon1: Double,
        lat2: Double, lon2: Double,
        threshold: Double
    ): Boolean {
        val latDiff = Math.abs(lat1 - lat2)
        val lonDiff = Math.abs(lon1 - lon2)
        return latDiff < threshold && lonDiff < threshold
    }
    private fun membershipFunction(x: Double, y: Double, s: Double, r: Double, a: Double, b: Double): Double {
        //return exp(-((x - s) / 0.01*a).pow(2) - ((y - r) / 0.01*b).pow(2)) //tutaj można się pobawić z tymi wagami
        val distanceX = abs(x - s)
        val distanceY = abs(y - r)
        val weightX = exp(-(distanceX / a).pow(2))
        val weightY = exp(-(distanceY / b).pow(2)) //TODO ciekawostka
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
        val earthRadius = 6371000.0 // Radius of the Earth in meters

        val dLat = abs(lat2 - lat1)
        val dLon = abs(lon2 - lon1)

        //val a = sin(dLat / 2).pow(2) + cos(lat1) * cos(lat2) * sin(dLon / 2).pow(2)
        //val c = 2 * atan2(sqrt(a), sqrt(1 - a))

        //val distance = earthRadius * c * 100 // Convert distance to centimeters

        //val latDistance = distance * cos(lat1) // Distance on latitude in centimeters
        //val lonDistance = distance // Distance on longitude in centimeters
        Log.d("zwracam odległość", "$dLat $dLon")
        return Pair(dLat, dLon)

    }
    private fun movingFuzzyAverage(index: Int, windowIndexMap: MutableMap<Int, Int>): Pair<Double, Double>{
        var sumLat = 0.0
        var sumLon = 0.0
        var weightSum = 0.0
        val groupToIterate = windowIndexMap[index]

        for ((i, group) in windowIndexMap) {
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
        Log.d("koord:", "$smoothedLatitude" + " $smoothedLongitude" +" $index")
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