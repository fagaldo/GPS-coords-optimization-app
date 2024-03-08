package polsl.gps.gpsoptimization
import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import org.jetbrains.kotlinx.multik.api.mk
import org.jetbrains.kotlinx.multik.api.ndarray
import kotlin.math.abs
import kotlin.collections.mutableMapOf

class GpsLowessSmoothing(private val latitudes: DoubleArray, private val longitudes: DoubleArray, private var bandwidth: Double, private val trueLatitudes: DoubleArray,
                         private val trueLongitudes: DoubleArray) {
    private val smoothedLatitudes = DoubleArray(latitudes.size)
    private val smoothedLongitudes = DoubleArray(longitudes.size)
    private val groups = mutableMapOf<Int, MutableList<Int>>()
    private val errors = DoubleArray(latitudes.size)
    private var windowIndexMap = mutableMapOf<Int, Int>()
    @RequiresApi(Build.VERSION_CODES.N)
    fun smoothAndEvaluateAndGroup(threshold: Double) {
        //val windowSize = calculateWindowSize(latitudes, longitudes)
        createWindows(latitudes, longitudes, threshold, windowIndexMap)
        for (i in latitudes.indices) {

            Log.d("WINDOWS SIZES", " indeksy: $windowIndexMap")

            val smoothedCoordinate = loessSmooth(i, windowIndexMap)
            smoothedLatitudes[i] = smoothedCoordinate.first
            smoothedLongitudes[i] = smoothedCoordinate.second
            Log.d("index grupy", windowIndexMap[i].toString())
            var groupId = windowIndexMap[i]
            Log.d("koorydanty LOWESS:", (i.toString() + " " + smoothedLatitudes[i].toString() + smoothedLongitudes[i].toString() + "  $i" + groupId.toString()))
            // Ewaluacja MAE
            errors[i] = calculateMAE(smoothedCoordinate.first, smoothedCoordinate.second, groupId)
            if (groupId != null) {
                groups.computeIfAbsent(groupId) { mutableListOf() }.add(i)
            }
        }
        Log.d("GRUPY", groups.toString())
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


    private fun loessSmooth(index: Int, windowIndexMap: MutableMap<Int, Int>): Pair<Double, Double> {
        val xi = latitudes[index]
        val weights = calculateWeights(xi, index, windowIndexMap)

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

    private fun calculateWeights(xi: Double, index: Int, windowIndexMap: MutableMap<Int, Int>): DoubleArray {
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
    private fun isSimilar(
        lat1: Double, lon1: Double,
        lat2: Double, lon2: Double,
        threshold: Double
    ): Boolean {
        val latDiff = Math.abs(lat1 - lat2)
        val lonDiff = Math.abs(lon1 - lon2)
        return latDiff < threshold && lonDiff < threshold
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
