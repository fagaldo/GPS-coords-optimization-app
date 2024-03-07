package polsl.gps.gpsoptimization
import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class GpsKalmanPostProcessing(
    private val latitudes: DoubleArray,
    private val longitudes: DoubleArray,
    private val accelerationsX: MutableList<Double>,
    private val accelerationsY: MutableList<Double>,
    private val threshold: Double,
    private val trueLatitudes: DoubleArray,
    private val trueLongitudes: DoubleArray
) {
    private val correctedLatitudes = DoubleArray(latitudes.size)
    private val correctedLongitudes = DoubleArray(longitudes.size)
    private val groups = mutableMapOf<Int, MutableList<Int>>()
    private val errors = DoubleArray(latitudes.size)
    private var groupIndexMap = mutableMapOf<Int, Int>()

    // Inicjalizacja filtru Kalmana
    private val kalmanFilter = KalmanFilter(R = 0.999999f, Q = 0.0000001f)

    @RequiresApi(Build.VERSION_CODES.N)
    fun applyPostProcessingAndGroup() {
        // Iteracja po wszystkich pomiarach
        createWindows(latitudes, longitudes, threshold, groupIndexMap)
        for (i in latitudes.indices) {
            // Sprawdzenie, czy odczyty z akcelerometra są wystarczająco duże, aby wskazać na zmianę kierunku
            // Wykorzystanie filtru Kalmana do korekcji ścieżki lokalizacyjnej na podstawie przyspieszenia z akcelerometra
            //val correctedLat = kalmanFilter.filter(0.0f, accelerationsX[i].toFloat())
            //val correctedLat = kalmanFilter.getLatEstimate()
            //val correctedLon = kalmanFilter.filter(0.0f, accelerationsY[i].toFloat())
            //val correctedLon = kalmanFilter.getLonEstimate()
            val accelerationData =
                sqrt(accelerationsX[i] * accelerationsX[i] + accelerationsY[i] * accelerationsY[i])
            val correctedLat =
                kalmanFilter.filter(latitudes[i].toFloat(), accelerationData.toFloat())
            val correctedLon =
                kalmanFilter.filter(longitudes[i].toFloat(), accelerationData.toFloat())
            correctedLatitudes[i] = correctedLat.toDouble()
            correctedLongitudes[i] = correctedLon.toDouble()
            var groupId = groupIndexMap[i]

            errors[i] = calculateMAE(correctedLatitudes[i], correctedLongitudes[i], groupId)
            if (groupId != null) {
                groups.computeIfAbsent(groupId) { mutableListOf() }.add(i)
            }
        }

        Log.d("GRUPY KALman", groups.toString())
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
    private fun assignToGroup(index: Int, groupIndices: MutableList<Int>){
        for (i in latitudes.indices) {
            if (isNearby(latitudes[index], longitudes[index], latitudes[i], longitudes[i])) {
                groupIndices.add(i)
            }
        }
    }
    private fun isNearby(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Boolean {
        val latDiff = abs(lat1 - lat2)
        val lonDiff = abs(lon1 - lon2)
        return latDiff < threshold && lonDiff < threshold
    }
    private fun whichGroup(index: Int): MutableList<Int>{
        groups.values.forEach { groupIndices ->
            if (index in groupIndices) {
                return groupIndices
            }
        }
        return mutableListOf<Int>()
    }
    fun getGroups(): Map<Int, List<Int>> {
        return groups
    }
    fun getMAE(): DoubleArray {
        return errors
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
    // Obliczenie odległości między dwoma punktami na sferze
    private fun calculateDistance(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        // Rząd wielkości promienia Ziemi w metrach
        val R = 6371e3

        // Konwersja na radiany
        val lat1Rad = Math.toRadians(lat1)
        val lon1Rad = Math.toRadians(lon1)
        val lat2Rad = Math.toRadians(lat2)
        val lon2Rad = Math.toRadians(lon2)

        // Obliczenie różnicy długości i szerokości geograficznych
        val dLon = lon2Rad - lon1Rad
        val dLat = lat2Rad - lat1Rad

        // Obliczenie odległości
        val a = kotlin.math.sin(dLat / 2) * kotlin.math.sin(dLat / 2) +
                kotlin.math.cos(lat1Rad) * kotlin.math.cos(lat2Rad) *
                kotlin.math.sin(dLon / 2) * kotlin.math.sin(dLon / 2)
        val c = 2 * kotlin.math.atan2(kotlin.math.sqrt(a), kotlin.math.sqrt(1 - a))
        return R * c
    }
    private fun isSignificantChange(accelerationX: Double, accelerationY: Double): Boolean {
        val totalAcceleration = kotlin.math.sqrt(accelerationX.pow(2) + accelerationY.pow(2))
        return totalAcceleration > threshold
    }
    fun getCorrectedLatLngList(): List<LatLng> {
        val correctedLatLngList = mutableListOf<LatLng>()
        for (i in correctedLatitudes.indices) {
            correctedLatLngList.add(LatLng(correctedLatitudes[i], correctedLongitudes[i]))
        }
        return correctedLatLngList
    }
}