package polsl.gps.gpsoptimization
import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.math.*

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
        createWindows()
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
            val groupId = groupIndexMap[i]

            errors[i] = calculateMAE(correctedLatitudes[i], correctedLongitudes[i], groupId)
            if (groupId != null) {
                groups.computeIfAbsent(groupId) { mutableListOf() }.add(i)
            }
        }

        Log.d("GRUPY KALman", groups.toString())
        }

    private fun createWindows(): List<Int>{
        val windows = mutableListOf<Int>()
        val indexesInWindows = mutableListOf<Int>()
        var windowIndex = 0
        var lastIndex: Int
        var index = 0
        val ungroupedIndexes = mutableListOf<Int>()
        var incrementRate = 0.001
        while (index < latitudes.size) {
            var windowSize = 1 // Rozmiar aktualnego okna

            // Sprawdzamy kolejne współrzędne, dopóki są podobne
            while (index < latitudes.size - 1 &&
                isSimilar(latitudes[index], longitudes[index], latitudes[index + 1], longitudes[index + 1], threshold)) {
                if(index !in indexesInWindows) {
                    windowSize++
                    indexesInWindows.add(index)
                    groupIndexMap[index] = windowIndex
                }

                index++
            }
            lastIndex = index
            if(isSimilar(latitudes[index-1], longitudes[index-1], latitudes[lastIndex], longitudes[lastIndex], threshold) && lastIndex !in indexesInWindows)
            {
                groupIndexMap[lastIndex] = windowIndex
                indexesInWindows.add(lastIndex)
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
                }
                oldDiff = 1.0

            }
            ungroupedIndexes.removeAll(foundIndex)
            incrementRate+=incrementRate
        }

        return windows
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

//    private fun isSignificantChange(accelerationX: Double, accelerationY: Double): Boolean {
//        val totalAcceleration = kotlin.math.sqrt(accelerationX.pow(2) + accelerationY.pow(2))
//        return totalAcceleration > threshold
//    }
    fun getCorrectedLatLngList(): List<LatLng> {
        val correctedLatLngList = mutableListOf<LatLng>()
        for (i in correctedLatitudes.indices) {
            correctedLatLngList.add(LatLng(correctedLatitudes[i], correctedLongitudes[i]))
        }
        return correctedLatLngList
    }
}