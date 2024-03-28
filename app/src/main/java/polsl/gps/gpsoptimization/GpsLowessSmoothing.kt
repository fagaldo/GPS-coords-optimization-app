package polsl.gps.gpsoptimization
import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.collections.mutableMapOf
import kotlin.math.*

class GpsLowessSmoothing(private val latitudes: DoubleArray, private val longitudes: DoubleArray, private var bandwidth: Double, private val trueLatitudes: DoubleArray,
                         private val trueLongitudes: DoubleArray, private val timeStamps: MutableList<Long>) {
    private val smoothedLatitudes = DoubleArray(latitudes.size)
    private val smoothedLongitudes = DoubleArray(longitudes.size)
    private val groups = mutableMapOf<Int, MutableList<Int>>()
    private val errors = DoubleArray(latitudes.size)
    private var windowIndexMap = mutableMapOf<Int, Int>()
    @RequiresApi(Build.VERSION_CODES.N)
    fun smoothAndEvaluateAndGroup(threshold: Double) {
        //val windowSize = calculateWindowSize(latitudes, longitudes)
        createWindows(threshold, 1000)
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
    private fun createWindows(threshold: Double, timeThreshold: Long): List<Int>{
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
            //Sprawdzamy czas dopóki jest podobny
            //TODO
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
            if(index == 0){
                lastIndex = 1
                index = 1
                Log.d("index", 0.toString())
                if(isSimilar(latitudes[index-1], longitudes[index-1], latitudes[lastIndex], longitudes[lastIndex], threshold) && lastIndex !in indexesInWindows)
                {
                    windowIndexMap[lastIndex] = windowIndex
                    indexesInWindows.add(lastIndex)
                }
                //index = 0
            }
            else{
                lastIndex = index
                if(isSimilar(latitudes[index-1], longitudes[index-1], latitudes[lastIndex], longitudes[lastIndex], threshold) && lastIndex !in indexesInWindows)
                {
                    windowIndexMap[lastIndex] = windowIndex
                    indexesInWindows.add(lastIndex)
                }
            }

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
                            val group = windowIndexMap[tmp]
                            if (group != null) {
                                windowIndexMap[it] = group
                                foundIndex.add(it)
                            }
                        }

                    }
                    if(it !in foundIndex && isSimilarTime(timeStamps[it], timeStamps[tmp], timeThreshold + incrementTimeRate)){
                        //Log.d("Dorzucam", "na podstawie czasu dla $it")
                        val group = windowIndexMap[tmp]
                        if (group != null) {
                            windowIndexMap[it] = group
                            foundIndex.add(it)
                        }
                    }
                }
                oldDiff = 1.0
                //if(isFound) {break}
            }
            ungroupedIndexes.removeAll(foundIndex)
            incrementRate+=incrementRate
            incrementTimeRate+=incrementTimeRate
        }

        return windows
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
