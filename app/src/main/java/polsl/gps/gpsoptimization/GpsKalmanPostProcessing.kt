package polsl.gps.gpsoptimization
import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.math.*

class GpsKalmanPostProcessing(
    private val latitudes: DoubleArray, private val longitudes: DoubleArray,
    private val accelerationsX: MutableList<Double>, private val accelerationsY: MutableList<Double>,
    private val threshold: Double, private val trueLatitudes: DoubleArray,
    private val trueLongitudes: DoubleArray, private val timeStamps: MutableList<Long>,
    private val Q_metres_per_second: Float, private val accuracies: MutableList<Float>,
    private val azimuths: MutableList<Float>
) {
    private val MinAccuracy = 1f
    private var TimeStamp_milliseconds: Long = 0
    private var lat = 0.0
    private var lng = 0.0
    private var variance = -1f // P matrix.  Negative means object uninitialised.  NB: units irrelevant, as long as same units used throughout
    private val correctedLatitudes = DoubleArray(latitudes.size)
    private val correctedLongitudes = DoubleArray(longitudes.size)
    private val groups = mutableMapOf<Int, MutableList<Int>>()
    private val errors = DoubleArray(latitudes.size)
    private var groupIndexMap = mutableMapOf<Int, Int>()
    private var prevAccX: Double = 0.0
    private var prevAccY: Double = 0.0
    private var velocityVariance = 1.0 // Initial variance
    private var maxGroupSize: Int = 6
    @RequiresApi(Build.VERSION_CODES.N)
    fun smoothAndEvaluateAndGroup() {
        createWindows(1000)
        for (i in latitudes.indices) {
            val correctedCoordinate = process(latitudes[i], longitudes[i], accuracies[i], timeStamps[i],
                accelerationsX[i], accelerationsY[i], azimuths[i])
            correctedLatitudes[i] = correctedCoordinate.first
            correctedLongitudes[i] = correctedCoordinate.second
            val groupId = groupIndexMap[i]
            //errors[i] = calculateMAE(smoothedCoordinate.first, smoothedCoordinate.second, groupId)
            if (groupId != null) {
                groups.computeIfAbsent(groupId) { mutableListOf() }.add(i)
            }
        }
    }
/*    fun get_TimeStamp(): Long {
        return TimeStamp_milliseconds
    }

    fun get_lat(): Double {
        return lat
    }

    fun get_lng(): Double {
        return lng
    }

    fun get_accuracy(): Float {
        return kotlin.math.sqrt(variance.toDouble()).toFloat()
    }
    fun SetState(lat: Double, lng: Double, accuracy: Float, TimeStamp_milliseconds: Long) {
        this.lat = lat
        this.lng = lng
        variance = accuracy * accuracy
        this.TimeStamp_milliseconds = TimeStamp_milliseconds
    }*/

    /**
     * Kalman filter processing for latitude and longitude
     * @param lat_measurement new measurement of latitude
     * @param lng_measurement new measurement of longitude
     * @param accuracy measurement of 1 standard deviation error in metres
     * @param TimeStamp_milliseconds time of measurement
     * @return new state
     */
    private fun process(lat_measurement: Double, lng_measurement: Double, accuracy: Float,
                        TimeStamp_milliseconds: Long, acceleration_x: Double, acceleration_y: Double,
                        azimuth: Float)
    :Pair<Double, Double>
    {
        var velocity_lat = 0.0
        var velocity_lng = 0.0

        var accuracy = accuracy
        if (accuracy < MinAccuracy) accuracy = MinAccuracy
        if (variance < 0) {
            // if variance < 0, object is uninitialised, so initialise with current values
            this.TimeStamp_milliseconds = TimeStamp_milliseconds
            lat = lat_measurement
            lng = lng_measurement
            variance = accuracy * accuracy
            velocityVariance = variance.toDouble()
            prevAccX = acceleration_x
            prevAccY = acceleration_y


        } else {
            // else apply Kalman filter methodology
            val timeIncMilliseconds = TimeStamp_milliseconds - this.TimeStamp_milliseconds
            if (timeIncMilliseconds > 0) {
                // time has moved on, so the uncertainty in the current position increases
                variance += timeIncMilliseconds * Q_metres_per_second * Q_metres_per_second / 1000
                this.TimeStamp_milliseconds = TimeStamp_milliseconds
                // TO DO: USE VELOCITY INFORMATION HERE TO GET A BETTER ESTIMATE OF CURRENT POSITION

                // Obliczanie przyspieszenia na podstawie danych z akcelerometru
                val dtSeconds = timeIncMilliseconds / 1000.0 // Czas w sekundach
                // Obliczanie prędkości na podstawie poprzedniego i aktualnego przyspieszenia
                velocity_lat += (prevAccX + acceleration_x) / 2 * dtSeconds
                velocity_lng += (prevAccY + acceleration_y) / 2 * dtSeconds
                prevAccX = acceleration_x
                prevAccY = acceleration_y
                val metersPerDegree = 111_000
                val azimuthRadians = Math.toRadians(azimuth.toDouble())

                // Konwersja prędkości w m/s na zmianę stopni na sekundę
                val delta_lat = (velocity_lat / metersPerDegree) / 3600.0 // Prędkość w metrach na sekundę dzielona przez metry na stopień, a następnie przez 3600 sekund w godzinie
                val delta_lng = (velocity_lng / (metersPerDegree * Math.cos(Math.toRadians(lat))))/ 3600.0
                // Przesunięcie kierunku ruchu w oparciu o azymut
                val adjustedDeltaLat = delta_lat * cos(azimuthRadians) - delta_lng * Math.sin(azimuthRadians)
                val adjustedDeltaLng = delta_lat * Math.sin(azimuthRadians) + delta_lng * Math.cos(azimuthRadians)
                // Prędkość w metrach na sekundę, uwzględniając kosinus szerokości geograficznej
                lat += adjustedDeltaLat
                lng += adjustedDeltaLng
            }
            // Kalman gain matrix K = Covarariance * Inverse(Covariance + MeasurementVariance)
            // NB: because K is dimensionless, it doesn't matter that variance has different units to lat and lng
            val K = variance / (variance + accuracy * accuracy)
            // apply K
            lat += K * (lat_measurement - lat)
            lng += K * (lng_measurement - lng)
            // new Covarariance  matrix is (IdentityMatrix - K) * Covarariance
            variance *= (1 - K)
        }
        Log.d("Zwracane z kalmana", "lat: $lat long: $lng")
        return Pair(lat, lng)
    }
    // Funkcja do obliczania przyspieszenia na podstawie danych z akcelerometru
    /*fun calculateAcceleration(
        accelerationData: List<Triple<Double, Double, Double>>, // Lista odczytów przyspieszenia dla każdej osi (x, y, z)
        timeStamps: List<Long> // Lista znaczników czasowych odpowiadających każdemu odczytowi
    ): List<Triple<Double, Double, Double>> {
        val accelerations = mutableListOf<Triple<Double, Double, Double>>()

        for (i in 1 until accelerationData.size) {
            val (x1, y1, z1) = accelerationData[i - 1]
            val (x2, y2, z2) = accelerationData[i]

            val t1 = timeStamps[i - 1].toDouble()
            val t2 = timeStamps[i].toDouble()

            val accelerationX = (x2 - x1) / (t2 - t1)
            val accelerationY = (y2 - y1) / (t2 - t1)
            val accelerationZ = (z2 - z1) / (t2 - t1)

            accelerations.add(Triple(accelerationX, accelerationY, accelerationZ))
        }

        return accelerations
    }*/

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