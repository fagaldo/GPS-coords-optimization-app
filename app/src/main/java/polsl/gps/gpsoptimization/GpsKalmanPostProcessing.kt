package polsl.gps.gpsoptimization

import android.os.Build
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.math.*

/**
 * Klasa GpsKalmanPostProcessing implementuje przetwarzanie danych GPS z wykorzystaniem filtru Kalmana.
 * Odpowiedzialna za poprawianie pozycji GPS oraz grupowanie punktów na podstawie podobieństwa.
 *
 * @param latitudes Tablica szerokości geograficznych.
 * @param longitudes Tablica długości geograficznych.
 * @param accelerationsX Lista wartości przyspieszenia w osi X.
 * @param accelerationsY Lista wartości przyspieszenia w osi Y.
 * @param threshold Próg podobieństwa geograficznego do grupowania punktów.
 * @param timeStamps Lista znaczników czasowych dla każdej lokalizacji.
 * @param Q_metres_per_second Szum procesu modelowanego w filtrze Kalmana.
 * @param accuracies Lista dokładności pomiarów GPS.
 * @param azimuths Lista wartości azymutu (kierunku).
 */
class GpsKalmanPostProcessing(
    private val latitudes: DoubleArray,
    private val longitudes: DoubleArray,
    private val accelerationsX: MutableList<Double>,
    private val accelerationsY: MutableList<Double>,
    private val threshold: Double,
    private val timeStamps: MutableList<Long>,
    private val Q_metres_per_second: Float,
    private val accuracies: MutableList<Float>,
    private val azimuths: MutableList<Float>
) {
    /**
     * Minimalna dokładność GPS.
     */
    private val MinAccuracy = 1f

    /**
     * Aktualny znacznik czasu w milisekundach.
     */
    private var TimeStamp_milliseconds: Long = 0

    /**
     * Aktualne wartości szerokości i długości geograficznej.
     */
    private var lat = 0.0
    private var lng = 0.0

    /**
     * Wariancja filtru Kalmana (macierz P).
     */
    private var variance = -1f

    /**
     * Poprawione szerokości geograficzne.
     */
    private val correctedLatitudes = DoubleArray(latitudes.size)

    /**
     * Poprawione długości geograficzne.
     */
    private val correctedLongitudes = DoubleArray(longitudes.size)

    /**
     * Grupy punktów podobnych.
     */
    private val groups = mutableMapOf<Int, MutableList<Int>>()

    /**
     * Mapa indeksów grup.
     */
    private var groupIndexMap = mutableMapOf<Int, Int>()

    /**
     * Wartości poprzednich przyspieszeń w osi X.
     */
    private var prevAccX: Double = 0.0

    /**
     * Wartości poprzednich przyspieszeń w osi Y.
     */
    private var prevAccY: Double = 0.0

    /**
     * Wariancja prędkości.
     */
    private var velocityVariance = 1.0

    /**
     * Maksymalny rozmiar grupy.
     */
    private var maxGroupSize: Int = 10

    /**
     * Wykonuje wygładzanie i ocenę pozycji GPS oraz grupowanie punktów.
     */
    @RequiresApi(Build.VERSION_CODES.N)
    fun smoothAndEvaluateAndGroup() {
        createWindows(1000)
        for (i in latitudes.indices) {
            val correctedCoordinate = process(
                latitudes[i],
                longitudes[i],
                accuracies[i],
                timeStamps[i],
                accelerationsX[i],
                accelerationsY[i],
                azimuths[i]
            )
            correctedLatitudes[i] = correctedCoordinate.first
            correctedLongitudes[i] = correctedCoordinate.second
            val groupId = groupIndexMap[i]
            if (groupId != null) {
                groups.computeIfAbsent(groupId) { mutableListOf() }.add(i)
            }
        }
    }

    /**
     * Przetwarza dane GPS z wykorzystaniem filtru Kalmana.
     *
     * @param lat_measurement Nowy pomiar szerokości geograficznej.
     * @param lng_measurement Nowy pomiar długości geograficznej.
     * @param accuracy Dokładność pomiaru.
     * @param TimeStamp_milliseconds Znacznik czasu pomiaru.
     * @param acceleration_x Przyspieszenie w osi X.
     * @param acceleration_y Przyspieszenie w osi Y.
     * @param azimuth Azymut (kierunek).
     * @return Zaktualizowana pozycja GPS jako para (szerokość, długość).
     */
    private fun process(
        lat_measurement: Double,
        lng_measurement: Double,
        accuracy: Float,
        TimeStamp_milliseconds: Long,
        acceleration_x: Double,
        acceleration_y: Double,
        azimuth: Float
    ): Pair<Double, Double> {
        var velocity_lat = 0.0
        var velocity_lng = 0.0

        var accuracy = accuracy
        if (accuracy < MinAccuracy) accuracy = MinAccuracy
        if (variance < 0) {
            // Inicjalizacja obiektu, jeśli wariancja jest mniejsza od 0
            this.TimeStamp_milliseconds = TimeStamp_milliseconds
            lat = lat_measurement
            lng = lng_measurement
            variance = accuracy * accuracy
            velocityVariance = variance.toDouble()
            prevAccX = acceleration_x
            prevAccY = acceleration_y
        } else {
            // Zastosowanie metodologii filtru Kalmana
            val timeIncMilliseconds = TimeStamp_milliseconds - this.TimeStamp_milliseconds
            if (timeIncMilliseconds > 0) {
                // Zwiększenie niepewności pozycji w czasie
                variance += timeIncMilliseconds * Q_metres_per_second * Q_metres_per_second / 1000
                this.TimeStamp_milliseconds = TimeStamp_milliseconds
                val dtSeconds = timeIncMilliseconds / 1000.0

                velocity_lat += (prevAccX + acceleration_x) / 2 * dtSeconds
                velocity_lng += (prevAccY + acceleration_y) / 2 * dtSeconds
                prevAccX = acceleration_x
                prevAccY = acceleration_y
                val metersPerDegree = 111_000
                val azimuthRadians = Math.toRadians(azimuth.toDouble())

                val delta_lat = (velocity_lat / metersPerDegree) / 3600.0
                val delta_lng = (velocity_lng / (metersPerDegree * Math.cos(Math.toRadians(lat)))) / 3600.0

                val adjustedDeltaLat = delta_lat * cos(azimuthRadians) - delta_lng * Math.sin(azimuthRadians)
                val adjustedDeltaLng = delta_lat * Math.sin(azimuthRadians) + delta_lng * Math.cos(azimuthRadians)

                lat += adjustedDeltaLat
                lng += adjustedDeltaLng
            }

            val K = variance / (variance + accuracy * accuracy)

            lat += K * (lat_measurement - lat)
            lng += K * (lng_measurement - lng)

            variance *= (1 - K)
        }
        return Pair(lat, lng)
    }

    /**
     * Tworzy okna czasowe do grupowania punktów na podstawie ich podobieństwa.
     *
     * @param timeThreshold Próg czasowy dla grupowania.
     * @return Lista identyfikatorów okien.
     */
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
                if (i != j && isSimilar(latitudes[i], longitudes[i], latitudes[j], longitudes[j], threshold)
                    && isSimilarTime(timeStamps[i], timeStamps[j], 3000)) {

                    val groupI = groupIndexMap[i]
                    val groupJ = groupIndexMap[j]

                    if (groupI == null && groupJ == null) {
                        groupIndexMap[i] = currentGroupIndex
                        groupIndexMap[j] = currentGroupIndex
                        indexesInWindows.add(i)
                        indexesInWindows.add(j)
                        windows.add(currentGroupIndex)
                        groupSizes[currentGroupIndex] = 2
                        currentGroupIndex++
                    } else if (groupI != null && groupJ == null) {
                        if (groupSizes[groupI]!! < maxGroupSize) {
                            groupIndexMap[j] = groupI
                            indexesInWindows.add(j)
                            groupSizes[groupI] = groupSizes[groupI]!! + 1
                        }
                    } else if (groupI == null && groupJ != null) {
                        if (groupSizes[groupJ]!! < maxGroupSize) {
                            groupIndexMap[i] = groupJ
                            indexesInWindows.add(i)
                            groupSizes[groupJ] = groupSizes[groupJ]!! + 1
                        }
                    }
                }
            }
        }

        ungroupedIndexes.addAll((latitudes.indices).filter { !indexesInWindows.contains(it) })

        var diff = 5.0
        var oldDiff = diff
        var timeDiff: Long = 2000
        var oldTimeDiff = timeDiff
        var iteration = 0

        while (ungroupedIndexes.isNotEmpty()) {
            val iterator = ungroupedIndexes.iterator()
            val foundIndex = mutableListOf<Int>()

            while (iterator.hasNext()) {
                val itIndex = iterator.next()
                var isAssigned = false

                for (tmp in indexesInWindows) {
                    if (iteration % 1000 == 0)
                        maxGroupSize++
                    var group: Int? = null
                    for (tmp1 in indexesInWindows) {
                        diff = similarityVal(latitudes[itIndex], longitudes[itIndex], latitudes[tmp1], longitudes[tmp1])
                        if (diff < oldDiff && groupSizes[groupIndexMap[tmp1]]!! < maxGroupSize) {
                            oldDiff = diff
                            group = groupIndexMap[tmp1]
                        }
                    }
                    if (group != null && groupSizes[group]!! < maxGroupSize) {
                        groupIndexMap[itIndex] = group
                        indexesInWindows.add(itIndex)
                        foundIndex.add(itIndex)
                        groupSizes[group] = groupSizes[group]!! + 1
                        iterator.remove()
                        isAssigned = true
                        break
                    }

                    if (!isAssigned) {
                        for (tmp2 in indexesInWindows) {
                            timeDiff = similarityTimeVal(timeStamps[itIndex], timeStamps[tmp2])
                            if (timeDiff < oldTimeDiff && groupSizes[groupIndexMap[tmp2]]!! < maxGroupSize) {
                                oldTimeDiff = timeDiff
                                group = groupIndexMap[tmp2]
                            }
                        }
                        if (group != null && (groupSizes[group]!! < maxGroupSize)) {
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
                if (oldTimeDiff > 10000) {
                    oldTimeDiff = 2000
                    incrementTimeRate = 100
                }
            }
            iteration++
            incrementRate += 0.001
            incrementTimeRate += 100
        }
        groupSizes.forEach { (group, size) ->
        }
        return windows
    }

    /**
     * Oblicza różnicę czasu między dwoma punktami.
     *
     * @param time1 Czas pierwszego punktu.
     * @param time2 Czas drugiego punktu.
     * @return Różnica czasu jako wartość Long.
     */
    private fun similarityTimeVal(time1: Long, time2: Long): Long {
        return abs(time1 - time2)
    }

    /**
     * Sprawdza, czy dwa punkty GPS są podobne pod względem czasu.
     *
     * @param time1 Czas pierwszego punktu.
     * @param time2 Czas drugiego punktu.
     * @param threshold Próg podobieństwa czasowego.
     * @return True, jeśli punkty są podobne pod względem czasu.
     */
    private fun isSimilarTime(time1: Long, time2: Long, threshold: Long): Boolean {
        return abs(time1 - time2) < threshold
    }

    /**
     * Sprawdza, czy dwa punkty GPS są podobne pod względem współrzędnych.
     *
     * @param lat1 Szerokość geograficzna pierwszego punktu.
     * @param lon1 Długość geograficzna pierwszego punktu.
     * @param lat2 Szerokość geograficzna drugiego punktu.
     * @param lon2 Długość geograficzna drugiego punktu.
     * @param threshold Próg podobieństwa współrzędnych.
     * @return True, jeśli punkty są podobne pod względem współrzędnych.
     */
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

    /**
     * Oblicza odległość między dwoma punktami GPS.
     *
     * @param lat1 Szerokość geograficzna pierwszego punktu.
     * @param lon1 Długość geograficzna pierwszego punktu.
     * @param lat2 Szerokość geograficzna drugiego punktu.
     * @param lon2 Długość geograficzna drugiego punktu.
     * @return Odległość między punktami w kilometrach.
     */
    private fun similarityVal(
        lat1: Double, lon1: Double,
        lat2: Double, lon2: Double
    ): Double {
        val R = 6371.0 // Średni promień Ziemi w kilometrach

        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c
    }

    /**
     * Zwraca mapę grup punktów GPS.
     *
     * @return Mapa grup punktów GPS.
     */
    fun getGroups(): Map<Int, List<Int>> {
        return groups
    }

    /**
     * Zwraca listę poprawionych współrzędnych GPS.
     *
     * @return Lista poprawionych współrzędnych GPS jako obiekty LatLng.
     */
    fun getCorrectedLatLngList(): List<LatLng> {
        val correctedLatLngList = mutableListOf<LatLng>()
        for (i in correctedLatitudes.indices) {
            correctedLatLngList.add(LatLng(correctedLatitudes[i], correctedLongitudes[i]))
        }
        return correctedLatLngList
    }
}
