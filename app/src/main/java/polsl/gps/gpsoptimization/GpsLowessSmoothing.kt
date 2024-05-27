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
        val ungroupedIndexes = mutableListOf<Int>()
        var incrementRate = 0.01
        var incrementTimeRate: Long = 10

        var windowSize = 1 // Rozmiar aktualnego okna
        var currentGroupIndex = 0
        for (i in 0 until latitudes.size) {
            for (j in 0 until latitudes.size) {
                Log.d("Porownanie", "Porownuje i: $i z j: $j")
                if (isSimilar(latitudes[i], longitudes[i], latitudes[j], longitudes[j], threshold)
                    && isSimilarTime(timeStamps[i], timeStamps[j], 2000) && i!=j) {
                    Log.d("Podobienstwo", "Znaleziona podobna para: ($i, $j)")

                    // Sprawdzamy, czy którykolwiek z indeksów jest już w jakiejś grupie
                    val groupI = windowIndexMap[i]
                    val groupJ = windowIndexMap[j]

                    if (groupI == null && groupJ == null) {
                        // Żaden z punktów nie jest jeszcze w grupie, tworzymy nową grupę
                        windowIndexMap[i] = currentGroupIndex
                        windowIndexMap[j] = currentGroupIndex
                        indexesInWindows.add(i)
                        indexesInWindows.add(j)
                        windowSize += 2
                        windows.add(windowSize)
                        Log.d("Tworze grupe", "Index: $i, $j, Grupa: $currentGroupIndex")
                        currentGroupIndex++
                    } else if (groupI != null && groupJ == null) {
                        // i jest w grupie, dodajemy j do tej samej grupy
                        windowIndexMap[j] = groupI
                        indexesInWindows.add(j)
                        windowSize++
                        windows.add(windowSize)
                        Log.d("Dodaje do grupy", "Index: $j, Grupa: $groupI")
                    } else if (groupI == null && groupJ != null) {
                        // j jest w grupie, dodajemy i do tej samej grupy
                        windowIndexMap[i] = groupJ
                        indexesInWindows.add(i)
                        windowSize++
                        windows.add(windowSize)
                        Log.d("Dodaje do grupy", "Index: $i, Grupa: $groupJ")
                    } else {
                        // Oba punkty są już w grupach
                        Log.d("Oba punkty w grupach", "Index: $i, Grupa: $groupI, Index: $j, Grupa: $groupJ")
                    }
                }
            }
        }

// Dodaj wszystkie nieprzypisane indeksy do ungroupedIndexes
        ungroupedIndexes.addAll((latitudes.indices).filter { !indexesInWindows.contains(it) })

        var diff = 5.0
        var oldDiff = diff

        while (ungroupedIndexes.isNotEmpty()) {
            Log.d("Oprozniamy", ungroupedIndexes.toString())
            Log.d("Time thresh", (timeThreshold + incrementTimeRate).toString())

            val iterator = ungroupedIndexes.iterator()
            while (iterator.hasNext()) {
                val itIndex = iterator.next()
                var isAssigned = false

                for (tmp in indexesInWindows) {
                    // Sprawdzenie podobieństwa geograficznego i czasowego
                    if (isSimilar(latitudes[itIndex], longitudes[itIndex], latitudes[tmp], longitudes[tmp], threshold + incrementRate)
                        && isSimilarTime(timeStamps[itIndex], timeStamps[tmp], 1000)) {

                        // Obliczenie różnicy dla znalezionego podobnego indeksu
                        diff = similarityVal(latitudes[itIndex], longitudes[itIndex], latitudes[tmp], longitudes[tmp])
                        if (diff < oldDiff) {
                            oldDiff = diff
                            Log.d("DorzucamG", "na podstawie geografii dla $itIndex")
                            val group = windowIndexMap[tmp]
                            if (group != null) {
                                windowIndexMap[itIndex] = group
                                indexesInWindows.add(itIndex) // Dodaj do indeksów w oknach
                                iterator.remove() // Usuń przetworzony indeks z ungroupedIndexes
                                isAssigned = true
                                break
                            }
                        }
                    } else if (!isAssigned && isSimilarTime(timeStamps[itIndex], timeStamps[tmp], timeThreshold + incrementTimeRate)) {
                        // Dodanie na podstawie czasu, jeśli nie znaleziono na podstawie podobieństwa geograficznego
                        Log.d("Dorzucam", "na podstawie czasu dla $itIndex")
                        val group = windowIndexMap[tmp]
                        if (group != null) {
                            windowIndexMap[itIndex] = group
                            indexesInWindows.add(itIndex) // Dodaj do indeksów w oknach
                            iterator.remove() // Usuń przetworzony indeks z ungroupedIndexes
                            isAssigned = true
                            break
                        }
                    }
                }

                oldDiff = 5.0 // Resetowanie oldDiff po przetworzeniu każdego indeksu
            }

            // Zwiększanie wartości progów
            incrementRate += incrementRate
            incrementTimeRate += incrementTimeRate

            Log.d("XD", incrementRate.toString())
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
