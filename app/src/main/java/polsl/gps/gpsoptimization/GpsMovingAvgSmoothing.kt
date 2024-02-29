package polsl.gps.gpsoptimization

import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.google.android.gms.maps.model.LatLng
import kotlin.math.abs

class GpsMovingAvgSmoothing(
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

    @RequiresApi(Build.VERSION_CODES.N)
    fun smoothAndEvaluateAndGroup() {
        var groupId = 0
        var isFirst = true
        for (i in latitudes.indices) {
            if (!isGrouped(i)) {
                if(!isFirst)
                    groupId++
                val groupIndices = mutableListOf<Int>()
                val smoothedCoordinate = movingAverage(i, groupIndices)
                smoothedLatitudes[i] = smoothedCoordinate.first
                smoothedLongitudes[i] = smoothedCoordinate.second
                Log.d("koorydanty:", (smoothedLatitudes[i].toString() + smoothedLongitudes[i].toString() + "$i" + "nowa grupd $groupId"))
                // Ewaluacja MAE tylko dla pierwszego punktu w grupie
                errors[i] = calculateMAE(smoothedCoordinate.first, smoothedCoordinate.second, groupId)
                if(isFirst) isFirst = false
                // Dodawanie grupy do mapy
                groups[groupId] = groupIndices

            }
            else{
                val groupId = groups.entries.find { it.value == whichGroup(i) }?.key
                val smoothedCoordinate = movingAverage(i, whichGroup(i))
                smoothedLatitudes[i] = smoothedCoordinate.first
                smoothedLongitudes[i] = smoothedCoordinate.second
                Log.d("koorydanty:", (smoothedLatitudes[i].toString() + smoothedLongitudes[i].toString() + "$i" + " $groupId"))
                errors[i] = calculateMAE(smoothedCoordinate.first, smoothedCoordinate.second, groupId)
            }

        }
    }

    private fun movingAverage(index: Int, groupIndices: MutableList<Int>): Pair<Double, Double> {
        var sumLat = 0.0
        var sumLon = 0.0
        var count = 0

        // Obliczanie średniej wartości punktów w grupie
        for (i in latitudes.indices) {
            if (isNearby(latitudes[index], longitudes[index], latitudes[i], longitudes[i])) {
                sumLat += latitudes[i]
                sumLon += longitudes[i]
                count++
                groupIndices.add(i)
            }
        }

        val smoothedLatitude = sumLat / count
        val smoothedLongitude = sumLon / count
        Log.d("koord:", "$smoothedLatitude" + " $smoothedLongitude" +" $index")
        return Pair(smoothedLatitude, smoothedLongitude)
    }
    private fun isNearby(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Boolean {
        val latDiff = abs(lat1 - lat2)
        val lonDiff = abs(lon1 - lon2)
        return latDiff < threshold && lonDiff < threshold
    }

    private fun isGrouped(index: Int): Boolean {
        groups.values.forEach { groupIndices ->
            if (index in groupIndices) {
                return true
            }
        }
        return false
    }
    private fun whichGroup(index: Int): MutableList<Int>{
        groups.values.forEach { groupIndices ->
            if (index in groupIndices) {
                return groupIndices
            }
        }
        return mutableListOf<Int>()
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
        Log.d("Index", index.toString())
        val trueLatitude = trueLatitudes[index!!]
        val trueLongitude = trueLongitudes[index!!]
        return abs(smoothedLatitude - trueLatitude) + abs(smoothedLongitude - trueLongitude)
    }
    fun getMAE(): DoubleArray {
        return errors
    }
}