/*
package polsl.gps.gpsoptimization

import android.os.Build.VERSION_CODES.P
import android.os.Build.VERSION_CODES.Q
import com.google.gson.GsonBuilder
import org.jetbrains.kotlinx.multik.api.*
import org.jetbrains.kotlinx.multik.ndarray.data.D2
import org.jetbrains.kotlinx.multik.ndarray.data.D2Array
import org.jetbrains.kotlinx.multik.ndarray.data.get
import org.jetbrains.kotlinx.multik.ndarray.data.set
import org.jetbrains.kotlinx.multik.ndarray.operations.minus
import org.jetbrains.kotlinx.multik.ndarray.operations.plus
import org.jetbrains.kotlinx.multik.ndarray.operations.reversed
import org.jetbrains.kotlinx.multik.ndarray.operations.times
import java.io.File
import kotlin.math.*

const val EARTH_RADIUS = 6371 * 1000.0
const val FOUR_SPACES = "    "
const val ACTUAL_GRAVITY = 9.80665

class ProperKalman(val Timestamp: Double,
                   val GpsLat: Double,
                   val GpsLon: Double,
                   val GpsAlt: Double,
                   val AbsNorthAcc: Float,
                   val AbsEastAcc: Float,
                   val AbsUpAcc: Float,
                   val VelNorth: Double,
                   val VelEast: Double,
                   val VelDown: Double,
                   val VelError: Double,
                   val AltitudeError: Double) {

    data class LatLng(
        val Latitude: Double,
        val Longitude: Double
    )

    private fun radiansToDegrees(someting: Double): Degrees {
        return someting * 180.0 / PI
    }

    fun getPointAhead(fromCoordinate: LatLng, distanceMeters: Double, azimuth: Degrees): LatLng {
        val radiusFraction = distanceMeters / EARTH_RADIUS
        val bearing = DegreesToRadians(azimuth)
        val lat1 = geoAngle(fromCoordinate.Latitude)
        val lng1 = geoAngle(fromCoordinate.Longitude)
        val lat2_part1 = sin(lat1) * cos(radiusFraction)
        val lat2_part2 = cos(lat1) * sin(radiusFraction) * cos(bearing)
        val lat2 = asin(lat2_part1 + lat2_part2)
        val lng2_part1 = sin(bearing) * sin(radiusFraction) * cos(lat1)
        val lng2_part2 = cos(radiusFraction) - (sin(lat1) * sin(lat2))
        var lng2 = lng1 + atan2(lng2_part1, lng2_part2)
        lng2 = (lng2 + 3 * PI) % (2 * PI) - PI
        return LatLng(
            Latitude = radiansToDegrees(lat2),
            Longitude = radiansToDegrees(lng2)
        )
    }

    private fun DegreesToRadians(azimuth: Degrees): Double {
        return azimuth * PI / 180.0
    }

    fun pointPlusDistanceEast(fromCoordinate: LatLng, distance: Double): LatLng {
        return getPointAhead(fromCoordinate, distance, 90.0)
    }

    fun pointPlusDistanceNorth(fromCoordinate: LatLng, distance: Double): LatLng {
        return getPointAhead(fromCoordinate, distance, 0.0)
    }

    fun metersToGeopoint(latAsMeters: Double, lonAsMeters: Double): LatLng {
        val point = LatLng(0.0, 0.0)
        val pointEast = pointPlusDistanceEast(point, lonAsMeters)
        val pointNorthEast = pointPlusDistanceNorth(pointEast, latAsMeters)
        return pointNorthEast
    }


    fun geoAngle(latOrLon: Double): Double {
        return DegreesToRadians(latOrLon)
    }

    fun getDistanceMeters(fromCoordinate: LatLng, toCoordinate: LatLng): Double {
        val deltaLon = geoAngle(toCoordinate.Longitude - fromCoordinate.Longitude)
        val deltaLat = geoAngle(toCoordinate.Latitude - fromCoordinate.Latitude)
        val a = sin(deltaLat / 2.0).pow(2) +
                cos(geoAngle(fromCoordinate.Latitude)) *
                cos(geoAngle(toCoordinate.Latitude)) *
                sin(deltaLon / 2.0).pow(2)
        val c = 2 * atan2(sqrt(a), sqrt(1.0 - a))
        return EARTH_RADIUS * c
    }

    fun latitudeToMeters(latitude: Double): Double {
        val distance = getDistanceMeters(
            LatLng(latitude, 0.0),
            LatLng(0.0, 0.0)
        )
        return if (latitude < 0) -distance else distance
    }

    fun longitudeToMeters(longitude: Double): Double {
        val distance = getDistanceMeters(
            LatLng(0.0, longitude),
            LatLng(0.0, 0.0)
        )
        return if (longitude < 0) -distance else distance
    }

    data class SensorData(
        val Timestamp: Double,
        val GpsLat: Double,
        val GpsLon: Double,
        val GpsAlt: Double,
        val AbsNorthAcc: Float,
        val AbsEastAcc: Float,
        val AbsUpAcc: Float,
        val VelNorth: Double,
        val VelEast: Double,
        val VelDown: Double,
        val VelError: Double,
        val AltitudeError: Double
    )

    data class OutputPacket(
        val sensorData: SensorData,
        val PredictedLat: Double,
        val PredictedLon: Double,
        val PredictedAlt: Double,
        val ResultantMPH: Double,
        val GPSLat: Double,
        val GPSLon: Double
    )

    val u = Array(2){ Array<Int>(2) {0}}


    fun writeJsonSerializableToFile(jsonEntity: Any, filename: String) {
        val gson = GsonBuilder().setPrettyPrinting().create()
        val serialized = gson.toJson(jsonEntity)
        File(filename).writeText(serialized)
    }

    class KalmanFilterFusedPositionAccelerometer(
        private val I: D2Array<Double>,
        private val H: D2Array<Double>,
        private var P: D2Array<Double>,
        private val Q: D2Array<Double>,
        private val R: D2Array<Double>,
        private val u: D2Array<Double>,
        private val z: D2Array<Double>,
        private val A: D2Array<Double>,
        private val B: D2Array<Double>,
        private var currentState: D2Array<Double>,
        private var currentStateTimestampSeconds: Double = 0.0
    ) {
        fun predict(accelerationThisAxis: Double, timestampNow: Double) {
            val deltaT = timestampNow - currentStateTimestampSeconds
            recreateControlMatrix(deltaT)
            recreateStateTransitionMatrix(deltaT)
            u[0, 0] = accelerationThisAxis
            currentState = (A * currentState) + (B * u)
            P = ((A * P) * A.transpose()) + Q
            currentStateTimestampSeconds = timestampNow
        }

        fun update(position: Double, velocityThisAxis: Double, positionError: Double?, velocityError: Double) {
            z[0, 0] = position
            z[0, 0] = velocityThisAxis
            if (positionError != null) {
                R[0, 0] = positionError * positionError
            }
            R[1, 1] = velocityError * velocityError
            val y = z - currentState
            val s = P + R
            val sInverse = s.reversed() ?: return
            val K = P * sInverse
            currentState += K * y
            P = (I - K) * P
        }

        private fun recreateControlMatrix(deltaSeconds: Double) {
            val dtSquared = 0.5 * deltaSeconds * deltaSeconds
            B[0, 0] = dtSquared
            B[1, 0] = deltaSeconds
        }

        private fun recreateStateTransitionMatrix(deltaSeconds: Double) {
            A[0, 0] = 1.0
            A[0, 1] = deltaSeconds
            A[1, 0] = 0.0
            A[1, 1] = 1.0
        }

        fun getPredictedPosition(): Double {
            return currentState[0, 0]
        }

        fun getPredictedVelocityThisAxis(): Double {
            return currentState[1, 0]
        }
    }

    fun main() {
        val collection = readFileAsJson("pos_final.json")
        //val collection = this
        val initialSensorData = collection[0]
        val latLonStandardDeviation = 2.0
        val altitudeStandardDeviation = 3.518522417151836

        val accelerometerEastStandardDeviation = ACTUAL_GRAVITY * 0.033436506994600976
        val accelerometerNorthStandardDeviation = ACTUAL_GRAVITY * 0.05355371135598354
        val accelerometerUpStandardDeviation = ACTUAL_GRAVITY * 0.2088683796078286
        val longitudeEastKalmanFilter = KalmanFilterFusedPositionAccelerometer(
            I = mk.identity<Double>(2),
            H = mk.identity<Double>(2),
            P = mk.identity<Double>(2),
            Q = mk.identity<Double>(2).apply {
                this[0, 0] = accelerometerEastStandardDeviation * accelerometerEastStandardDeviation
                this[1, 1] = accelerometerEastStandardDeviation * accelerometerEastStandardDeviation },
            R = mk.identity<Double>(2).apply {
                this[0, 0] = latLonStandardDeviation * latLonStandardDeviation
                this [1, 1] = latLonStandardDeviation * latLonStandardDeviation
            },
            u = mk.zeros(1, 1),
            z = mk.zeros(2, 1),
            A = mk.zeros(2, 2),
            B = mk.zeros(2, 1),
            currentState = mk.zeros<Double>(2,1).apply {
                this[0, 0] = longitudeToMeters(initialSensorData.GpsLon)
                this[1, 0] = initialSensorData.VelEast
            },
            currentStateTimestampSeconds = initialSensorData.Timestamp
        )
        val latitudeNorthKalmanFilter = KalmanFilterFusedPositionAccelerometer(
            I = mk.identity<Double>(2),
            H = mk.identity<Double>(2),
            P = mk.identity<Double>(2),
            Q = mk.identity<Double>(2).apply {
                this[0, 0] = accelerometerNorthStandardDeviation * accelerometerNorthStandardDeviation
                this[1, 1] = accelerometerNorthStandardDeviation * accelerometerNorthStandardDeviation },
            R = mk.identity<Double>(2).apply {
                this[0, 0] = latLonStandardDeviation * latLonStandardDeviation
                this [1, 1] = latLonStandardDeviation * latLonStandardDeviation
            },
            u = mk.zeros(1, 1),
            z = mk.zeros(2, 1),
            A = mk.zeros(2, 2),
            B = mk.zeros(2, 1),
            currentState = mk.zeros<Double>(2,1).apply {
                this[0, 0] = latitudeToMeters(initialSensorData.GpsLon)
                this[1, 0] = initialSensorData.VelNorth
            },
            currentStateTimestampSeconds = initialSensorData.Timestamp
        )
        val altitudeUpKalmanFilter = KalmanFilterFusedPositionAccelerometer(
            I = mk.identity<Double>(2),
            H = mk.identity<Double>(2),
            P = mk.identity<Double>(2),
            Q = mk.identity<Double>(2).apply {
                this[0, 0] = accelerometerUpStandardDeviation * accelerometerUpStandardDeviation
                this[1, 1] = accelerometerUpStandardDeviation * accelerometerUpStandardDeviation },
            R = mk.identity<Double>(2).apply {
                this[0, 0] = altitudeStandardDeviation * altitudeStandardDeviation
                this [1, 1] = altitudeStandardDeviation * altitudeStandardDeviation
            },
            u = mk.zeros(1, 1),
            z = mk.zeros(2, 1),
            A = mk.zeros(2, 2),
            B = mk.zeros(2, 1),
            currentState = mk.zeros<Double>(2,1).apply {
                this[0, 0] = initialSensorData.GpsAlt
                this[1, 0] = initialSensorData.VelDown * -1.0
            },
            currentStateTimestampSeconds = initialSensorData.Timestamp
        )
        val outputs = mutableListOf<OutputPacket>()
        for (i in 1 until collection.size) {
            val data = collection[i]
            longitudeEastKalmanFilter.predict(
                data.AbsEastAcc.toDouble() * ACTUAL_GRAVITY,
                data.Timestamp
            )
            latitudeNorthKalmanFilter.predict(
                data.AbsNorthAcc.toDouble() * ACTUAL_GRAVITY,
                data.Timestamp
            )
            altitudeUpKalmanFilter.predict(
                data.AbsUpAcc.toDouble() * ACTUAL_GRAVITY,
                data.Timestamp
            )
            if (data.GpsLat != 0.0) {
                val defaultPositionErr: Double? = null
                val vEast = data.VelEast
                val longitudeAsMeters = longitudeToMeters(data.GpsLon)
                longitudeEastKalmanFilter.update(
                    longitudeAsMeters,
                    vEast,
                    defaultPositionErr,
                    data.VelError
                )
                val vNorth = data.VelNorth
                val latitudeAsMeters = latitudeToMeters(data.GpsLat)
                latitudeNorthKalmanFilter.update(
                    latitudeAsMeters,
                    vNorth,
                    defaultPositionErr,
                    data.VelError
                )
                val vUp = data.VelDown * -1.0
                altitudeUpKalmanFilter.update(
                    data.GpsAlt,
                    vUp,
                    data.AltitudeError,
                    data.VelError
                )
            }
            val predictedLonMeters = longitudeEastKalmanFilter.getPredictedPosition()
            val predictedLatMeters = latitudeNorthKalmanFilter.getPredictedPosition()
            val predictedAlt = altitudeUpKalmanFilter.getPredictedPosition()
            val point = metersToGeopoint(predictedLatMeters, predictedLonMeters)
            val predictedLon = point.Longitude
            val predictedLat = point.Latitude
            val predictedVE = longitudeEastKalmanFilter.getPredictedVelocityThisAxis()
            val predictedVN = latitudeNorthKalmanFilter.getPredictedVelocityThisAxis()
            val resultantV = sqrt(predictedVE.pow(2) + predictedVN.pow(2))
            val deltaT = data.Timestamp - initialSensorData.Timestamp
            println("$deltaT seconds in, Lat: $predictedLat, Lon: $predictedLon, Alt: $predictedAlt, V(mph): ${2.23694 * resultantV}, A: ${data.AbsEastAcc * ACTUAL_GRAVITY}")
            outputs.add(
                OutputPacket(
                    sensorData = data,
                    PredictedLat = predictedLat,
                    PredictedLon = predictedLon,
                    PredictedAlt = predictedAlt,
                    ResultantMPH = 2.23694 * resultantV,
                    GPSLat = data.GpsLat,
                    GPSLon = data.GpsLon
                )
            )
        }
        println("got to end with no crash: $longitudeEastKalmanFilter")
        writeJsonSerializableToFile(outputs, "finalOut.json")
    }


}

typealias Degrees = Double

typealias Radians = Double
*/
