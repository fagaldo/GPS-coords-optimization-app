package polsl.gps.gpsoptimization
import android.location.Location


class MyLocation(provider: String?): Location(" ") {
    var accelerationX: Double? = 0.0
    var accelerationY: Double? = 0.0
    var accelerationZ: Double? = 0.0
    var azimuth: Float? = 0.0f
    init {
        this.provider = provider
        this.latitude = latitude
        this.longitude = longitude
    }
}