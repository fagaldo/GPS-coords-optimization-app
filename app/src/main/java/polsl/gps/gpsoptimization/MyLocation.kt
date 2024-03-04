package polsl.gps.gpsoptimization
import android.location.Location


class MyLocation(provider: String?): Location(" ") {
    var velocityX: Double? = 0.0
    var velocityY: Double? = 0.0

    init {
        this.provider = provider
        this.latitude = latitude
        this.longitude = longitude
    }
}