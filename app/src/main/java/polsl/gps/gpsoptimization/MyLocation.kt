package polsl.gps.gpsoptimization

import android.location.Location

/**
 * Klasa `MyLocation` rozszerza klasę `Location`, dodając dodatkowe właściwości, takie jak przyspieszenie i azymut.
 *
 * @param provider Nazwa dostawcy usług lokalizacyjnych.
 */
class MyLocation(provider: String?) : Location(provider) {

    /** Przyspieszenie w osi X. */
    var accelerationX: Double? = 0.0

    /** Przyspieszenie w osi Y. */
    var accelerationY: Double? = 0.0

    /** Przyspieszenie w osi Z. */
    var accelerationZ: Double? = 0.0

    /** Azymut (orientacja w stopniach). */
    var azimuth: Float? = 0.0f

    /**
     * Inicjalizuje obiekt `MyLocation` na podstawie dostarczonego providera.
     *
     * @param provider Nazwa dostawcy usług lokalizacyjnych.
     */
    init {
        this.provider = provider
        this.latitude = latitude
        this.longitude = longitude
    }
}
