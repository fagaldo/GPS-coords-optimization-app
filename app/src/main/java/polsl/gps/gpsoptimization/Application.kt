package polsl.gps.gpsoptimization

import android.content.SharedPreferences
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken

/**
 * Główna klasa aplikacji, która zarządza zapisanymi lokalizacjami.
 * Obsługuje zapis i odczyt lokalizacji z pamięci SharedPreferences.
 */
class Application : android.app.Application() {
    /**
     * Lista zapisanych lokalizacji użytkownika.
     */
    private lateinit var locations: ArrayList<MyLocation>

    /**
     * Lista zapisanych lokalizacji jako ciągi znaków.
     */
    private lateinit var locationsStrings: ArrayList<String>

    override fun onCreate() {
        super.onCreate()
        instance = this
        locations = ArrayList()
        loadData()
    }

    /**
     * Zwraca listę zapisanych lokalizacji.
     * @return Lista obiektów MyLocation.
     */
    fun getLocations(): ArrayList<MyLocation> {
        return this.locations
    }

    /**
     * Ustawia listę lokalizacji.
     * @param locations Lista lokalizacji.
     */
    fun setLocations(locations: ArrayList<MyLocation>) {
        this.locations = locations
    }

    /**
     * Wczytuje zapisane dane lokalizacji z SharedPreferences.
     */
    private fun loadData() {
        val sharedPreferences: SharedPreferences = getSharedPreferences("waypoints", MODE_PRIVATE)
        val emptyList = Gson().toJson(ArrayList<String>())
        val json = sharedPreferences.getString("waypoints", emptyList)

        val type = object : TypeToken<ArrayList<String>>() {}.type
        locationsStrings = Gson().fromJson(json, type)

        for (loc in locationsStrings) {
            if (locationsStrings.isNotEmpty())
                locations.add(parseStringToLoc(loc))
        }
    }

    /**
     * Konwertuje zapisany ciąg tekstowy na obiekt MyLocation.
     * @param input Zapisany ciąg danych lokalizacji.
     * @return Obiekt MyLocation.
     */
    private fun parseStringToLoc(input: String): MyLocation {
        val parts = input.split(" ")
        var latitude: Double? = null
        var longitude: Double? = null
        var accuracy: Double? = null
        var altitude: Double? = null
        var x: Double? = null
        var y: Double? = null
        var z: Double? = null
        var time: Long? = null
        var azimuth: Float? = null
        val location = MyLocation("saved")

        for (part in parts) {
            when {
                part.startsWith("Latitude:") -> latitude = part.substringAfter("Latitude:").toDoubleOrNull()
                part.startsWith("Longitude:") -> longitude = part.substringAfter("Longitude:").toDoubleOrNull()
                part.startsWith("Altitude:") -> altitude = part.substringAfter("Altitude:").toDoubleOrNull()
                part.startsWith("Accuracy:") -> accuracy = part.substringAfter("Accuracy:").toDoubleOrNull()
                part.startsWith("SX:") -> x = part.substringAfter("X:").toDoubleOrNull()
                part.startsWith("SY:") -> y = part.substringAfter("Y:").toDoubleOrNull()
                part.startsWith("SZ:") -> z = part.substringAfter("Z:").toDoubleOrNull()
                part.startsWith("Time:") -> time = part.substringAfter("Time:").toLongOrNull()
                part.startsWith("Azimuth:") -> azimuth = part.substringAfter("Azimuth:").toFloatOrNull()
            }
        }

        if (latitude != null && longitude != null && accuracy != null && x != null && y != null &&
            time != null && z != null && altitude != null && azimuth != null) {

            location.latitude = latitude
            location.longitude = longitude
            location.accuracy = accuracy.toFloat()
            location.accelerationX = x
            location.accelerationY = y
            location.accelerationZ = z
            location.time = time
            location.altitude = altitude
            location.azimuth = azimuth
        }

        return location
    }

    companion object {
        private var instance: Application? = null

        /**
         * Zwraca instancję aplikacji.
         * @return Obiekt Application.
         */
        fun getInstance(): Application {
            if (instance == null) {
                instance = Application()
            }
            return instance as Application
        }
    }
}