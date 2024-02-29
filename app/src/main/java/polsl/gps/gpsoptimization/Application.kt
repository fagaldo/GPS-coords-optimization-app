package polsl.gps.gpsoptimization

import android.content.SharedPreferences
import android.location.Location
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import java.util.*


class Application : android.app.Application() {
    private lateinit var locations: ArrayList<Location>
    private lateinit var locationsStrings: ArrayList<String>
    override fun onCreate() {
        super.onCreate()
        instance = this
        locations = ArrayList()
        loadData()
    }
    fun getLocations():ArrayList<Location>{
        return this.locations
    }
    fun setLocations(locations: ArrayList<Location>)
    {
        this.locations = locations
    }
    private fun loadData()
    {
        val sharedPreferences: SharedPreferences = getSharedPreferences("waypoints", MODE_PRIVATE)
        val emptyList = Gson().toJson(ArrayList<String>())
        val json = sharedPreferences.getString("waypoints", emptyList)
        val type = object : TypeToken<ArrayList<String>>() {}.type
        locationsStrings = Gson().fromJson(json, type)
        for (loc in locationsStrings) {
            locations.add(parseStringToLoc(loc))
        }

    }

    private fun parseStringToLoc(input: String): Location{
        val parts = input.split(" ")

        var latitude: Double? = null
        var longitude: Double? = null
        var speed: Double? = null
        var accuracy: Double? = null
        var location: Location = Location("saved")
        for (part in parts) {
            if (part.startsWith("Latitude:")) {
                latitude = part.substringAfter("Latitude:").toDoubleOrNull()
            } else if (part.startsWith("Longtitude:")) { // Tutaj użyto "Longtitude" zamiast "Longitude" w danych wejściowych
                longitude = part.substringAfter("Longtitude:").toDoubleOrNull()
            } else if (part.startsWith("Speed:")) {
                speed = part.substringAfter("Speed:").toDoubleOrNull()
            } else if (part.startsWith("Accuracy:")) {
                accuracy = part.substringAfter("Accuracy:").toDoubleOrNull()
            }
        }
        if (latitude != null && longitude != null && speed != null && accuracy != null) {

            println("Latitude: $latitude")
            location.latitude = latitude
            println("Longitude: $longitude")
            location.longitude = longitude
            println("Speed: $speed")
            location.speed = speed.toFloat()
            println("Accuracy: $accuracy")
            location.accuracy = accuracy.toFloat()
            println(location)
        }
        return location
    }

    companion object {
        private var instance: Application? = null

        fun getInstance(): Application {
            if(instance == null){
                instance = Application()
            }
            return instance as Application
        }

    }
}
