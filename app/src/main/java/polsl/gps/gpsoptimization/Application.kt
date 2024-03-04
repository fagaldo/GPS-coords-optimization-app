package polsl.gps.gpsoptimization

import android.content.SharedPreferences
import android.util.Log
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import kotlin.collections.ArrayList


class Application : android.app.Application() {
    private lateinit var locations: ArrayList<MyLocation>
    private lateinit var locationsStrings: ArrayList<String>
    override fun onCreate() {
        super.onCreate()
        instance = this
        locations = ArrayList()
        loadData()
    }
    fun getLocations():ArrayList<MyLocation>{
        return this.locations
    }
    fun setLocations(locations: ArrayList<MyLocation>)
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

    private fun parseStringToLoc(input: String): MyLocation{
        val parts = input.split(" ")
        var latitude: Double? = null
        var longitude: Double? = null
        var accuracy: Double? = null
        var x: Double? = null
        var y: Double? = null
        var time: Long? = null
        var location: MyLocation = MyLocation("saved")
        for (part in parts) {
            if (part.startsWith("Latitude:")) {
                latitude = part.substringAfter("Latitude:").toDoubleOrNull()
            } else if (part.startsWith("Longitude:")) { // Tutaj użyto "Longtitude" zamiast "Longitude" w danych wejściowych
                longitude = part.substringAfter("Longitude:").toDoubleOrNull()
            } else if (part.startsWith("Accuracy:")) {
                accuracy = part.substringAfter("Accuracy:").toDoubleOrNull()
            }
            else if (part.startsWith("SX:")) {
                x = part.substringAfter("X:").toDoubleOrNull()
            }
            else if (part.startsWith("SY:")) {
                y = part.substringAfter("Y:").toDoubleOrNull()
            }
            else if (part.startsWith("Time:")) {
                time = part.substringAfter("Time:").toLongOrNull()
            }
        }
        if (latitude != null && longitude != null && accuracy != null && x != null && y != null && time != null) {

            println("Latitude: $latitude")
            location.latitude = latitude
            println("Longitude: $longitude")
            location.longitude = longitude
            println("Accuracy: $accuracy")
            location.accuracy = accuracy.toFloat()
            Log.d("speed x", "$x")
            location.velocityX = x
            Log.d("speed y", "$y")
            location.velocityY = y
            location.time = time
            println(location)

        }
        //Log.d("Zwracam lokacje", location.toString())
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
