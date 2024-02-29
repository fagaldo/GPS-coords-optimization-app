package polsl.gps.gpsoptimization

import android.content.SharedPreferences
import android.location.Location
import android.media.MediaCodec.MetricsConstants.MODE

import android.os.Bundle
import android.os.Parcel
import android.os.Parcelable
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.ListView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.google.gson.*
import com.google.gson.annotations.JsonAdapter
import com.google.gson.reflect.TypeToken
import java.lang.reflect.Type
import java.util.*


class ShowSavedLocationsList : AppCompatActivity() {
    lateinit var lv_savedLocations: ListView
    lateinit var saveBtn: Button
    lateinit var deleteBtn: Button
    private var printList: ArrayList<String> = ArrayList()
    private var savedLocations: ArrayList<Location> = ArrayList()
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_show_saved_locations_list)
        lv_savedLocations = findViewById(R.id.lv_wayPoints)
        saveBtn = findViewById(R.id.saveBtn)
        deleteBtn = findViewById(R.id.delBtn)
        saveBtn.setOnClickListener{
            saveData()
        }
        deleteBtn.setOnClickListener{
            deleteData()
        }
        val application = applicationContext as Application
        savedLocations = application.getLocations()
        for(location in savedLocations)
            printList.add("Latitude:" + location.latitude.toString() + " Longtitude:" +
                    location.longitude.toString() + " Speed:" + location.speed.toString()
                    + " Accuracy:" + location.accuracy.toString())
        lv_savedLocations.adapter =
            ArrayAdapter(this, android.R.layout.simple_list_item_1, printList)
        }

    private fun saveData() {
        var sharedPreferences: SharedPreferences = getSharedPreferences("waypoints", MODE_PRIVATE)
        var sharedPreferencesEdit: SharedPreferences.Editor = sharedPreferences.edit()
        val gson = Gson()
        val type = object : TypeToken<ArrayList<String>>() {}.type
        var json = gson.toJson(printList, type)
        sharedPreferencesEdit.putString("waypoints", json)
        sharedPreferencesEdit.apply()
        Toast.makeText(this, "Waypoints were saved", Toast.LENGTH_SHORT).show()
    }

    /*private fun loadData(){
        val sharedPreferences: SharedPreferences = getSharedPreferences("waypoints", MODE_PRIVATE)
        val json = sharedPreferences.getString("waypoints", "{}")
        val type = object : TypeToken<ArrayList<String>>() {}.type
        printList = Gson()?.fromJson(json, type)
        if(printList == null)
            printList = ArrayList()
    }*/

    private fun deleteData(){
        var sharedPreferences: SharedPreferences = getSharedPreferences("waypoints", MODE_PRIVATE)
        var sharedPreferencesEdit: SharedPreferences.Editor = sharedPreferences.edit()
        sharedPreferencesEdit.remove("waypoints")
        sharedPreferencesEdit.apply()
        savedLocations.clear()
        printList.clear()
        lv_savedLocations.adapter =
            ArrayAdapter(this, android.R.layout.simple_list_item_1, printList)
        val application = applicationContext as polsl.gps.gpsoptimization.Application
        application.setLocations(savedLocations)
        Toast.makeText(this, "Waypoints were deleted", Toast.LENGTH_SHORT).show()
    }

    private class TimeSerialize : JsonSerializer<Location>{
        override fun serialize(
            src: Location?,
            typeOfSrc: Type?,
            context: JsonSerializationContext?
        ): JsonElement {
            return JsonPrimitive(src.toString())
        }
    }



}

