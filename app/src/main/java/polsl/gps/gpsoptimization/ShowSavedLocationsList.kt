package polsl.gps.gpsoptimization

import android.content.Context
import android.content.SharedPreferences
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.media.MediaCodec.MetricsConstants.MODE

import android.os.Bundle
import android.os.Parcel
import android.os.Parcelable
import android.util.Log
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.ListView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.google.gson.*
import com.google.gson.annotations.JsonAdapter
import com.google.gson.reflect.TypeToken
import java.io.File
import java.io.FileOutputStream
import java.lang.reflect.Type
import java.util.*
import kotlin.math.pow


class ShowSavedLocationsList : AppCompatActivity() {
    lateinit var lv_savedLocations: ListView
    lateinit var saveBtn: Button
    lateinit var deleteBtn: Button
    private var printList: ArrayList<String> = ArrayList()
    private var savedLocations: ArrayList<MyLocation> = ArrayList()
    private lateinit var accelerometerSensor: Sensor
    private lateinit var sensorManager: SensorManager
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
            printList.add("Latitude:" + location.latitude.toString() + " Longitude:" +
                    location.longitude.toString() + " Accuracy:" + location.accuracy.toString() +
                    " SX:" + location.velocityX + " SY:" + location.velocityY + " Time:" + location.time
                    )

        lv_savedLocations.adapter =
            ArrayAdapter(this, android.R.layout.simple_list_item_1, printList)
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        }

    private fun saveData() {
        var sharedPreferences: SharedPreferences = getSharedPreferences("waypoints", MODE_PRIVATE)
        var sharedPreferencesEdit: SharedPreferences.Editor = sharedPreferences.edit()
        val gson = Gson()
        val type = object : TypeToken<ArrayList<String>>() {}.type
        var json = gson.toJson(printList, type)
        sharedPreferencesEdit.putString("waypoints", json)
        sharedPreferencesEdit.apply()
        val outputString = StringBuilder()
        for((j, i) in printList.withIndex())
        {
            outputString.append("Point $j:" + printList[j] + "\n")
        }
        val file = File(this.getExternalFilesDir(null), "original_location_info.txt")
        FileOutputStream(file).use {
            it.write(outputString.toString().toByteArray())
        }
        Toast.makeText(this, "Waypoints were saved", Toast.LENGTH_SHORT).show()
    }

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

}

