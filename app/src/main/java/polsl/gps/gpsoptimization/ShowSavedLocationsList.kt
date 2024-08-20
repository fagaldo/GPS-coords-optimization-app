package polsl.gps.gpsoptimization

import android.content.Context
import android.content.SharedPreferences
import android.hardware.Sensor
import android.hardware.SensorManager
import android.os.Bundle
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.ListView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import java.io.File
import java.io.FileOutputStream
import java.util.*

/**
 * Klasa `ShowSavedLocationsList` odpowiada za wyświetlanie listy zapisanych lokalizacji,
 * a także obsługę ich zapisu oraz usuwania.
 */
class ShowSavedLocationsList : AppCompatActivity() {
    /**
     * ListView do wyświetlania zapisanych lokalizacji.
     */
    lateinit var lv_savedLocations: ListView

    /**
     * Przycisk do zapisywania lokalizacji.
     */
    lateinit var saveBtn: Button

    /**
     * Przycisk do usuwania lokalizacji.
     */
    lateinit var deleteBtn: Button

    /**
     * Lista lokalizacji w formacie tekstowym do wyświetlenia.
     */
    private var printList: ArrayList<String> = ArrayList()

    /**
     * Lista zapisanych obiektów MyLocation.
     */
    private var savedLocations: ArrayList<MyLocation> = ArrayList()

    /**
     * Sensor akcelerometru.
     */
    private lateinit var accelerometerSensor: Sensor

    /**
     * Menedżer sensorów.
     */
    private lateinit var sensorManager: SensorManager


    /**
     * Funkcja `onCreate` wywoływana przy tworzeniu aktywności.
     * Inicjalizuje interfejs użytkownika oraz sensory.
     *
     * @param savedInstanceState Stan zapisany aktywności.
     */
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_show_saved_locations_list)

        lv_savedLocations = findViewById(R.id.lv_wayPoints)
        saveBtn = findViewById(R.id.saveBtn)
        deleteBtn = findViewById(R.id.delBtn)

        // Obsługa przycisków zapisu i usuwania danych
        saveBtn.setOnClickListener {
            saveData()
        }
        deleteBtn.setOnClickListener {
            deleteData()
        }

        val application = applicationContext as Application
        savedLocations = application.getLocations()

        // Dodanie lokalizacji do listy wyświetlania
        for (location in savedLocations)
            printList.add(
                "Latitude:" + location.latitude.toString() + " Longitude:" +
                        location.longitude.toString() + " Altitude:" + location.altitude.toString() +
                        " Accuracy:" + location.accuracy.toString() + " SX:" + location.accelerationX +
                        " SY:" + location.accelerationY + " SZ:" + location.accelerationZ +
                        " Time:" + location.time + " Azimuth:" + location.azimuth.toString()
            )

        lv_savedLocations.adapter =
            ArrayAdapter(this, android.R.layout.simple_list_item_1, printList)

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    }

    /**
     * Zapisuje listę lokalizacji do pliku oraz `SharedPreferences`.
     */
    private fun saveData() {
        val sharedPreferences: SharedPreferences = getSharedPreferences("waypoints", MODE_PRIVATE)
        val sharedPreferencesEdit: SharedPreferences.Editor = sharedPreferences.edit()
        val gson = Gson()
        val type = object : TypeToken<ArrayList<String>>() {}.type
        val json = gson.toJson(printList, type)
        sharedPreferencesEdit.putString("waypoints", json)
        sharedPreferencesEdit.apply()

        // Zapis lokalizacji do pliku
        val outputString = StringBuilder()
        for ((j, i) in printList.withIndex()) {
            outputString.append("Point $j: $i\n")
        }
        val file = File(this.getExternalFilesDir(null), "original_location_info.txt")
        FileOutputStream(file).use {
            it.write(outputString.toString().toByteArray())
        }

        Toast.makeText(this, "Waypoints were saved", Toast.LENGTH_SHORT).show()
    }

    /**
     * Usuwa zapisane lokalizacje z `SharedPreferences` oraz czyści listę lokalizacji.
     */
    private fun deleteData() {
        val sharedPreferences: SharedPreferences = getSharedPreferences("waypoints", MODE_PRIVATE)
        val sharedPreferencesEdit: SharedPreferences.Editor = sharedPreferences.edit()
        sharedPreferencesEdit.remove("waypoints")
        sharedPreferencesEdit.apply()

        savedLocations.clear()
        printList.clear()
        lv_savedLocations.adapter =
            ArrayAdapter(this, android.R.layout.simple_list_item_1, printList)

        val application = applicationContext as Application
        application.setLocations(savedLocations)

        Toast.makeText(this, "Waypoints were deleted", Toast.LENGTH_SHORT).show()
    }
}