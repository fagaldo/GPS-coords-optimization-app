package polsl.gps.gpsoptimization

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Geocoder
import android.location.Location
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.*
import androidx.annotation.RequiresApi
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.google.android.gms.location.*
import java.util.*


class MainActivity : AppCompatActivity() {
    private lateinit var tv_lat: TextView
    private lateinit var tv_lon: TextView
    private lateinit var tv_altitude: TextView
    private lateinit var tv_accuracy: TextView
    private lateinit var tv_speed: TextView
    private lateinit var tv_sensor: TextView
    private lateinit var tv_updates: TextView
    private lateinit var tv_address: TextView
    private lateinit var tv_wayptsCount: TextView
    private lateinit var btn_newWayPoint: Button
    private lateinit var btn_showWaypoints: Button
    private lateinit var btn_showMap: Button
    private lateinit var sw_locationsupdates: Switch
    private lateinit var sw_gps: Switch
    private lateinit var spinner: Spinner
    private var x= 0.0
    private var y = 0.0
    private lateinit var accelerometerSensor: Sensor
    private lateinit var sensorManager: SensorManager
    var updateOn = true

    //current location
    private lateinit var currLocation: Location
    private lateinit var myCurrLocation: MyLocation
    //list of savec locations
    private lateinit var savedLocations: MutableList<MyLocation>

    //Google's API for location services. Hearth and sould of the app
    private var fusedLocationProviderClient: FusedLocationProviderClient? = null
    private lateinit var locationCallBack: LocationCallback
    private lateinit var locationRequest: LocationRequest
    //Location request is a config file for all settings related to FusedLocationProviderClient


    @RequiresApi(Build.VERSION_CODES.S)
    override fun onCreate(savedInstanceState: Bundle?) {
        savedLocations = ArrayList()
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        tv_lat = findViewById(R.id.tv_lat)
        tv_lon = findViewById(R.id.tv_lon)
        tv_altitude = findViewById(R.id.tv_altitude)
        tv_accuracy = findViewById(R.id.tv_accuracy)
        tv_speed = findViewById(R.id.tv_speed)
        tv_sensor = findViewById(R.id.tv_sensor)
        tv_updates = findViewById(R.id.tv_updates)
        tv_address = findViewById(R.id.tv_address)
        sw_locationsupdates = findViewById(R.id.sw_locationsupdates)
        sw_gps = findViewById(R.id.sw_gps)
        btn_newWayPoint = findViewById(R.id.btn_newWayPoint)
        btn_showWaypoints = findViewById(R.id.btn_showWaypoints)
        tv_wayptsCount = findViewById(R.id.tv_wayPointsCount)
        btn_showMap = findViewById(R.id.btn_showMap)
        spinner = findViewById(R.id.spinner_smoothing_algorithm)
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)

        val adapter: ArrayAdapter<CharSequence> = ArrayAdapter.createFromResource(
            this,
            R.array.smoothing_algorithms,
            android.R.layout.simple_spinner_item
        )
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        spinner.adapter = adapter
        //set all properties of LocatonRequest
        //locationRequest.setInterval(1000 * DEFAULT_UPDATE_INTERVAL);
        //locationRequest.setFastestInterval(100 * DEFAULT_FAST_UPDATE_INTERVAL)
       // locationRequest.setPriority(LocationRequest.PRIORITY_BALANCED_POWER_ACCURACY)

        locationCallBack = object : LocationCallback() {
            override fun onLocationResult(locationResult: LocationResult) {
                super.onLocationResult(locationResult)
                //save the location
                locationResult.lastLocation?.let { updateUIValues(it) }
            }
        }
        locationRequest = LocationRequest.Builder(10)
            .setMinUpdateDistanceMeters(0F) // Najkrótszy dopuszczalny interwał aktualizacji lokalizacji (np. co 5 sekund)
            .setPriority(Priority.PRIORITY_BALANCED_POWER_ACCURACY) // Priorytet lokalizacji (wysoka dokładność)
            .build()

        sw_gps.setOnClickListener {
            if (sw_gps.isChecked()) {
                //most accurate - use GPS
                locationRequest =
                    LocationRequest.Builder(10).setPriority(Priority.PRIORITY_HIGH_ACCURACY)
                        .build()
                tv_sensor.setText("High accuracy")
            } else {
                locationRequest =
                    LocationRequest.Builder(100).setPriority(Priority.PRIORITY_LOW_POWER)
                        .build()
                tv_sensor.setText("Low Power")
            }
            startLocationUpdates()
        }
        sw_locationsupdates.setOnClickListener(View.OnClickListener {
            if (sw_locationsupdates.isChecked()) {
                updateOn = true
                //turn on location tracking
                startLocationUpdates()
            } else {
                //turn off location tracking
                stopLocationUpdates()
                updateOn = false
            }
        })
        btn_newWayPoint.setOnClickListener(View.OnClickListener { //add the new location to the global list

            val application = applicationContext as polsl.gps.gpsoptimization.Application
            //savedLocations = application.getLocations()

            savedLocations.add(myCurrLocation)
            Log.d("Accuracy dodanego ", myCurrLocation.accuracy.toString())
            application.setLocations(savedLocations as ArrayList<MyLocation>)
            updateGPS()
        })
        btn_showWaypoints.setOnClickListener(View.OnClickListener {
            val i = Intent(this@MainActivity, ShowSavedLocationsList::class.java)
            startActivity(i)
        })
        btn_showMap.setOnClickListener(View.OnClickListener {
            val selectedAlgorithm = spinner.selectedItem.toString()
            val i = Intent(this@MainActivity, MapsActivity::class.java)
            i.putExtra("selectedAlgorithm", selectedAlgorithm)
            startActivity(i)
        })

        startLocationUpdates()
    }
    override fun onResume() {
        super.onResume()
        sensorManager.registerListener(accelerometerListener, accelerometerSensor, SensorManager.SENSOR_STATUS_ACCURACY_HIGH, SensorManager.SENSOR_DELAY_FASTEST)
    }
    private val accelerometerListener = object : SensorEventListener{
        override fun onSensorChanged(event: SensorEvent) {
            if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
                x = event.values[0].toDouble()
                y = event.values[1].toDouble()
                //Log.d("Przyspieszenia","Przyspieszenie x: $x przypiszenie y: $y, przyspieszenie z: $z")
            }
        }
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            // Metoda wywoływana, gdy dokładność sensora ulegnie zmianie
        }
    }

    private fun startLocationUpdates() {
        try {
            sensorManager.registerListener(accelerometerListener, accelerometerSensor, SensorManager.SENSOR_STATUS_ACCURACY_HIGH, SensorManager.SENSOR_DELAY_FASTEST)
            fusedLocationProviderClient?.requestLocationUpdates(locationRequest, locationCallBack, null)

        } catch (e: SecurityException) {
            tv_updates!!.text = "Somehow we lack the permission"
        }
        Toast.makeText(this, "Tracking location", Toast.LENGTH_SHORT
        ).show()
        updateGPS()
    }

    private fun stopLocationUpdates() {
        tv_updates!!.text = "Location in NOT being tracked"
        tv_lat!!.text = "Location tracking disabled"
        tv_lon!!.text = "Location tracking disabled"
        tv_speed!!.text = "Location tracking disabled"
        tv_address!!.text = "Location tracking disabled"
        tv_accuracy!!.text = "Location tracking disabled"
        tv_altitude!!.text = "Location tracking disabled"
        fusedLocationProviderClient?.removeLocationUpdates(locationCallBack)
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        when (requestCode) {
            PERMISSIONS_FINE_LOCATION -> if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                updateGPS()
            } else {
                Toast.makeText(
                    this,
                    "This app requires permission to be granted in order to work",
                    Toast.LENGTH_SHORT
                ).show()
                finish()
            }
        }
    }

    private fun updateGPS() {
        //get permissions from the user to track GPS
        //get the current location from the fuesd client
        //Update the UI = i.e. set all properties in their associated text view items
        fusedLocationProviderClient =
            LocationServices.getFusedLocationProviderClient(this@MainActivity)
        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) == PackageManager.PERMISSION_GRANTED
        ) {
            //user provided the permission
            fusedLocationProviderClient!!.lastLocation
                .addOnSuccessListener {
                    fun onSuccess(location: Location) {
                        tv_updates!!.text = "Location is being tracked"
                        currLocation = location
                        myCurrLocation = MyLocation("")
                        myCurrLocation.accuracy = currLocation.accuracy
                        myCurrLocation.altitude = currLocation.altitude
                        myCurrLocation.time = currLocation.time
                        myCurrLocation.latitude = currLocation.latitude
                        myCurrLocation.longitude = currLocation.longitude
                        myCurrLocation.velocityX = x
                        myCurrLocation.velocityY = y
                        //we got permission. Put the values of location. XXX into the UI components.
                        updateUIValues(location)
                    }
                    onSuccess(it)
                }
        } else {
            //permission denied
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                requestPermissions(
                    arrayOf(Manifest.permission.ACCESS_FINE_LOCATION),
                    PERMISSIONS_FINE_LOCATION
                )
            }
        }
    }

    //update all of the text view with new location
    private fun updateUIValues(location: Location) {

        tv_lat!!.text = location.latitude.toString()
        tv_lon!!.text = location.longitude.toString()
        tv_accuracy!!.text = location.accuracy.toString()
        if (location.hasAltitude()) {
            tv_altitude!!.text = location.altitude.toString()
        } else {
            tv_altitude!!.text = "Not available on this device"
        }
        if (myCurrLocation.velocityX != null) {
            tv_speed!!.text = "x: " + myCurrLocation.velocityX.toString() + "y: " + myCurrLocation.velocityY.toString()
        } else {
            tv_speed!!.text = "Not available on this device"
        }
        val geocoder = Geocoder(this@MainActivity)
        try {
            val addresses = geocoder.getFromLocation(location.latitude, location.longitude, 1)
            tv_address!!.text = addresses?.get(0)?.getAddressLine(0)
        } catch (e: Exception) {
            tv_address!!.text = "Something went wrong with retrieving the address"
        }
        val application = applicationContext as polsl.gps.gpsoptimization.Application
        savedLocations = application.getLocations()
        //show the number of waypoints saved
        tv_wayptsCount!!.text = Integer.toString(savedLocations!!.size)
    }

    companion object {
        const val DEFAULT_UPDATE_INTERVAL = 30
        const val DEFAULT_FAST_UPDATE_INTERVAL = 5
        const val PERMISSIONS_FINE_LOCATION = 99
        const val MAPS_API_KEY = "AIzaSyDjz9DNYG-G6w2teu95VxuyfkjrKUdrZCw"
    }
}