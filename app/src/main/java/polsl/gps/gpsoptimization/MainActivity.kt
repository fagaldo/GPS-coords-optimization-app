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
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.View
import android.widget.*
import androidx.annotation.RequiresApi
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.google.android.gms.location.*
import java.util.*
import kotlin.collections.ArrayList


class MainActivity : AppCompatActivity() {
    private lateinit var tv_lat: TextView
    private lateinit var tv_azm: TextView
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
    private var z = 0.0
    private var currentRotationMatrix = FloatArray(9)
    private var orientationValues = FloatArray(3)
    private var accelerometerValues: FloatArray? = null
    private var magnetometerValues: FloatArray? = null
    private var gyroscopeValues: FloatArray? = null
    private var previousTimestamp: Long = 0
    private var alpha = 0.1f // Współczynnik filtru komplementarnego
    // Poprzednia wartość kąta azymutu
    private var previousAzimuth = 0f
    // Poprzednie wartości kątów orientacji
    private var previousOrientationValues = FloatArray(3)

    private var accelerometerSensor: Sensor? = null
    private var magnetometerSensor: Sensor? = null
    private var gyroscopeSensor: Sensor? = null
    private lateinit var sensorManager: SensorManager
    var updateOn = true
    private var filteredAzimuth = 0.0f
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
    private val handler = Handler(Looper.getMainLooper())
    private val delay: Long = 1000 // opóźnienie w milisekundach (5 sekund)

    private val runnable = object : Runnable {
        override fun run() {
            // Tutaj umieść kod, który ma być wykonany co określoną liczbę sekund
            // Na przykład:
            if(updateOn)
                updateGPS()

            // Zaplanuj ponowne wywołanie po upływie określonego czasu
            handler.postDelayed(this, delay)
        }
    }

    @RequiresApi(Build.VERSION_CODES.S)
    override fun onCreate(savedInstanceState: Bundle?) {
        savedLocations = ArrayList()
        magnetometerValues = FloatArray(10)
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        tv_lat = findViewById(R.id.tv_lat)
        tv_azm = findViewById(R.id.tv_azmVal)
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
        magnetometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        // Ensure you check if the sensors are null before using them
        if (accelerometerSensor == null) {
            // Handle the absence of an accelerometer
            do {
                accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
            }while(accelerometerSensor == null)
        }
        if (magnetometerSensor == null) {
            // Handle the absence of a magnetometer
            do {
                magnetometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
            }while(magnetometerSensor == null)
        }
        if (gyroscopeSensor == null) {
            // Handle the absence of a gyroscope
            do {
                gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
            } while (gyroscopeSensor == null)
        }

        handler.postDelayed(runnable, delay)
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
                locationResult.lastLocation?.let {
                    updateUIValues(it)
                }
            }
        }
        locationRequest = LocationRequest.Builder(DEFAULT_FAST_UPDATE_INTERVAL)
            .setMinUpdateDistanceMeters(1.0F)
            .setPriority(Priority.PRIORITY_BALANCED_POWER_ACCURACY) // Priorytet lokalizacji (wysoka dokładność)
            .setIntervalMillis(DEFAULT_FAST_UPDATE_INTERVAL)
            .build()

        sw_gps.setOnClickListener {
            if (sw_gps.isChecked()) {
                //most accurate - use GPS
                locationRequest =
                    LocationRequest.Builder(DEFAULT_FAST_UPDATE_INTERVAL).setPriority(Priority.PRIORITY_HIGH_ACCURACY)
                        .build()
                tv_sensor.setText("High accuracy")
            } else {
                locationRequest =
                    LocationRequest.Builder(DEFAULT_UPDATE_INTERVAL).setPriority(Priority.PRIORITY_LOW_POWER)
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
        sensorManager.registerListener(magnetometerListener, magnetometerSensor, SensorManager.SENSOR_STATUS_ACCURACY_HIGH, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(gyroscopeListener, gyroscopeSensor, SensorManager.SENSOR_STATUS_ACCURACY_HIGH, SensorManager.SENSOR_DELAY_FASTEST)
    }
    private val accelerometerListener = object : SensorEventListener{
        override fun onSensorChanged(event: SensorEvent) {
            if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
                x = event.values[0].toDouble()
                y = event.values[1].toDouble()
                z = event.values[2].toDouble()
                //updateGPS()
                accelerometerValues = event.values.clone()
            }
        }
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            // Metoda wywoływana, gdy dokładność sensora ulegnie zmianie
        }
    }
    private val magnetometerListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.sensor.type == Sensor.TYPE_MAGNETIC_FIELD) {
                magnetometerValues = event.values.clone()
            }
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            // Obsłuż zmianę dokładności sensora, jeśli to konieczne
        }
    }

    private val gyroscopeListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.sensor.type == Sensor.TYPE_GYROSCOPE) {
                // Aktualizuj odpowiednie pola lub wykonaj inne operacje z danymi z żyroskopu
                gyroscopeValues = event.values.clone()
                if (accelerometerValues != null && magnetometerValues != null && gyroscopeValues != null) {
                    SensorManager.getRotationMatrix(currentRotationMatrix, null, accelerometerValues, magnetometerValues)
                    SensorManager.getOrientation(currentRotationMatrix, orientationValues)

                    // Obliczamy wartość kąta na podstawie wartości zwróconej przez SensorManager.getOrientation
                    val azimuth = Math.toDegrees(orientationValues[0].toDouble()).toFloat()

                    // Zastosowanie filtru komplementarnego na podstawie danych z żyroskopu
                    // Nowa wartość kąta = alpha * nowa wartość kąta + (1 - alpha) * (poprzednia wartość kąta + zmiana z żyroskopu)
                    val gyroscopeDelta = gyroscopeValues!![2] * (event.timestamp - previousTimestamp) / 1000000000 // Delta obrotu wokół osi Z
                    filteredAzimuth = alpha * azimuth + (1 - alpha) * (previousAzimuth + gyroscopeDelta)
                    previousAzimuth = filteredAzimuth
                    Log.d("kąt", "taki: $filteredAzimuth")
                    // Aktualizujemy poprzednie wartości
                    previousOrientationValues = orientationValues.clone()
                    previousTimestamp = event.timestamp
                }
                else{Log.d("null", "null")}
            }
        }
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            // Obsłuż zmianę dokładności sensora, jeśli to konieczne
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
        tv_azm!!.text = "Location tracking disabled"
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
                        myCurrLocation.azimuth = filteredAzimuth
                        myCurrLocation.altitude = currLocation.altitude
                        myCurrLocation.time = currLocation.time
                        myCurrLocation.latitude = currLocation.latitude
                        myCurrLocation.longitude = currLocation.longitude
                        myCurrLocation.accelerationX = x
                        myCurrLocation.accelerationY = y
                        myCurrLocation.accelerationZ = z
                        savedLocations.add(myCurrLocation)
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
        tv_azm!!.text = myCurrLocation.azimuth.toString()
        if (location.hasAltitude()) {
            tv_altitude!!.text = location.altitude.toString()
        } else {
            tv_altitude!!.text = "Not available on this device"
        }
        if (myCurrLocation.accelerationX != null) {
            tv_speed!!.text = "x: " + myCurrLocation.accelerationX.toString() + "y: " + myCurrLocation.accelerationY.toString()
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
        const val DEFAULT_UPDATE_INTERVAL: Long = 30
        const val DEFAULT_FAST_UPDATE_INTERVAL: Long = 1
        const val PERMISSIONS_FINE_LOCATION = 99
    }
}