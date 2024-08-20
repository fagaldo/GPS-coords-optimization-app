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
import android.os.*
import android.widget.*
import androidx.annotation.RequiresApi
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.google.android.gms.location.*

/**
 * Klasa `MainActivity` odpowiada za główną aktywność aplikacji GPS.
 * Zarządza lokalizacją, sensorami oraz interfejsem użytkownika.
 */
class MainActivity : AppCompatActivity() {

    // Komponenty UI
    /**
     * Tekst wyświetlający szerokość geograficzną.
     */
    private lateinit var tv_lat: TextView

    /**
     * Tekst wyświetlający azymut.
     */
    private lateinit var tv_azm: TextView

    /**
     * Tekst wyświetlający długość geograficzną.
     */
    private lateinit var tv_lon: TextView

    /**
     * Tekst wyświetlający wysokość n.p.m.
     */
    private lateinit var tv_altitude: TextView

    /**
     * Tekst wyświetlający dokładność lokalizacji.
     */
    private lateinit var tv_accuracy: TextView

    /**
     * Tekst wyświetlający prędkość.
     */
    private lateinit var tv_speed: TextView

    /**
     * Tekst wyświetlający status sensora.
     */
    private lateinit var tv_sensor: TextView

    /**
     * Tekst wyświetlający informacje o aktualizacji lokalizacji.
     */
    private lateinit var tv_updates: TextView

    /**
     * Tekst wyświetlający adres lokalizacji.
     */
    private lateinit var tv_address: TextView

    /**
     * Tekst wyświetlający liczbę zapisanych punktów trasy.
     */
    private lateinit var tv_wayptsCount: TextView

    /**
     * Przycisk dodający nowy punkt trasy.
     */
    private lateinit var btn_newWayPoint: Button

    /**
     * Przycisk wyświetlający zapisane punkty trasy.
     */
    private lateinit var btn_showWaypoints: Button

    /**
     * Przycisk wyświetlający mapę.
     */
    private lateinit var btn_showMap: Button

    /**
     * Przełącznik aktualizacji lokalizacji.
     */
    private lateinit var sw_locationsupdates: Switch

    /**
     * Przełącznik GPS.
     */
    private lateinit var sw_gps: Switch

    /**
     * Spinner do wyboru algorytmu wygładzania.
     */
    private lateinit var spinner: Spinner

// Zmienne dla sensorów
    /**
     * Wartość przyspieszenia w osi X.
     */
    private var x = 0.0

    /**
     * Wartość przyspieszenia w osi Y.
     */
    private var y = 0.0

    /**
     * Wartość przyspieszenia w osi Z.
     */
    private var z = 0.0

    /**
     * Macierz rotacji.
     */
    private var currentRotationMatrix = FloatArray(9)

    /**
     * Wartości orientacji.
     */
    private var orientationValues = FloatArray(3)

    /**
     * Wartości z akcelerometru.
     */
    private var accelerometerValues: FloatArray? = null

    /**
     * Wartości z magnetometru.
     */
    private var magnetometerValues: FloatArray? = null

    /**
     * Wartości z żyroskopu.
     */
    private var gyroscopeValues: FloatArray? = null

    /**
     * Znacznik czasu poprzedniego odczytu.
     */
    private var previousTimestamp: Long = 0

    /**
     * Współczynnik filtru komplementarnego.
     */
    private var alpha = 0.1f

    /**
     * Poprzedni azymut.
     */
    private var previousAzimuth = 0f

    /**
     * Poprzednie wartości orientacji.
     */
    private var previousOrientationValues = FloatArray(3)

    private var accelerometerSensor: Sensor? = null
    private var magnetometerSensor: Sensor? = null
    private var gyroscopeSensor: Sensor? = null
    private lateinit var sensorManager: SensorManager

// Zmienne dla lokalizacji
    /**
     * Flaga wskazująca, czy aktualizacje lokalizacji są włączone.
     */
    var updateOn = true

    /**
     * Przefiltrowany azymut.
     */
    private var filteredAzimuth = 0.0f

    /**
     * Bieżąca lokalizacja.
     */
    private lateinit var currLocation: Location

    /**
     * Obiekt bieżącej lokalizacji z dodatkowymi danymi.
     */
    private lateinit var myCurrLocation: MyLocation

    /**
     * Lista zapisanych lokalizacji.
     */
    private lateinit var savedLocations: MutableList<MyLocation>

    private var fusedLocationProviderClient: FusedLocationProviderClient? = null
    private lateinit var locationCallBack: LocationCallback
    private lateinit var locationRequest: LocationRequest
    private val handler = Handler(Looper.getMainLooper())

    /**
     * Opóźnienie w milisekundach (2 sekundy).
     */
    private val delay: Long = 2000

    /**
     * Runnable do cyklicznego odświeżania GPS.
     */
    private val runnable = object : Runnable {
        override fun run() {
            if (updateOn) updateGPS()
            handler.postDelayed(this, delay)
        }
    }


    /**
     * Funkcja `onCreate` wywoływana przy tworzeniu aktywności. Inicjuje komponenty UI oraz ustawia początkowe wartości.
     *
     * @param savedInstanceState Stan zapisany aktywności.
     */
    @RequiresApi(Build.VERSION_CODES.S)
    override fun onCreate(savedInstanceState: Bundle?) {
        savedLocations = ArrayList()
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Inicjalizacja komponentów UI
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

        // Inicjalizacja sensorów
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        magnetometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        // Inicjalizacja aktualizacji GPS
        handler.postDelayed(runnable, delay)
        val adapter: ArrayAdapter<CharSequence> = ArrayAdapter.createFromResource(
            this,
            R.array.smoothing_algorithms,
            android.R.layout.simple_spinner_item
        )
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        spinner.adapter = adapter

        // Inicjalizacja callbacka dla lokalizacji
        locationCallBack = object : LocationCallback() {
            override fun onLocationResult(locationResult: LocationResult) {
                super.onLocationResult(locationResult)
                locationResult.lastLocation?.let {
                    updateUIValues(it)
                }
            }
        }

        locationRequest = LocationRequest.Builder(DEFAULT_FAST_UPDATE_INTERVAL)
            .setMinUpdateDistanceMeters(1.0F)
            .setPriority(Priority.PRIORITY_BALANCED_POWER_ACCURACY)
            .setIntervalMillis(DEFAULT_FAST_UPDATE_INTERVAL)
            .build()

        // Obsługa przełączników
        sw_gps.setOnClickListener {
            if (sw_gps.isChecked) {
                locationRequest = LocationRequest.Builder(DEFAULT_FAST_UPDATE_INTERVAL)
                    .setPriority(Priority.PRIORITY_HIGH_ACCURACY)
                    .build()
                tv_sensor.text = "High accuracy"
            } else {
                locationRequest = LocationRequest.Builder(DEFAULT_UPDATE_INTERVAL)
                    .setPriority(Priority.PRIORITY_LOW_POWER)
                    .build()
                tv_sensor.text = "Low Power"
            }
            startLocationUpdates()
        }

        sw_locationsupdates.setOnClickListener {
            if (sw_locationsupdates.isChecked) {
                updateOn = true
                startLocationUpdates()
            } else {
                stopLocationUpdates()
                updateOn = false
            }
        }

        // Obsługa przycisków
        btn_newWayPoint.setOnClickListener {
            val application = applicationContext as polsl.gps.gpsoptimization.Application
            savedLocations.add(myCurrLocation)
            application.setLocations(savedLocations as ArrayList<MyLocation>)
            updateGPS()
        }

        btn_showWaypoints.setOnClickListener {
            val i = Intent(this@MainActivity, ShowSavedLocationsList::class.java)
            startActivity(i)
        }

        btn_showMap.setOnClickListener {
            val selectedAlgorithm = spinner.selectedItem.toString()
            val i = Intent(this@MainActivity, MapsActivity::class.java)
            i.putExtra("selectedAlgorithm", selectedAlgorithm)
            startActivity(i)
        }

        startLocationUpdates()
    }

    /**
     * Funkcja `onResume` rejestruje listenerów sensorów po wznowieniu aktywności.
     */
    override fun onResume() {
        super.onResume()
        sensorManager.registerListener(accelerometerListener, accelerometerSensor, SensorManager.SENSOR_STATUS_ACCURACY_HIGH, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(magnetometerListener, magnetometerSensor, SensorManager.SENSOR_STATUS_ACCURACY_HIGH, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(gyroscopeListener, gyroscopeSensor, SensorManager.SENSOR_STATUS_ACCURACY_HIGH, SensorManager.SENSOR_DELAY_FASTEST)
    }

    /**
     * Listener dla akcelerometru.
     */
    private val accelerometerListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
                x = event.values[0].toDouble()
                y = event.values[1].toDouble()
                z = event.values[2].toDouble()
                accelerometerValues = event.values.clone()
            }
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    }

    /**
     * Listener dla magnetometru.
     */
    private val magnetometerListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.sensor.type == Sensor.TYPE_MAGNETIC_FIELD) {
                magnetometerValues = event.values.clone()
            }
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    }

    /**
     * Listener dla żyroskopu.
     */
    private val gyroscopeListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.sensor.type == Sensor.TYPE_GYROSCOPE) {
                gyroscopeValues = event.values.clone()
                if (accelerometerValues != null && magnetometerValues != null && gyroscopeValues != null && updateOn) {
                    SensorManager.getRotationMatrix(currentRotationMatrix, null, accelerometerValues, magnetometerValues)
                    SensorManager.getOrientation(currentRotationMatrix, orientationValues)
                    val azimuth = Math.toDegrees(orientationValues[0].toDouble()).toFloat()
                    val gyroscopeDelta = gyroscopeValues!![2] * (event.timestamp - previousTimestamp) / 1000000000
                    filteredAzimuth = alpha * azimuth + (1 - alpha) * (previousAzimuth + gyroscopeDelta)
                    previousAzimuth = filteredAzimuth
                    previousOrientationValues = orientationValues.clone()
                    previousTimestamp = event.timestamp
                }
            }
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    }

    /**
     * Rozpoczyna aktualizacje lokalizacji.
     */
    private fun startLocationUpdates() {
        try {
            sensorManager.registerListener(accelerometerListener, accelerometerSensor, SensorManager.SENSOR_STATUS_ACCURACY_HIGH, SensorManager.SENSOR_DELAY_FASTEST)
            fusedLocationProviderClient?.requestLocationUpdates(locationRequest, locationCallBack, null)
        } catch (e: SecurityException) {
            tv_updates.text = "Somehow we lack the permission"
        }
        Toast.makeText(this, "Tracking location", Toast.LENGTH_SHORT).show()
        updateGPS()
    }

    /**
     * Zatrzymuje aktualizacje lokalizacji.
     */
    private fun stopLocationUpdates() {
        tv_updates.text = "Location is NOT being tracked"
        tv_lat.text = "Location tracking disabled"
        tv_azm.text = "Location tracking disabled"
        tv_lon.text = "Location tracking disabled"
        tv_speed.text = "Location tracking disabled"
        tv_address.text = "Location tracking disabled"
        tv_accuracy.text = "Location tracking disabled"
        tv_altitude.text = "Location tracking disabled"
        fusedLocationProviderClient?.removeLocationUpdates(locationCallBack)
    }

    /**
     * Obsługuje wynik zapytania o uprawnienia.
     */
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

    /**
     * Aktualizuje informacje o lokalizacji GPS.
     */
    private fun updateGPS() {
        fusedLocationProviderClient = LocationServices.getFusedLocationProviderClient(this@MainActivity)
        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) == PackageManager.PERMISSION_GRANTED
        ) {
            fusedLocationProviderClient!!.lastLocation
                .addOnSuccessListener {
                    fun onSuccess(location: Location) {
                        tv_updates.text = "Location is being tracked"
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
                        updateUIValues(location)
                    }
                    onSuccess(it)
                }
        } else {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                ActivityCompat.requestPermissions(
                    this,
                    arrayOf(Manifest.permission.ACCESS_FINE_LOCATION),
                    PERMISSIONS_FINE_LOCATION
                )
            }
        }
    }

    /**
     * Aktualizuje wartości UI na podstawie bieżącej lokalizacji.
     *
     * @param location Bieżąca lokalizacja użytkownika.
     */
    private fun updateUIValues(location: Location) {
        tv_lat.text = location.latitude.toString()
        tv_lon.text = location.longitude.toString()
        tv_accuracy.text = location.accuracy.toString()
        tv_azm.text = myCurrLocation.azimuth.toString()

        if (location.hasAltitude()) {
            tv_altitude.text = location.altitude.toString()
        } else {
            tv_altitude.text = "Not available on this device"
        }

        if (myCurrLocation.accelerationX != null) {
            tv_speed.text = "x: ${myCurrLocation.accelerationX} y: ${myCurrLocation.accelerationY}"
        } else {
            tv_speed.text = "Not available on this device"
        }

        val geocoder = Geocoder(this@MainActivity)
        try {
            val addresses = geocoder.getFromLocation(location.latitude, location.longitude, 1)
            tv_address.text = addresses?.get(0)?.getAddressLine(0)
        } catch (e: Exception) {
            tv_address.text = "Something went wrong with retrieving the address"
        }

        val application = applicationContext as polsl.gps.gpsoptimization.Application
        savedLocations = application.getLocations()
        tv_wayptsCount.text = savedLocations.size.toString()
    }

    companion object {
        const val DEFAULT_UPDATE_INTERVAL: Long = 30
        const val DEFAULT_FAST_UPDATE_INTERVAL: Long = 1
        const val PERMISSIONS_FINE_LOCATION = 99
    }
}