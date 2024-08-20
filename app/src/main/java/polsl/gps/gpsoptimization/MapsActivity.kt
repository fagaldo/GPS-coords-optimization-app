package polsl.gps.gpsoptimization

import android.graphics.Color
import android.os.Build
import android.os.Bundle
import androidx.annotation.RequiresApi
import androidx.fragment.app.FragmentActivity
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.OnMapReadyCallback
import com.google.android.gms.maps.SupportMapFragment
import com.google.android.gms.maps.model.*
import polsl.gps.gpsoptimization.databinding.ActivityMapsBinding
import java.io.File
import java.io.FileOutputStream
import kotlin.math.*
import kotlin.text.Typography.times

/**
 * Klasa `MapsActivity` odpowiada za wyświetlanie mapy oraz rysowanie markerów i linii
 * na podstawie zapisanych lokalizacji użytkownika.
 */
class MapsActivity : FragmentActivity(), OnMapReadyCallback {
    /**
     * Obiekt mapy Google.
     */
    private var mMap: GoogleMap? = null

    /**
     * Powiązanie z layoutem aktywności.
     */
    private var binding: ActivityMapsBinding? = null

    /**
     * Lista zapisanych lokalizacji.
     */
    var savedLocations: ArrayList<MyLocation>? = null

    /**
     * Lista wygładzonych współrzędnych GPS.
     */
    private var smoothedLatLngList: List<LatLng>? = null

    /**
     * Lista szerokości geograficznych.
     */
    private var latitudes: MutableList<Double> = arrayListOf()

    /**
     * Lista długości geograficznych.
     */
    private var longitudes: MutableList<Double> = arrayListOf()

    /**
     * Lista wysokości n.p.m.
     */
    private var altitudes: MutableList<Double> = arrayListOf()

    /**
     * Lista wartości przyspieszenia w osi X.
     */
    private var accX: MutableList<Double> = arrayListOf()

    /**
     * Lista wartości przyspieszenia w osi Y.
     */
    private var accY: MutableList<Double> = arrayListOf()
    /**
     * Lista wartości przyspieszenia w osi Z.
     */
    private var accZ: MutableList<Double> = arrayListOf()

    /**
     * Lista znaczników czasu.
     */
    private var times: MutableList<Long> = arrayListOf()

    /**
     * Lista dokładności lokalizacji.
     */
    private var accuracies: MutableList<Float> = arrayListOf()

    /**
     * Lista azymutów (orientacji w stopniach).
     */
    private var azimuths: MutableList<Float> = arrayListOf()

    /**
     * Wybrany algorytm wygładzania.
     */
    private var selectedAlgorithm: String? = null

    /**
     * Mapa grup punktów GPS.
     */
    private lateinit var groups: Map<Int, List<Int>>


    /**
     * Funkcja `onCreate` wywoływana przy tworzeniu aktywności.
     * Inicjalizuje mapę oraz dane lokalizacji.
     *
     * @param savedInstanceState Stan zapisany aktywności.
     */
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMapsBinding.inflate(layoutInflater)
        setContentView(binding!!.root)

        val mapFragment = supportFragmentManager
            .findFragmentById(R.id.map) as SupportMapFragment?
        mapFragment!!.getMapAsync(this)

        val application = applicationContext as Application
        savedLocations = application.getLocations()
        selectedAlgorithm = intent.getStringExtra("selectedAlgorithm")
    }

    /**
     * Funkcja `onMapReady` wywoływana, gdy mapa jest gotowa do użycia.
     * Odpowiada za rysowanie markerów i linii na mapie na podstawie przetworzonych danych lokalizacji.
     *
     * @param googleMap Obiekt GoogleMap.
     */
    @RequiresApi(Build.VERSION_CODES.N)
    override fun onMapReady(googleMap: GoogleMap) {
        // Przetwarzanie zapisanych lokalizacji
        for (location in savedLocations!!) {
            latitudes.add(location.latitude)
            longitudes.add(location.longitude)
            location.accelerationX?.let { accX.add(it) }
            location.accelerationY?.let { accY.add(it) }
            location.accelerationZ?.let { accZ.add(it) }
            altitudes.add(location.altitude)
            times.add(location.time)
            accuracies.add(location.accuracy)
            location.azimuth?.let { azimuths.add(it) }
        }
        mMap = googleMap

        // Wybór algorytmu wygładzania na podstawie przekazanej wartości
        when (selectedAlgorithm) {
            "LOWESS" -> {
                val gpsLowess = GpsLowessSmoothing(
                    latitudes.toDoubleArray(),
                    longitudes.toDoubleArray(),
                    0.00085,
                    times,
                    0.01
                )
                gpsLowess.smoothAndEvaluateAndGroup()
                smoothedLatLngList = gpsLowess.getSmoothedLatLngList()
                groups = gpsLowess.getGroups()
            }
            "Simple MA" -> {
                val gpsMA = GpsMovingAvgSmoothing(latitudes.toDoubleArray(),
                    longitudes.toDoubleArray(),
                    0.01,
                    times)
                gpsMA.smoothAndEvaluateAndGroup()
                smoothedLatLngList = gpsMA.getSmoothedLatLngList()
                groups = gpsMA.getGroups()
            }
            "MA with sensory fusion" -> {
                val gpsMAS = GpsMovingAvgWithAzimuth(latitudes.toDoubleArray(),
                    longitudes.toDoubleArray(),
                    0.01,
                    times, azimuths)
                gpsMAS.smoothAndEvaluateAndGroup()
                smoothedLatLngList = gpsMAS.getSmoothedLatLngList()
                groups = gpsMAS.getGroups()
            }
            "Kalman Filter" -> {
                val gpsKF = GpsKalmanPostProcessing(latitudes.toDoubleArray(),
                    longitudes.toDoubleArray(),
                    accX,
                    accY,
                    0.01,
                    times, 5f, accuracies, azimuths)
                gpsKF.smoothAndEvaluateAndGroup()
                smoothedLatLngList = gpsKF.getCorrectedLatLngList()
                groups = gpsKF.getGroups()
            }
            else -> {
                val gpsMA = GPSMovingAvgFuzzySmoothing(latitudes.toDoubleArray(),
                    longitudes.toDoubleArray(),
                    0.01, times)
                gpsMA.smoothAndEvaluateAndGroup()
                smoothedLatLngList = gpsMA.getSmoothedLatLngList()
                groups = gpsMA.getGroups()
            }
        }

        // Dodanie markerów dla wygładzonych punktów GPS
        for ((index, latLng) in smoothedLatLngList?.withIndex()!!) {
            val groupId = getGroupIdForPoint(index, groups)
            mMap!!.addMarker(
                MarkerOptions()
                    .position(latLng)
                    .title("(Group $groupId) Smoothed $latLng ")
                    .icon(BitmapDescriptorFactory.defaultMarker(359.0f))
                    .zIndex(1.0f)
            )
        }

        // Dodanie markerów dla oryginalnych punktów GPS
        for ((groupId, groupPoints) in groups) {
            for (pointIndex in groupPoints) {
                val originalLatLng = LatLng(latitudes[pointIndex], longitudes[pointIndex])
                mMap!!.addMarker(
                    MarkerOptions()
                        .position(originalLatLng)
                        .title("Original Group $groupId Point no. $pointIndex")
                        .icon(BitmapDescriptorFactory.defaultMarker(120.0f))
                        .zIndex(0.0f)
                )
            }
        }

        mMap!!.moveCamera(CameraUpdateFactory.newLatLngBounds(getLatLngBounds(smoothedLatLngList!!), 50))

        // Zapis wygładzonych współrzędnych do pliku
        val outputString = StringBuilder()
        for ((index, latLng) in smoothedLatLngList!!.withIndex()) {
            val groupId = getGroupIdForPoint(index, groups)
            outputString.append("Point $index: Lat: ${latLng.latitude}, Long: ${latLng.longitude}, " +
                    "Group: $groupId, Time: " + times[index].toString() + "\n")
        }
        val file = File(this.getExternalFilesDir(null), "location_info.txt")
        FileOutputStream(file).use {
            it.write(outputString.toString().toByteArray())
        }

        connectSmoothedGroups()
        connectOriginalPoints()
    }

    /**
     * Pobiera identyfikator grupy dla danego punktu GPS.
     *
     * @param pointIndex Indeks punktu GPS.
     * @param groups Mapa grup punktów GPS.
     * @return Identyfikator grupy.
     */
    private fun getGroupIdForPoint(pointIndex: Int, groups: Map<Int, List<Int>>): Int {
        return groups.entries.firstOrNull { it.value.contains(pointIndex) }?.key ?: -1
    }

    /**
     * Tworzy granice mapy na podstawie listy współrzędnych GPS.
     *
     * @param latLngList Lista współrzędnych GPS.
     * @return Obiekt LatLngBounds obejmujący wszystkie punkty.
     */
    private fun getLatLngBounds(latLngList: List<LatLng>): LatLngBounds {
        val builder = LatLngBounds.Builder()
        for (latLng in latLngList) {
            builder.include(latLng)
        }
        return builder.build()
    }

    /**
     * Łączy wygładzone punkty GPS w grupach za pomocą linii.
     */
    private fun connectSmoothedGroups() {
        val groupIndicesList = groups.keys.sorted()
        for (element in 0 until groupIndicesList.size - 1) {
            val currentGroup = groups[element]!!
            val nextGroup = groups[groupIndicesList[element + 1]]!!
            val polylineOptions = PolylineOptions()
            var latLng1: LatLng = smoothedLatLngList!![0]
            for (index in currentGroup) {
                val latLng = smoothedLatLngList!![index]
                polylineOptions.add(latLng)
                polylineOptions.color(Color.BLUE)
                polylineOptions.width(7f)
                latLng1 = latLng
            }
            val latLng2: LatLng = smoothedLatLngList!![nextGroup[0]]
            if (shouldConnect(latLng1.latitude, latLng2.latitude, latLng1.longitude, latLng2.longitude, 1.0)) {
                for (index in nextGroup) {
                    val latLng = smoothedLatLngList!![index]
                    polylineOptions.add(latLng)
                    polylineOptions.color(Color.BLUE)
                    polylineOptions.width(7f)
                }
            }
            mMap?.addPolyline(polylineOptions)
        }
    }

    /**
     * Łączy oryginalne punkty GPS za pomocą linii.
     */
    private fun connectOriginalPoints() {
        for (i in 0 until longitudes.size - 1) {
            if (shouldConnect(latitudes[i], latitudes[i + 1], longitudes[i], longitudes[i + 1], 1.0)) {
                val latLng = LatLng(latitudes[i], longitudes[i])
                val latLng2 = LatLng(latitudes[i + 1], longitudes[i + 1])
                mMap?.addPolyline(
                    PolylineOptions()
                        .add(latLng, latLng2).color(Color.YELLOW).width(5f)
                )
            }
        }
    }

    /**
     * Sprawdza, czy dwa punkty GPS powinny być połączone na podstawie ich odległości.
     *
     * @param latitude Szerokość geograficzna pierwszego punktu.
     * @param latitude1 Szerokość geograficzna drugiego punktu.
     * @param longitude Długość geograficzna pierwszego punktu.
     * @param longitude1 Długość geograficzna drugiego punktu.
     * @param threshold Próg odległości w kilometrach.
     * @return True, jeśli punkty powinny być połączone.
     */
    private fun shouldConnect(latitude: Double, latitude1: Double, longitude: Double, longitude1: Double, threshold: Double): Boolean {
        val r = 6371.0
        val dLat = Math.toRadians(latitude1 - latitude)
        val dLon = Math.toRadians(longitude1 - longitude)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(latitude)) * cos(Math.toRadians(latitude1)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return r * c < threshold
    }
}