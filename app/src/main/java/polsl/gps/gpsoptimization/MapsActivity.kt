package polsl.gps.gpsoptimization
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.util.Log
import androidx.annotation.RequiresApi
import androidx.fragment.app.FragmentActivity
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.OnMapReadyCallback
import com.google.android.gms.maps.SupportMapFragment
import com.google.android.gms.maps.model.*
import polsl.gps.gpsoptimization.databinding.ActivityMapsBinding
import com.google.android.gms.maps.model.LatLng
import java.io.File
import java.io.FileOutputStream


class MapsActivity : FragmentActivity(), OnMapReadyCallback {
    private var mMap: GoogleMap? = null
    private var binding: ActivityMapsBinding? = null
    var savedLocations: ArrayList<MyLocation>? = null
    private var smoothedLatLngList: List<LatLng>? = null
    private var latitudes: MutableList<Double> = arrayListOf()
    private var longitudes: MutableList<Double> = arrayListOf()
    private var accX: MutableList<Double> = arrayListOf()
    private var accY: MutableList<Double> = arrayListOf()
    private var selectedAlgorithm: String? = null
    private lateinit var groups: Map<Int, List<Int>>
    private lateinit var mae: DoubleArray
    var trueLats = DoubleArray(4)
    var trueLongs = DoubleArray(4)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMapsBinding.inflate(layoutInflater)
        setContentView(binding!!.getRoot())
        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        var mapFragment = supportFragmentManager
            .findFragmentById(R.id.map) as SupportMapFragment?
        mapFragment!!.getMapAsync(this)
        val application = applicationContext as Application
        savedLocations = application.getLocations()
        // Odczytaj przekazaną wartość z Intentu
        selectedAlgorithm = intent.getStringExtra("selectedAlgorithm")
        // Sprawdź czy wartość została pomyślnie odczytana
        if (selectedAlgorithm != null) {
            Log.d("MapsActivity", "XWybrany algorytm: $selectedAlgorithm")
        } else {
            Log.e("MapsActivity", "XBrak przekazanej wartości lub błąd odczytu")
        }
    }

    /**
     * Manipulates the map once available.
     * This callback is triggered when the map is ready to be used.
     * This is where we can add markers or lines, add listeners or move the camera. In this case,
     * we just add a marker near Sydney, Australia.
     * If Google Play services is not installed on the device, the user will be prompted to install
     * it inside the SupportMapFragment. This method will only be triggered once the user has
     * installed Google Play services and returned to the app.
     */
    @RequiresApi(Build.VERSION_CODES.N)
    override fun onMapReady(googleMap: GoogleMap) {
        for (location in savedLocations!!){
            latitudes.add(location.latitude)
            longitudes.add(location.longitude)
            location.velocityX?.let { accX.add(it) }
            location.velocityY?.let { accY.add(it) }
        }
        mMap = googleMap
        trueLats[0] = 50.13489
        //trueLats[1] = 50.12968
        //trueLats[2] = 50.29099
        //trueLats[3] = 50.136036378069974
        trueLongs[0] = 19.43183
        //trueLongs[1] = 19.43375
        //trueLongs[2] = 18.67301
        //trueLongs[3] = 19.435716129309384

        if(selectedAlgorithm == "LOWESS") {
            val gpsLowess = GpsLowessSmoothing(
                latitudes.toDoubleArray(),
                longitudes.toDoubleArray(),
                0.005,
                trueLats,
                trueLongs
            )
            gpsLowess.smoothAndEvaluateAndGroup(0.000089)
            smoothedLatLngList = gpsLowess.getSmoothedLatLngList()
            groups = gpsLowess.getGroups()
            mae = gpsLowess.getMAE()
        }
        else if(selectedAlgorithm == "Simple MA")
        {
            val gpsMA = GpsMovingAvgSmoothing(latitudes.toDoubleArray(),
                longitudes.toDoubleArray(),
                0.001,
                trueLats,
                trueLongs)
            gpsMA.smoothAndEvaluateAndGroup()
            smoothedLatLngList = gpsMA.getSmoothedLatLngList()
            groups = gpsMA.getGroups()
            mae = gpsMA.getMAE()
        }
        else if(selectedAlgorithm == "Kalman Filter")
        {
            val gpsKF = GpsKalmanPostProcessing(latitudes.toDoubleArray(),
            longitudes.toDoubleArray(),
            accX,
            accY,
            0.001,
            trueLats,
            trueLongs)
            gpsKF.applyPostProcessingAndGroup()
            smoothedLatLngList=gpsKF.getCorrectedLatLngList()
            groups = gpsKF.getGroups()
            mae = gpsKF.getMAE()
        }
        else
        {
            val gpsMA = GPSMovingAvgFuzzySmoothing(latitudes.toDoubleArray(),
                longitudes.toDoubleArray(),
                0.001,
                trueLats,
                trueLongs)
            gpsMA.smoothAndEvaluateAndGroup()
            smoothedLatLngList = gpsMA.getSmoothedLatLngList()
            groups = gpsMA.getGroups()
            mae = gpsMA.getMAE()
        }
        for ((index, latLng) in smoothedLatLngList?.withIndex()!!) {
            val groupId = getGroupIdForPoint(index, groups)
            val maeForPoint = mae[index]
                mMap!!.addMarker(
                    MarkerOptions()
                        .position(latLng)
                        .title("(Group $groupId) MAE$maeForPoint Smoothed $latLng ")
                        .icon(BitmapDescriptorFactory.defaultMarker(getMarkerColor(maeForPoint)))
                        .zIndex(1.0f)
                )
        }

        for ((groupId, groupPoints) in groups) {
            for (pointIndex in groupPoints) {
                val originalLatLng = LatLng(latitudes[pointIndex], longitudes[pointIndex])
                mMap!!.addMarker(
                    MarkerOptions()
                        .position(originalLatLng)
                        .title("Original Group $groupId")
                        .icon(BitmapDescriptorFactory.defaultMarker(120.0f))
                        .zIndex(0.0f)
                )
            }
        }

        mMap!!.moveCamera(CameraUpdateFactory.newLatLngBounds(getLatLngBounds(smoothedLatLngList!!), 50))
        val outputString = StringBuilder()
        for ((index, latLng) in smoothedLatLngList!!.withIndex()) {
            val groupId = getGroupIdForPoint(index, groups)
            val maeForPoint = mae[index]
            outputString.append("Point $index: Lat: ${latLng.latitude}, Long: ${latLng.longitude}, MAE: $maeForPoint, Group: $groupId\n")
        }
        val file = File(this.getExternalFilesDir(null), "location_info.txt")
        FileOutputStream(file).use {
            it.write(outputString.toString().toByteArray())
        }
        connectPoints()
        ///TMP to poniżej
        val polylineOptions = PolylineOptions()
        for (index in smoothedLatLngList!!)
        {
            polylineOptions.add(index)
            polylineOptions.color(Color.BLUE)
            polylineOptions.width(5f)
        }
        mMap!!.addPolyline(polylineOptions)

    }
    private fun getGroupIdForPoint(pointIndex: Int, groups: Map<Int, List<Int>>): Int {
        return groups.entries.firstOrNull { it.value.contains(pointIndex) }?.key ?: -1
    }
    private fun getMarkerColor(mae: Double): Float {
        // Funkcja do przypisywania koloru markerowi na podstawie wartości błędu MAE
        // Możesz dostosować tę funkcję w zależności od swoich potrzeb
        val maxMae = 0.1 // Maksymalna wartość MAE dla maksymalnego koloru
        val minMae = 0.0 // Minimalna wartość MAE dla minimalnego koloru
        val hue = (mae.coerceIn(minMae, maxMae) / maxMae) * 120 // Wartości do 120, aby ograniczyć zakres kolorów
        if (hue < 0 || hue > 360 || hue == Double.NaN)   return 0.0f
        return hue.toFloat()
    }
    private fun getLatLngBounds(latLngList: List<LatLng>): LatLngBounds {
        val builder = LatLngBounds.Builder()
        for (latLng in latLngList) {
            builder.include(latLng)
        }
        return builder.build()
    }
    private fun connectPoints() {
        for (i in 0 until (smoothedLatLngList?.size ?: -1) ) {
            for ((groupId, groupPoints) in groups) {
                val origPoints = mutableListOf<LatLng>()
                val polylineOptions = PolylineOptions()
                val polylineOptions2 = PolylineOptions()
                for (pointIndex in groupPoints){
                    polylineOptions.add(smoothedLatLngList?.get(pointIndex))
                    origPoints.add(LatLng(latitudes[pointIndex], longitudes[pointIndex]))
                    val idx = origPoints.size - 1
                    polylineOptions2.add(origPoints[idx])
                    polylineOptions.color(Color.BLUE)
                    polylineOptions2.color(Color.YELLOW)
                    polylineOptions.width(5f)
                    polylineOptions2.width(5f)
                }
                mMap!!.addPolyline(polylineOptions)
                mMap!!.addPolyline(polylineOptions2)
            }
        }

    }
}