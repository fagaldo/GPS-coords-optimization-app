import android.content.Context
import android.util.EventLogTags
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.components.Description
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet
import com.github.mikephil.charting.interfaces.datasets.ILineDataSet

class ChartMaker(private val context: Context) {

    fun setupLineChart(lineChart: LineChart) {
        // Konfiguracja wykresu
        lineChart.setTouchEnabled(true)
        lineChart.setPinchZoom(true)

        // Ustaw opis wykresu
        val description = Description()
        description.text = "GPS Coordinates"
        lineChart.description = description
    }

    fun generateLineChart(lineChart: LineChart, smoothedLatLngList: List<Pair<Double, Double>>, maeList: List<Double>) {
        val lineEntries = mutableListOf<Entry>()
        val maeEntries = mutableListOf<Entry>()

        // Przygotowanie danych dla wykresu
        for ((index, latLng) in smoothedLatLngList.withIndex()) {
            val latLngForPoint = latLng.first.toFloat()
            val maeForPoint = maeList[index].toFloat()
            lineEntries.add(Entry(index.toFloat(), latLngForPoint))
            maeEntries.add(Entry(index.toFloat(), maeForPoint))
        }

        // Utworzenie zestawów danych
        val lineDataSet = LineDataSet(lineEntries, "Smoothed Coordinates")
        val maeDataSet = LineDataSet(maeEntries, "MAE")

        // Dodanie zestawów danych do listy
        val dataSets = ArrayList<ILineDataSet>()
        dataSets.add(lineDataSet)
        dataSets.add(maeDataSet)

        // Utworzenie obiektu LineData i przypisanie zestawów danych
        val lineData = LineData(dataSets)

        // Ustawienie danych na wykresie
        lineChart.data = lineData

        // Odświeżenie wykresu
        lineChart.invalidate()
    }
}