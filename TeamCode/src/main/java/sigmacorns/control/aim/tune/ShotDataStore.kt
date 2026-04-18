package sigmacorns.control.aim.tune

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import java.io.File

/**
 * Stores distance-to-speed data points and persists to JSON.
 * Simple sorted list of (distance, speed) points used for linear interpolation.
 */
class ShotDataStore(private val dataFile: String = DEFAULT_DATA_FILE) {
    companion object {
        private const val DEFAULT_DATA_FILE = "/sdcard/FIRST/shot_tuning_data.json"
        private const val CSV_FILE = "/sdcard/FIRST/shot_tuning_data.csv"
    }

    private val points = mutableListOf<SpeedPoint>()
    private val gson: Gson = GsonBuilder().setPrettyPrinting().create()

    @Synchronized
    fun addPoint(point: SpeedPoint) {
        points.add(point)
        points.sortBy { it.distance }
    }

    @Synchronized
    fun updatePoint(index: Int, point: SpeedPoint) {
        if (index in points.indices) {
            points[index] = point
            points.sortBy { it.distance }
        }
    }

    @Synchronized
    fun removePoint(index: Int) {
        if (index in points.indices) {
            points.removeAt(index)
        }
    }

    @Synchronized
    fun getPoints(): List<SpeedPoint> = points.toList()

    @Synchronized
    fun getPoints(includePhysicsEstimates: Boolean): List<SpeedPoint> =
        if (includePhysicsEstimates) points.toList()
        else points.filter { !it.physicsEstimate }

    @Synchronized
    fun getPhysicsEstimateCount(): Int = points.count { it.physicsEstimate }

    @Synchronized
    fun getEmpiricalCount(): Int = points.count { !it.physicsEstimate }

    @Synchronized
    fun getPointsSorted(): List<SpeedPoint> = points.sortedBy { it.distance }

    @Synchronized
    fun exportPointsJson(): String = gson.toJson(getPointsSorted())

    @Synchronized
    fun save() {
        try {
            val json = gson.toJson(points.toList())
            File(dataFile).writeText(json)
        } catch (e: Exception) {
            e.printStackTrace()
        }
        saveCsv()
    }

    private fun saveCsv() {
        try {
            val csvFile = File(CSV_FILE)
            val sb = StringBuilder("distance_m,speed_rad_s,speed_rpm,hood_angle_deg,physics_estimate\n")
            for (p in points) {
                val rpm = p.speed * 60.0 / (2.0 * Math.PI)
                sb.append("%.3f,%.1f,%.0f,%.1f,%s\n".format(p.distance, p.speed, rpm, p.hoodAngle, p.physicsEstimate))
            }
            csvFile.writeText(sb.toString())
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    @Synchronized
    fun load() {
        try {
            val file = File(dataFile)
            if (!file.exists()) return

            val json = file.readText()
            val type = object : com.google.gson.reflect.TypeToken<List<SpeedPoint>>() {}.type
            val loaded: List<SpeedPoint> = gson.fromJson(json, type) ?: return

            points.clear()
            points.addAll(loaded)
            points.sortBy { it.distance }
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    @Synchronized
    fun exportCsv(): String {
        val sb = StringBuilder("distance_m,speed_rad_s,speed_rpm,hood_angle_deg,physics_estimate\n")
        for (p in points.sortedBy { it.distance }) {
            val rpm = p.speed * 60.0 / (2.0 * Math.PI)
            sb.append("%.3f,%.1f,%.0f,%.1f,%s\n".format(p.distance, p.speed, rpm, p.hoodAngle, p.physicsEstimate))
        }
        return sb.toString()
    }

    @Synchronized
    fun clear() {
        points.clear()
    }
}
