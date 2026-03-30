package sigmacorns.control.aim.ballistic

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.google.gson.reflect.TypeToken
import sigmacorns.subsystem.HoodConfig
import java.io.File

/**
 * Calibration map: v_exit -> flywheel omega.
 *
 * Currently interpolates omega as a 1D function of exit velocity.
 * The phi field in [FlywheelMapPoint] is stored for future 2D interpolation
 * but is not yet used in lookup. Falls back to the physics formula
 * omega = v_exit / (radius * efficiency) when no data is available.
 */
class FlywheelMap(private val dataFile: String = DEFAULT_DATA_FILE) {

    companion object {
        const val DEFAULT_DATA_FILE = "/sdcard/FIRST/optimized_shot_tuning_data.json"
    }

    private val points = mutableListOf<FlywheelMapPoint>()
    private var sortedByVExit: List<FlywheelMapPoint> = emptyList()
    private val gson: Gson = GsonBuilder().setPrettyPrinting().create()

    /**
     * Get the flywheel omega for a given exit velocity.
     * Interpolates linearly between calibration points sorted by v_exit.
     * Returns physics fallback when fewer than 2 calibration points exist.
     */
    fun getOmega(vExit: Double): Double {
        val sorted = sortedByVExit
        if (sorted.size < 2) return physicsFallback(vExit)

        // Clamp to data range
        if (vExit <= sorted.first().vExit) return sorted.first().omega
        if (vExit >= sorted.last().vExit) return sorted.last().omega

        // Find bracketing points
        val upperIdx = sorted.indexOfFirst { it.vExit >= vExit }
        val lower = sorted[upperIdx - 1]
        val upper = sorted[upperIdx]

        val t = (vExit - lower.vExit) / (upper.vExit - lower.vExit)
        return lower.omega + t * (upper.omega - lower.omega)
    }

    /**
     * Get the maximum exit velocity achievable within flywheel limits.
     */
    fun maxExitVelocity(): Double {
        if (sortedByVExit.isEmpty()) {
            return BallisticConstants.OMEGA_MAX * HoodConfig.flywheelRadius * HoodConfig.launchEfficiency
        }
        return sortedByVExit.last().vExit
    }

    private fun physicsFallback(vExit: Double): Double {
        return vExit / (HoodConfig.flywheelRadius * HoodConfig.launchEfficiency)
    }

    private fun rebuildSortedCache() {
        sortedByVExit = points.sortedBy { it.vExit }
    }

    fun load() {
        try {
            val file = File(dataFile)
            if (!file.exists()) return

            val json = file.readText()
            val type = object : TypeToken<List<FlywheelMapPoint>>() {}.type
            val loaded: List<FlywheelMapPoint> = gson.fromJson(json, type) ?: return

            points.clear()
            points.addAll(loaded)
            rebuildSortedCache()
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    fun save() {
        try {
            val json = gson.toJson(points.toList())
            File(dataFile).writeText(json)
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    fun addPoint(point: FlywheelMapPoint) {
        points.add(point)
        rebuildSortedCache()
    }

    fun getPoints(): List<FlywheelMapPoint> = points.toList()
}

/**
 * A calibration point mapping exit velocity to flywheel omega.
 * The phi field is stored for future 2D interpolation.
 */
data class FlywheelMapPoint(
    val phi: Double,    // hood angle in radians (stored, not yet used in lookup)
    val vExit: Double,  // ball exit velocity in m/s
    val omega: Double   // flywheel angular velocity in rad/s
)
