package sigmacorns.tuning

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.google.gson.reflect.TypeToken
import sigmacorns.tuning.model.DistanceSpeedEntry
import sigmacorns.tuning.model.FeedbackType
import sigmacorns.tuning.model.ShotTrial
import java.io.File
import kotlin.math.floor

/**
 * Stores shot trial data and manages the distance-to-speed lookup table.
 * Persists data to JSON file for retention across sessions.
 */
class ShotDataStore {
    companion object {
        private const val DATA_FILE = "/sdcard/FIRST/shot_tuning_data.json"
        const val DISTANCE_BUCKET_SIZE = 0.25  // 25cm buckets
    }

    private val trials = mutableListOf<ShotTrial>()
    private val lookupTable = mutableMapOf<Double, DistanceSpeedEntry>()
    private val gson: Gson = GsonBuilder().setPrettyPrinting().create()

    /**
     * Get distance bucket key for a given distance.
     * Buckets are 0.25m wide, so 2.37m -> 2.25m bucket.
     */
    fun distanceToBucket(distance: Double): Double {
        return floor(distance / DISTANCE_BUCKET_SIZE) * DISTANCE_BUCKET_SIZE
    }

    /**
     * Add a new trial and update lookup table if feedback was "Good".
     */
    @Synchronized
    fun addTrial(trial: ShotTrial) {
        trials.add(trial)

        if (trial.feedbackType == FeedbackType.GOOD) {
            val bucketKey = distanceToBucket(trial.distance)
            val existing = lookupTable[bucketKey]

            if (existing != null) {
                // Average the speeds if we have multiple confirmations
                val newAvgSpeed = (existing.optimalSpeed * existing.confirmedTrials + trial.flywheelSpeed) /
                        (existing.confirmedTrials + 1)
                lookupTable[bucketKey] = existing.copy(
                    optimalSpeed = newAvgSpeed,
                    confirmedTrials = existing.confirmedTrials + 1,
                    lastUpdated = System.currentTimeMillis()
                )
            } else {
                lookupTable[bucketKey] = DistanceSpeedEntry(
                    distanceMin = bucketKey,
                    distanceMax = bucketKey + DISTANCE_BUCKET_SIZE,
                    optimalSpeed = trial.flywheelSpeed,
                    confirmedTrials = 1,
                    lastUpdated = System.currentTimeMillis()
                )
            }
        }
    }

    /**
     * Get optimal speed for a distance (from lookup table or interpolated).
     * Returns null if no data available.
     */
    @Synchronized
    fun getOptimalSpeed(distance: Double): Double? {
        if (lookupTable.isEmpty()) return null

        val bucketKey = distanceToBucket(distance)

        // Direct lookup
        lookupTable[bucketKey]?.let { return it.optimalSpeed }

        // Try interpolation between nearest buckets
        val sortedKeys = lookupTable.keys.sorted()
        val lowerKey = sortedKeys.lastOrNull { it < bucketKey }
        val upperKey = sortedKeys.firstOrNull { it > bucketKey }

        return when {
            lowerKey != null && upperKey != null -> {
                // Linear interpolation
                val lowerSpeed = lookupTable[lowerKey]!!.optimalSpeed
                val upperSpeed = lookupTable[upperKey]!!.optimalSpeed
                val t = (distance - lowerKey) / (upperKey - lowerKey)
                lowerSpeed + t * (upperSpeed - lowerSpeed)
            }
            lowerKey != null -> lookupTable[lowerKey]!!.optimalSpeed
            upperKey != null -> lookupTable[upperKey]!!.optimalSpeed
            else -> null
        }
    }

    /**
     * Get all trials for a specific distance bucket.
     */
    @Synchronized
    fun getTrialsForDistance(distance: Double): List<ShotTrial> {
        val bucketKey = distanceToBucket(distance)
        return trials.filter { distanceToBucket(it.distance) == bucketKey }
            .sortedByDescending { it.timestamp }
    }

    /**
     * Get all trials.
     */
    @Synchronized
    fun getAllTrials(): List<ShotTrial> = trials.toList()

    /**
     * Get the full lookup table.
     */
    @Synchronized
    fun getLookupTable(): Map<Double, DistanceSpeedEntry> = lookupTable.toMap()

    /**
     * Get lookup table as sorted list.
     */
    @Synchronized
    fun getLookupTableSorted(): List<DistanceSpeedEntry> {
        return lookupTable.values.sortedBy { it.distanceMin }
    }

    /**
     * Save data to file.
     */
    @Synchronized
    fun save() {
        try {
            val data = StorageData(
                trials = trials.toList(),
                lookupTable = lookupTable.values.toList()
            )
            val json = gson.toJson(data)
            File(DATA_FILE).writeText(json)
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    /**
     * Load data from file.
     */
    @Synchronized
    fun load() {
        try {
            val file = File(DATA_FILE)
            if (!file.exists()) return

            val json = file.readText()
            val type = object : TypeToken<StorageData>() {}.type
            val data: StorageData = gson.fromJson(json, type) ?: return

            trials.clear()
            trials.addAll(data.trials)

            lookupTable.clear()
            data.lookupTable.forEach { entry ->
                lookupTable[entry.distanceMin] = entry
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    /**
     * Export lookup table as JSON for web display.
     */
    @Synchronized
    fun exportLookupTableJson(): String {
        return gson.toJson(getLookupTableSorted())
    }

    /**
     * Export trials as JSON for web display.
     */
    @Synchronized
    fun exportTrialsJson(distanceBucket: Double? = null): String {
        val filteredTrials = if (distanceBucket != null) {
            getTrialsForDistance(distanceBucket)
        } else {
            trials.takeLast(50).reversed()  // Most recent 50
        }
        return gson.toJson(filteredTrials)
    }

    /**
     * Clear all data.
     */
    @Synchronized
    fun clear() {
        trials.clear()
        lookupTable.clear()
    }

    /**
     * Internal storage format.
     */
    private data class StorageData(
        val trials: List<ShotTrial>,
        val lookupTable: List<DistanceSpeedEntry>
    )
}
