package sigmacorns.io

import sigmacorns.math.Pose2d
import java.io.File
import kotlin.time.Duration.Companion.seconds

/**
 * Utility for persisting robot pose between autonomous and teleop.
 * Saves pose with timestamp to allow teleop to resume from auto's end position.
 */
object PosePersistence {
    private const val POSE_FILE_NAME = "last_auto_pose.txt"
    private val MAX_AGE = 30.seconds

    /**
     * Save the robot pose with current system time.
     * Format: timestamp_ms,x,y,rotation
     */
    fun savePose(pose: Pose2d, storageDir: File) {
        try {
            val file = File(storageDir, POSE_FILE_NAME)
            val timestamp = System.currentTimeMillis()
            val content = "$timestamp,${pose.v.x},${pose.v.y},${pose.rot}"
            file.writeText(content)
            println("PosePersistence: Saved pose to ${file.absolutePath}")
        } catch (e: Exception) {
            println("PosePersistence: Failed to save pose: ${e.message}")
            e.printStackTrace()
        }
    }

    /**
     * Load the robot pose if it exists and is not older than MAX_AGE.
     * Returns null if file doesn't exist, is too old, or is corrupted.
     */
    fun loadPose(storageDir: File): Pose2d? {
        try {
            val file = File(storageDir, POSE_FILE_NAME)
            if (!file.exists()) {
                println("PosePersistence: No saved pose file found")
                return null
            }

            val content = file.readText().trim()
            val parts = content.split(",")
            if (parts.size != 4) {
                println("PosePersistence: Invalid pose file format")
                return null
            }

            val timestamp = parts[0].toLongOrNull()
            val x = parts[1].toDoubleOrNull()
            val y = parts[2].toDoubleOrNull()
            val rot = parts[3].toDoubleOrNull()

            if (timestamp == null || x == null || y == null || rot == null) {
                println("PosePersistence: Failed to parse pose values")
                return null
            }

            val ageMs = System.currentTimeMillis() - timestamp
            if (ageMs > MAX_AGE.inWholeMilliseconds) {
                println("PosePersistence: Pose too old (${ageMs}ms > ${MAX_AGE.inWholeMilliseconds}ms)")
                return null
            }

            val pose = Pose2d(x, y, rot)
            println("PosePersistence: Loaded pose from ${file.absolutePath} (age: ${ageMs}ms)")
            return pose
        } catch (e: Exception) {
            println("PosePersistence: Failed to load pose: ${e.message}")
            e.printStackTrace()
            return null
        }
    }
}
