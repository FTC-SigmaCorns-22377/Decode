package sigmacorns.opmode

import sigmacorns.math.Pose2d
import java.io.File

object PosePersistence {
    private const val POSE_FILE_NAME = "last_auto_pose.csv"

    fun save(storageDir: File, pose: Pose2d) {
        runCatching {
            val file = File(storageDir, POSE_FILE_NAME)
            file.parentFile?.mkdirs()
            file.writeText("${pose.v.x},${pose.v.y},${pose.rot}")
        }
    }

    fun load(storageDir: File): Pose2d? {
        return runCatching {
            val file = File(storageDir, POSE_FILE_NAME)
            if (!file.exists()) return null
            val parts = file.readText().trim().split(",")
            if (parts.size != 3) return null
            Pose2d(parts[0].toDouble(), parts[1].toDouble(), parts[2].toDouble())
        }.getOrNull()
    }
}
