package sigmacorns.constants

import org.joml.Quaterniond
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.control.aim.AutoAimGTSAM
import kotlin.math.PI

/**
 * Field landmark positions and goal coordinates shared across all opmodes.
 */
object FieldLandmarks {
    // AprilTag orientations
    private val q20 = Quaterniond().rotateX(-PI / 2.0).rotateLocalZ(Math.toRadians(54.046000))
    private val ypr20 = Vector3d().also { q20.getEulerAnglesZYX(it) }

    private val q24 = Quaterniond().rotateX(-PI / 2.0).rotateLocalZ(-Math.toRadians(54.046000))
    private val ypr24 = Vector3d().also { q24.getEulerAnglesZYX(it) }

    /** All tracked AprilTag landmarks and their 3D positions/orientations. */
    val landmarks: Map<Int, AutoAimGTSAM.LandmarkSpec> = mapOf(
        20 to AutoAimGTSAM.LandmarkSpec(
            Vector3d(-1.413321, 1.481870, 0.7493),
            pitch = ypr20.y,
            roll = ypr20.x,
            yaw = ypr20.z,
            size = 0.165
        ),
        24 to AutoAimGTSAM.LandmarkSpec(
            Vector3d(1.413321, 1.481870, 0.7493),
            pitch = ypr24.y,
            roll = ypr24.x,
            yaw = ypr24.z,
            size = 0.165
        ),
    )

    /** Set of landmark tag IDs for vision tracker filtering. */
    val landmarkTagIds: Set<Int> = landmarks.keys

    /** Goal position for the blue alliance. */
    val blueGoalPosition = Vector2d(-1.580126, 1.598982)

    /** Goal position for the red alliance. */
    val redGoalPosition = Vector2d(1.580126, 1.598982)

    /** Get goal position for an alliance. */
    fun goalPosition(blue: Boolean): Vector2d =
        if (blue) blueGoalPosition else redGoalPosition

    /** Get 3D goal position for an alliance, using the specified goal height. */
    fun goalPosition3d(blue: Boolean, goalHeight: Double): Vector3d {
        val pos2d = goalPosition(blue)
        return Vector3d(pos2d.x, pos2d.y, goalHeight)
    }
}
