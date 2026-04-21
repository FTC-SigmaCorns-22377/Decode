package sigmacorns.logic

import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.Drivetrain
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Drive toward [BallTrackingSystem.targetBallField] using the existing
 * [Drivetrain] primitive.
 *
 * Behavior:
 *   - No target → zero velocity (hand-off cleanly to manual driver input).
 *   - Have target, far away → move at speed ramped linearly from standoff.
 *   - Within stop radius → hold position (prevents pose-noise jitter from
 *     oscillating us past the ball once the estimate is close).
 *   - Yaw is commanded to point the robot at the ball (simple P on heading
 *     error). If the caller wants headingless chasing, set `rotateToTarget=false`.
 *
 * Coordinate conventions:
 *   - `targetBallField` is in field frame.
 *   - Drivetrain.drive expects robot-frame Pose2d power (x=fwd, y=left, rot=yaw).
 *   - We rotate the field-frame goal vector by -heading to get the robot-frame
 *     direction.
 */
class ChaseCoordinator(
    private val drivetrain: Drivetrain,
    private val tracking: BallTrackingSystem,
    private val io: SigmaIO,
    /** Maximum commanded translational power, 0..1. Scales down near the target. */
    private val maxSpeed: Double = 0.6,
    /** Stop within this distance (meters) to avoid oscillation. */
    private val stopRadiusM: Double = 0.15,
    /** Distance at which we begin ramping speed down toward zero (meters). */
    private val slowDownRadiusM: Double = 0.5,
    /** P gain on heading error (rad -> power). */
    private val headingP: Double = 1.5,
    /** Whether to rotate the robot to face the target. */
    var rotateToTarget: Boolean = true,
    /** True = chase enabled; false = coordinator idles and does not touch the drivetrain. */
    var enabled: Boolean = true,
) {
    /** True on the tick after arrival. Latched until the chase is disabled + re-enabled. */
    var arrived: Boolean = false
        private set

    fun update() {
        if (!enabled) return
        val target = tracking.targetBallField
        if (target == null) {
            drivetrain.drive(Pose2d(0.0, 0.0, 0.0), io)
            return
        }
        val pose = io.position()
        val dx = target.v.x - pose.v.x
        val dy = target.v.y - pose.v.y
        val d = hypot(dx, dy)

        if (d < stopRadiusM) {
            drivetrain.drive(Pose2d(0.0, 0.0, 0.0), io)
            arrived = true
            return
        }
        arrived = false

        val speed = (maxSpeed * (d / slowDownRadiusM)).coerceIn(0.0, maxSpeed)

        // Field-frame unit vector toward target, rotated into robot frame.
        val cosH = cos(pose.rot)
        val sinH = sin(pose.rot)
        val dirX = dx / d
        val dirY = dy / d
        val bodyX =  cosH * dirX + sinH * dirY   // robot forward
        val bodyY = -sinH * dirX + cosH * dirY   // robot left

        val yawCmd = if (rotateToTarget) {
            val desiredHeading = atan2(dy, dx)
            var err = desiredHeading - pose.rot
            while (err >  Math.PI) err -= 2.0 * Math.PI
            while (err < -Math.PI) err += 2.0 * Math.PI
            (headingP * err).coerceIn(-maxSpeed, maxSpeed)
        } else 0.0

        drivetrain.drive(
            Pose2d(speed * bodyX, speed * bodyY, yawCmd),
            io,
        )
    }
}
