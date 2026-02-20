package sigmacorns.control.aim

import sigmacorns.control.subsystem.Turret
import sigmacorns.io.SigmaIO
import kotlin.math.hypot
import kotlin.time.Duration

/**
 * Shared logic for updating a turret using auto-aim sensor fusion.
 * Used by both autonomous and teleop opmodes.
 */
object AutoAimTurretController {

    /**
     * Update turret targeting from auto-aim data.
     * Sets turret target angle based on auto-aim fusion and updates turret PID.
     *
     * @param autoAim the auto-aim system providing fused pose and target angles
     * @param turret the turret to control
     * @param io the IO interface for velocity readings
     * @param goalPosition the field-space goal position (x, y) to compute distance
     * @param dt the time delta for PID update
     * @return the computed target distance
     */
    fun update(
        autoAim: AutoAimGTSAM,
        turret: Turret,
        io: SigmaIO,
        goalX: Double,
        goalY: Double,
        dt: Duration,
    ): Double {
        // Update robot heading for field-relative aiming
        turret.robotHeading = autoAim.fusedPose.rot
        turret.robotAngularVelocity = io.velocity().rot

        // Calculate target distance using fused pose and goal position
        val pose = autoAim.fusedPose
        var targetDistance = hypot(goalX - pose.v.x, goalY - pose.v.y)

        if (autoAim.enabled && autoAim.hasTarget) {
            if (turret.fieldRelativeMode) {
                autoAim.getTargetFieldAngle()?.let { fieldAngle ->
                    turret.fieldTargetAngle = fieldAngle
                }
            } else {
                autoAim.getTargetTurretAngle()?.let { robotAngle ->
                    turret.targetAngle = robotAngle
                }
            }
            targetDistance = targetDistance.coerceIn(0.1, 10.0)
        }

        turret.targetDistance = targetDistance
        turret.update(dt)

        return targetDistance
    }
}
