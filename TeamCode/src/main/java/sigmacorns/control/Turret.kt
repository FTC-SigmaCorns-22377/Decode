package sigmacorns.control

import sigmacorns.io.SigmaIO
import sigmacorns.opmode.tune.TurretPIDConfig
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.time.Duration
import kotlin.time.DurationUnit

class Turret(
    val range: MotorRangeMapper,
    val io: SigmaIO
) {
    val angleController: PIDController = PIDController(
        TurretPIDConfig.kP, TurretPIDConfig.kD, TurretPIDConfig.kI, 0.0)
    val slewRateLimiter: SlewRateLimiter = SlewRateLimiter(maxRate = TurretPIDConfig.slewRate)
    val outputSlewRateLimiter: SlewRateLimiter = SlewRateLimiter(maxRate = TurretPIDConfig.outputSlewRate)

    // distance(m) from target
    var targetDistance: Double = 0.0

    // angle(rad) in robot-relative frame (Yaw) - used when fieldRelativeMode is false
    var targetAngle: Double = 0.0

    // angle(rad) in field-relative frame - used when fieldRelativeMode is true
    var fieldTargetAngle: Double = 0.0

    // Current robot heading from odometry (radians)
    var robotHeading: Double = 0.0
    // Current robot angular velocity (rad/s)
    var robotAngularVelocity: Double = 0.0

    /** Whether to use field-relative aiming */
    var fieldRelativeMode: Boolean = false

    // Pitch value for servo (0.0 to 1.0)
    var targetPitch: Double = 0.0

    var pos = 0.0

    var staticCompensationThresh = 0.01
    var staticPower = 0.03

    /** Whether slew rate limiting is enabled */
    var slewRateLimitingEnabled: Boolean = true

    /** The actual robot-relative target after field conversion and limiting */
    var effectiveTargetAngle: Double = 0.0
        private set
    /** The raw robot-relative target before limiting (for goal tracking) */
    var goalTargetAngle: Double = 0.0
        private set

    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle
        while (normalized > kotlin.math.PI) normalized -= 2 * kotlin.math.PI
        while (normalized < -kotlin.math.PI) normalized += 2 * kotlin.math.PI
        return normalized
    }

    fun update(dt: Duration) {
        val currentAngle = range.tickToPos(io.turretPosition())
        pos = currentAngle

        // Determine the raw target angle (field-relative or robot-relative)
        val rawTarget = if (fieldRelativeMode) {
            // Convert field-relative to robot-relative
            normalizeAngle(fieldTargetAngle - robotHeading)
        } else {
            normalizeAngle(targetAngle)
        }

        goalTargetAngle = rawTarget

        // Check aliases
        val candidates = listOf(rawTarget, rawTarget - 2 * kotlin.math.PI, rawTarget + 2 * kotlin.math.PI)
        val validCandidates = candidates.filter { it in range.limits }

        val targetToUse = if (validCandidates.isNotEmpty()) {
            validCandidates.minByOrNull { kotlin.math.abs(it - currentAngle) }!!
        } else {
            // Saturated
            val start = range.limits.start
            val end = range.limits.endInclusive

            // Lookahead based on robot angular velocity
            val lookaheadTime = 0.5 // seconds
            val projectedTarget = normalizeAngle(rawTarget - robotAngularVelocity * lookaheadTime)

            val distStart = kotlin.math.abs(normalizeAngle(projectedTarget - start))
            val distEnd = kotlin.math.abs(normalizeAngle(projectedTarget - end))

            if (distStart < distEnd) start else end
        }

        val limitTarget = targetToUse.coerceIn(range.limits)

        // Apply slew rate limiting to the target if enabled
        val slewLimitedTarget = if (slewRateLimitingEnabled) {
            slewRateLimiter.calculate(limitTarget, dt)
        } else {
            limitTarget
        }

        // Clamp target lead relative to current position (optional)
        val lead = TurretPIDConfig.maxTargetLead
        val leadClampedTarget = if (lead > 0.0) {
            val minTarget = maxOf(range.limits.start, currentAngle - lead)
            val maxTarget = minOf(range.limits.endInclusive, currentAngle + lead)
            slewLimitedTarget.coerceIn(min(maxTarget,minTarget), max(minTarget,maxTarget))
        } else {
            slewLimitedTarget
        }

        // Clamp to turret limits
        effectiveTargetAngle = leadClampedTarget.coerceIn(range.limits)
        angleController.setpoint = effectiveTargetAngle

        // Calculate turret motor power (Yaw)
        var turretPower = angleController.update(currentAngle, dt).coerceIn(-1.0, 1.0)

        if ((pos - effectiveTargetAngle).absoluteValue > staticCompensationThresh) {
            turretPower += staticPower * turretPower.sign
        }

        val robotAngularFeedforward = if (fieldRelativeMode) {
            -TurretPIDConfig.kVRobot * robotAngularVelocity
        } else {
            0.0
        }
        turretPower = (turretPower + robotAngularFeedforward).coerceIn(-1.0, 1.0)

        outputSlewRateLimiter.maxRate = TurretPIDConfig.outputSlewRate
        val power = outputSlewRateLimiter.calculate(turretPower, dt)

        io.turret = power / (io.voltage()/12.0)
    }
}
