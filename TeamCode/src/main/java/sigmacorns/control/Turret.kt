package sigmacorns.control

import sigmacorns.io.SigmaIO
import sigmacorns.opmode.test.TurretPIDConfig
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.time.Duration

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


    fun update(dt: Duration) {
        val currentAngle = range.tickToPos(io.turretPosition())
        pos = currentAngle

        // Determine the raw target angle (field-relative or robot-relative)
        val rawTarget = if (fieldRelativeMode) {
            // Convert field-relative to robot-relative
            fieldTargetAngle - robotHeading
        } else {
            targetAngle
        }

        val limitTarget = rawTarget.coerceIn(range.limits)

        // Apply slew rate limiting to the target if enabled
        val slewLimitedTarget = if (slewRateLimitingEnabled) {
            slewRateLimiter.calculate(limitTarget, dt)
        } else {
            limitTarget
        }

        // Clamp to turret limits
        effectiveTargetAngle = slewLimitedTarget
        angleController.setpoint = effectiveTargetAngle

        // Calculate turret motor power (Yaw)
        var turretPower = angleController.update(currentAngle, dt).coerceIn(-1.0, 1.0)

//        if ((pos - effectiveTargetAngle).absoluteValue > staticCompensationThresh) {
//            turretPower += staticPower * turretPower.sign
//        }

        val flywheelPower = flywheelSpeed()

        outputSlewRateLimiter.maxRate = TurretPIDConfig.outputSlewRate
        io.turret = outputSlewRateLimiter.calculate(turretPower, dt)
        //io.shooter = flywheelPower
    }

    private val MAX_LAUNCH_DIST = 10.0
    private fun flywheelSpeed(): Double {
        return (targetDistance/MAX_LAUNCH_DIST).coerceIn(0.0, 1.0)
    }
}
