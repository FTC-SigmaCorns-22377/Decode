package sigmacorns.subsystem

import sigmacorns.Robot
import sigmacorns.control.PIDController
import sigmacorns.control.SlewRateLimiter
import sigmacorns.math.normalizeAngle
import sigmacorns.opmode.tune.TurretPIDConfig
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.time.Duration

/**
 * PID-controlled dual-servo geared turret.
 *
 * Two servos (left and right) are geared to a single output gear.
 * The right servo is hardware-reversed so both receive the same
 * position value. Uses PID with encoder feedback for precise positioning,
 * then maps the PID output to servo positions.
 *
 * Supports field-relative aiming, slew-rate limiting, angle aliasing,
 * static friction compensation, and robot angular velocity feedforward.
 */
class Turret(val robot: Robot) {

    val angleController: PIDController = PIDController(
        TurretPIDConfig.kP, TurretPIDConfig.kD, TurretPIDConfig.kI, 0.0
    )
    val slewRateLimiter: SlewRateLimiter = SlewRateLimiter(maxRate = TurretPIDConfig.slewRate)
    val outputSlewRateLimiter: SlewRateLimiter =
        SlewRateLimiter(maxRate = TurretPIDConfig.outputSlewRate)

    /** Turret angle limits in radians. */
    val angleLimits: ClosedRange<Double> = TurretServoConfig.minAngle..TurretServoConfig.maxAngle

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

    // Current turret position in radians (from servo feedback or last commanded)
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
    /** The target angle lookahead offset based on shoot while move dynamics */
    var targetAngleOffset: Double = 0.0

    /** The actual servo position being commanded (for telemetry) */
    var currentServoPosition: Double = 0.5
        private set

    private var lastSaturatedTarget: Double? = null
    private var lastSaturatedError: Double? = null
    private val saturationHysteresis = 0.05  // rad - only switch if error improves by this much

    fun update(dt: Duration) {
        // Read current position from IO (turretPosition returns encoder ticks or last commanded)
        val currentAngle = pos

        // Determine the raw target angle (field-relative or robot-relative)
        val rawTarget = normalizeAngle(
            if (fieldRelativeMode) {
                // Convert field-relative to robot-relative
                fieldTargetAngle - robotHeading
            } else {
                targetAngle
            } + targetAngleOffset
        )

        goalTargetAngle = rawTarget

        // Check aliases
        val candidates = listOf(rawTarget, rawTarget - 2 * PI, rawTarget + 2 * PI)
        val validCandidates = candidates.filter { it in angleLimits }

        val targetToUse = if (validCandidates.isNotEmpty()) {
            validCandidates.minByOrNull { abs(it - currentAngle) }!!
        } else {
            // Saturated - use hysteresis to prevent rapid switching
            val start = angleLimits.start
            val end = angleLimits.endInclusive

            // Lookahead based on robot angular velocity
            val lookaheadTime = 0.5 // seconds
            val projectedTarget = normalizeAngle(rawTarget - robotAngularVelocity * lookaheadTime)

            val distStart = (normalizeAngle(projectedTarget - start)).absoluteValue
            val distEnd = (normalizeAngle(projectedTarget - end)).absoluteValue

            val preferredTarget = if (distStart < distEnd) start else end
            val preferredError = min(distStart, distEnd)

            if (lastSaturatedTarget == null) {
                lastSaturatedTarget = preferredTarget
                lastSaturatedError = preferredError
                preferredTarget
            } else if (preferredError < lastSaturatedError!! - saturationHysteresis) {
                lastSaturatedTarget = preferredTarget
                lastSaturatedError = preferredError
                preferredTarget
            } else {
                lastSaturatedTarget!!
            }
        }

        val limitTarget = targetToUse.coerceIn(angleLimits)

        // Apply slew rate limiting to the target if enabled
        val slewLimitedTarget = if (slewRateLimitingEnabled) {
            slewRateLimiter.calculate(limitTarget, dt)
        } else {
            limitTarget
        }

        // Clamp target lead relative to current position
        val lead = TurretPIDConfig.maxTargetLead
        val leadClampedTarget = if (lead > 0.0) {
            val minTarget = maxOf(angleLimits.start, currentAngle - lead)
            val maxTarget = minOf(angleLimits.endInclusive, currentAngle + lead)
            slewLimitedTarget.coerceIn(min(maxTarget, minTarget), max(minTarget, maxTarget))
        } else {
            slewLimitedTarget
        }

        // Clamp to turret limits
        effectiveTargetAngle = leadClampedTarget.coerceIn(angleLimits)

        // Update PID gains from config (allows live tuning)
        angleController.kp = TurretPIDConfig.kP
        angleController.kd = TurretPIDConfig.kD
        angleController.ki = TurretPIDConfig.kI
        angleController.setpoint = effectiveTargetAngle

        // Calculate PID output
        var turretPower = angleController.update(currentAngle, dt).coerceIn(-1.0, 1.0)

        // Static friction compensation
        if ((pos - effectiveTargetAngle).absoluteValue > staticCompensationThresh) {
            turretPower += staticPower * turretPower.sign
        }

        // Robot angular velocity feedforward (field-relative mode)
        val robotAngularFeedforward = if (fieldRelativeMode) {
            -TurretPIDConfig.kVRobot * robotAngularVelocity
        } else {
            0.0
        }
        turretPower = (turretPower + robotAngularFeedforward).coerceIn(-1.0, 1.0)

        // Output slew rate limiting
        outputSlewRateLimiter.maxRate = TurretPIDConfig.outputSlewRate
        val power = outputSlewRateLimiter.calculate(turretPower, dt)

        // Convert PID power output to servo position:
        // power [-1, 1] maps to servo delta from center
        // Center = 0.5, full range = [0.0, 1.0]
        // Integrate PID power to update position (servo acts as velocity-controlled)
        val dtSec = dt.inWholeMilliseconds / 1000.0
        val angleRate = power * TurretServoConfig.maxAngularVelocity
        pos = (pos + angleRate * dtSec).coerceIn(angleLimits)

        // Convert angle to servo position and write to both servos
        currentServoPosition = angleToServo(pos)
        robot.io.turretLeft = currentServoPosition
        robot.io.turretRight = currentServoPosition  // hardware-reversed
    }

    /** Map radians to servo position [0.0, 1.0]. */
    private fun angleToServo(angle: Double): Double {
        val range = angleLimits.endInclusive - angleLimits.start
        return ((angle - angleLimits.start) / range).coerceIn(0.0, 1.0)
    }

    /** Map servo position [0.0, 1.0] back to radians. */
    private fun servoToAngle(servo: Double): Double {
        val range = angleLimits.endInclusive - angleLimits.start
        return angleLimits.start + servo * range
    }
}

/**
 * Tunable constants for the servo-based turret.
 */
object TurretServoConfig {
    /** Minimum turret angle in radians */
    @JvmField var minAngle = -PI / 2.0

    /** Maximum turret angle in radians */
    @JvmField var maxAngle = PI / 2.0

    /** Slew rate limit (rad/s) for smooth target movement */
    @JvmField var slewRate = 3.0

    /** Maximum angular velocity the servos can achieve (rad/s) - maps PID power to angle rate */
    @JvmField var maxAngularVelocity = PI  // π rad/s ≈ 180°/s
}
