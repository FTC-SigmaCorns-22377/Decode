package sigmacorns.subsystem

import sigmacorns.Robot
import sigmacorns.io.SigmaIO
import sigmacorns.math.normalizeAngle
import sigmacorns.control.SlewRateLimiter
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.sign
import kotlin.time.Duration

/**
 * Servo mode dual-servo geared turret.
 *
 * Two servos (left and right) are geared to a single output gear.
 * The right servo is hardware-reversed so both receive the same
 * position value.
 *
 * Directly commands servo positions from the target angle.
 * Uses analog voltage sensor feedback for alias selection and alignment checking.
 */
class Turret(val io: SigmaIO) {

    /** Turret angle limits in radians. */
    val angleLimits: ClosedRange<Double> = TurretServoConfig.minAngle..TurretServoConfig.maxAngle

    // angle(rad) in robot-relative frame (Yaw) - used when fieldRelativeMode is false
    var targetAngle: Double = 0.0

    // angle(rad) in field-relative frame - used when fieldRelativeMode is true
    var fieldTargetAngle: Double = 0.0

    /** Additional manual offset (rad) added to target in either mode — for fine-tuning auto-aim. */
    var manualOffset: Double = 0.0

    // Current robot heading from odometry (radians)
    var robotHeading: Double = 0.0
    // Current robot angular velocity (rad/s)
    var robotAngularVelocity: Double = 0.0

    /** Whether to use field-relative aiming */
    var fieldRelativeMode: Boolean = false

    /** If non-null, bypasses all angle logic and writes this value directly to both servos. */
    var servoOverride: Double? = null

    /** Whether the localization system is currently trusted */
    var localizationTrusted: Boolean = false

    /** Slew rate limiter for untrusted localization (90 deg/s) */
    private val angleSlewLimiter = SlewRateLimiter(PI*2)

    // Current turret angle in radians from sensor feedback
    var pos = 0.0
        private set

    /** The actual robot-relative target after field conversion and limiting */
    var effectiveTargetAngle: Double = 0.0
        private set
    /** The raw robot-relative target before limiting (for goal tracking) */
    var goalTargetAngle: Double = 0.0
        private set
    /** The actual servo position being commanded (for telemetry) */
    var currentServoPosition: Double = 0.5
        private set

    /** Whether the turret is aligned to the target (within tolerance) */
    var aligned: Boolean = false
        private set

    /** Alignment tolerance in radians */
    var alignmentTolerance: Double = TurretServoConfig.alignmentTolerance

    private var lastSaturatedTarget: Double? = null
    private var lastSaturatedError: Double? = null
    private val saturationHysteresis = 0.05  // rad - only switch if error improves by this much

    fun update(dt: Duration) {
        servoOverride?.let { v ->
            currentServoPosition = v
            io.turretLeft = 1.0 - v
            io.turretRight = v
            return
        }

        // Read current position from analog sensor feedback
        pos = io.turretPosition()

        // Determine the raw target angle (field-relative or robot-relative), plus manual offset
        val rawTarget = normalizeAngle(
            (if (fieldRelativeMode) {
                fieldTargetAngle - robotHeading
            } else {
                targetAngle
            }) + manualOffset
        )

        // Apply localization-trusted clamping or slew rate limiting
        val processedTarget = if (localizationTrusted) {
            // Clamp to 90 degrees (PI/2) from current position, using signed angular difference
            val diff = normalizeAngle(rawTarget - pos)
            val signedDiff = if (diff > PI) diff - 2*PI else diff
            if (abs(signedDiff) > PI/2) {
                val clampedTarget = pos + sign(signedDiff) * PI/2
                normalizeAngle(clampedTarget)
            } else {
                rawTarget
            }
        } else {
            // Slew rate limit using angle traversal (not shortest normalized distance)
            angleSlewLimiter.calculate(rawTarget, dt)
        }

        goalTargetAngle = processedTarget

        // Check aliases - use sensor feedback to pick closest reachable target
        val candidates = listOf(processedTarget, processedTarget - 2 * PI, processedTarget + 2 * PI)
        val validCandidates = candidates.filter { it in angleLimits }

        val targetToUse = if (validCandidates.isNotEmpty()) {
            lastSaturatedTarget = null
            lastSaturatedError = null
            validCandidates.minByOrNull { abs(it - pos) }!!
        } else {
            // Saturated - stay at whichever limit pos is currently closest to,
            // only switching if the error to the other limit improves by more than the hysteresis threshold
            val start = angleLimits.start
            val end = angleLimits.endInclusive

            val preferredTarget = if (abs(pos - start) <= abs(pos - end)) start else end
            val preferredError = min(abs(pos - start), abs(pos - end))

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

        effectiveTargetAngle = targetToUse.coerceIn(angleLimits)

        // Check alignment using sensor feedback
        aligned = abs(pos - effectiveTargetAngle) < alignmentTolerance

        // Convert angle to servo position and write to both servos
        currentServoPosition = angleToServo(effectiveTargetAngle)
        io.turretLeft = 1.0 - currentServoPosition // hardware reversed
        io.turretRight = currentServoPosition
    }

    /** Convert radians to servo position [0.0, 1.0]. 0 rad = 0.5 (forward). */
    private fun angleToServo(angle: Double): Double {
        return (0.5 + ((angle - TurretServoConfig.servoCenterAngle) / TurretServoConfig.servoTotalRange)).coerceIn(0.0, 1.0)
    }
}

/**
 * Tunable constants for the servo-based turret.
 */
object TurretServoConfig {
    /** Total physical rotation of the servo in radians (355 degrees) */
    @JvmField var servoTotalRange = 315.0 * PI / 180.0

    /** Minimum turret angle in radians */
    @JvmField var minAngle = -PI*0.75

    /** Maximum turret angle in radians */
    @JvmField var maxAngle = PI*0.75

    /** Angle in radians that maps to servo position 0.5 */
    @JvmField var servoCenterAngle = 0.0

    /** Alignment tolerance in radians (~2 degrees) */
    @JvmField var alignmentTolerance = 0.035
}