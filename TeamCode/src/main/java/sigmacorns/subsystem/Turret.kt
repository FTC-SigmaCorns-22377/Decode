package sigmacorns.subsystem

import sigmacorns.Robot
import sigmacorns.io.SigmaIO
import sigmacorns.math.normalizeAngle
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.min
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

    // Current turret angle in radians from sensor feedback
    var pos = 0.0
        private set

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

    /** Whether the turret is aligned to the target (within tolerance) */
    var aligned: Boolean = false
        private set

    /** Alignment tolerance in radians */
    var alignmentTolerance: Double = TurretServoConfig.alignmentTolerance

    private var lastSaturatedTarget: Double? = null
    private var lastSaturatedError: Double? = null
    private val saturationHysteresis = 0.05  // rad - only switch if error improves by this much

    fun update(dt: Duration) {
        // Read current position from analog sensor feedback
        pos = io.turretPosition()

        // Determine the raw target angle (field-relative or robot-relative)
        val rawTarget = normalizeAngle(
            if (fieldRelativeMode) {
                fieldTargetAngle - robotHeading
            } else {
                targetAngle
            } + targetAngleOffset
        )

        goalTargetAngle = rawTarget

        // Check aliases - use sensor feedback to pick closest reachable target
        val candidates = listOf(rawTarget, rawTarget - 2 * PI, rawTarget + 2 * PI)
        val validCandidates = candidates.filter { it in angleLimits }

        val targetToUse = if (validCandidates.isNotEmpty()) {
            validCandidates.minByOrNull { abs(it - pos) }!!
        } else {
            // Saturated - use hysteresis to prevent rapid switching
            val start = angleLimits.start
            val end = angleLimits.endInclusive

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

        effectiveTargetAngle = targetToUse.coerceIn(angleLimits)

        // Check alignment using sensor feedback
        aligned = abs(pos - effectiveTargetAngle) < alignmentTolerance

        // Convert angle to servo position and write to both servos
        currentServoPosition = angleToServo(effectiveTargetAngle)
        io.turretLeft = currentServoPosition
        io.turretRight = 1-currentServoPosition  // hardware-reversed
    }

    /** Convert radians to servo position [0.0, 1.0]. 0 rad = 0.5 (forward). */
    private fun angleToServo(angle: Double): Double {
        return (0.5 + (angle - TurretServoConfig.servoCenterAngle) / TurretServoConfig.servoTotalRange).coerceIn(0.0, 1.0)
    }
}

/**
 * Tunable constants for the servo-based turret.
 */
object TurretServoConfig {
    /** Total physical rotation of the servo in radians (355 degrees) */
    @JvmField var servoTotalRange = 355.0 * PI / 180.0

    /** Minimum turret angle in radians */
    @JvmField var minAngle = -PI

    /** Maximum turret angle in radians */
    @JvmField var maxAngle = PI

    /** Angle in radians that maps to servo position 0.5 */
    @JvmField var servoCenterAngle = 0.0

    /** Alignment tolerance in radians (~2 degrees) */
    @JvmField var alignmentTolerance = 0.035
}