package sigmacorns.subsystem

import sigmacorns.io.SigmaIO
import kotlin.math.atan2
import kotlin.math.sqrt
import kotlin.time.Duration

/**
 * Hood angle controller for the shooter.
 *
 * Adjusts the hood servo to set the optimal launch angle. Uses saved tuning
 * data (distance -> hood angle interpolation) when available, falling back to
 * projectile motion trig when no data exists.
 *
 * Input properties [targetDistance] and [recommendedAngleDeg] are set by the
 * ShooterCoordinator each loop.
 */
class Hood(val io: SigmaIO) {

    /** Whether the hood auto-adjusts based on distance/speed. */
    var autoAdjust: Boolean = true

    /** Manual override angle in radians (used when autoAdjust is false). */
    var manualAngle: Double = Math.toRadians(HoodConfig.defaultAngleDeg)

    /** Distance from robot to goal (set by coordinator). */
    var targetDistance: Double = 3.0

    /** Recommended hood angle in degrees from adaptive tuner (set by coordinator, null if no data). */
    var recommendedAngleDeg: Double? = null

    /** The computed optimal angle in radians (for telemetry). */
    var computedAngle: Double = Math.toRadians(HoodConfig.defaultAngleDeg)
        private set

    /** Current servo position (for telemetry). */
    var currentServoPosition: Double = 0.5
        private set

    /** Whether we used saved data or trig fallback (for telemetry). */
    var usingInterpolatedData: Boolean = false
        private set

    fun update(dt: Duration) {
        val angle = if (autoAdjust) {
            computeAngle(
                distance = targetDistance,
                flywheelVelocity = io.flywheelVelocity()
            )
        } else {
            manualAngle
        }

        computedAngle = angle
        currentServoPosition = angleToServo(angle)
        io.hood = currentServoPosition
    }

    /**
     * Compute the hood angle. Prefers saved tuning data (interpolated),
     * falls back to projectile motion trig.
     */
    fun computeAngle(distance: Double, flywheelVelocity: Double): Double {
        val interpolatedDeg = recommendedAngleDeg
        if (interpolatedDeg != null) {
            usingInterpolatedData = true
            return Math.toRadians(interpolatedDeg)
        }

        // Fall back to trig calculation
        usingInterpolatedData = false
        return computeOptimalAngleTrig(distance, flywheelVelocity)
    }

    /**
     * Compute optimal launch angle using projectile motion.
     * theta = atan((v^2 - sqrt(v^4 - g(g*d^2 + 2h*v^2))) / (g*d))
     */
    fun computeOptimalAngleTrig(distance: Double, flywheelVelocity: Double): Double {
        val v = flywheelVelocity * HoodConfig.flywheelRadius * HoodConfig.launchEfficiency
        val d = distance
        val h = HoodConfig.goalHeight - HoodConfig.launchHeight
        val g = HoodConfig.gravity

        if (v < 0.5 || d < 0.1) {
            return Math.toRadians(HoodConfig.defaultAngleDeg)
        }

        val v2 = v * v
        val v4 = v2 * v2
        val discriminant = v4 - g * (g * d * d + 2.0 * h * v2)

        if (discriminant < 0.0) {
            return Math.toRadians(HoodConfig.maxAngleDeg)
        }

        val numerator = v2 - sqrt(discriminant)
        val denominator = g * d
        val angle = atan2(numerator, denominator)

        return angle.coerceIn(
            Math.toRadians(HoodConfig.minAngleDeg),
            Math.toRadians(HoodConfig.maxAngleDeg)
        )
    }

    private fun angleToServo(angle: Double): Double {
        val minRad = Math.toRadians(HoodConfig.minAngleDeg)
        val maxRad = Math.toRadians(HoodConfig.maxAngleDeg)
        return ((angle - minRad) / (maxRad - minRad)).coerceIn(0.0, 1.0)
    }

    fun servoToAngle(servo: Double): Double {
        val minRad = Math.toRadians(HoodConfig.minAngleDeg)
        val maxRad = Math.toRadians(HoodConfig.maxAngleDeg)
        return minRad + servo * (maxRad - minRad)
    }
}

object HoodConfig {
    @JvmField var goalHeight = 0.75
    @JvmField var launchHeight = 0.22
    @JvmField var gravity = 9.81
    @JvmField var flywheelRadius = 0.05
    @JvmField var launchEfficiency = 0.3
    @JvmField var minAngleDeg = 15.0
    @JvmField var maxAngleDeg = 70.0
    @JvmField var defaultAngleDeg = 45.0
}
