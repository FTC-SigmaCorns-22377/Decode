package sigmacorns.subsystem

import sigmacorns.control.PIDController
import sigmacorns.io.SigmaIO
import kotlin.math.atan2
import kotlin.math.sqrt
import kotlin.time.Duration

/**
 * Unified flywheel + hood shooter subsystem.
 *
 * The flywheel and hood are a single mechanical unit: the flywheel spins up to
 * launch balls, and the hood servo controls the launch angle. Both take `SigmaIO`
 * and write their respective outputs each loop.
 *
 * Flywheel uses a PID controller with feedforward to reach the target velocity.
 * Hood computes the optimal launch angle from distance and flywheel velocity,
 * preferring interpolated tuning data over trig fallback.
 *
 * Input properties [targetDistance], [recommendedHoodAngleDeg], and [flywheelTarget]
 * are set by the AimingSystem each loop.
 */
class Shooter(
    kP: Double,
    kD: Double,
    kI: Double,
    val maxVelocity: Double,
    val io: SigmaIO,
) {

    // --- Flywheel state ---

    val pid = PIDController(kP, kD, kI, 0.0)

    /** Target flywheel velocity in rad/s (set by AimingSystem or opmode). */
    var flywheelTarget: Double = 0.0
        set(value) {
            if (field != value) {
                field = value
                pid.setpoint = value
            }
        }

    // --- Hood state ---

    /** Whether the hood auto-adjusts based on distance/speed. */
    var autoAdjust: Boolean = true

    /** Manual override angle in radians (used when autoAdjust is false). */
    var manualHoodAngle: Double = Math.toRadians(ShooterConfig.defaultAngleDeg)

    /** Distance from robot to goal (set by AimingSystem). */
    var targetDistance: Double = 3.0

    /** Recommended hood angle in degrees from adaptive tuner (set by AimingSystem, null if no data). */
    var recommendedHoodAngleDeg: Double? = null

    /** The computed optimal hood angle in radians (for telemetry). */
    var computedHoodAngle: Double = Math.toRadians(ShooterConfig.defaultAngleDeg)
        private set

    /** Current hood servo position (for telemetry). */
    var hoodServoPosition: Double = 0.5
        private set

    /** Whether we used saved data or trig fallback (for telemetry). */
    var usingInterpolatedData: Boolean = false
        private set

    // ========================================================================
    // Update
    // ========================================================================

    /**
     * Update both flywheel and hood. Call once per loop.
     */
    fun update(dt: Duration) {
        updateFlywheel(io.flywheelVelocity(), dt)
        updateHood(dt)
    }

    // ========================================================================
    // Flywheel (PID + feedforward)
    // ========================================================================

    private fun updateFlywheel(curV: Double, dt: Duration) {
        if (dt <= Duration.ZERO) return

        val feedforward = flywheelTarget / maxVelocity
        val pidOutput = pid.update(curV, dt)
        val power = feedforward + pidOutput

        io.flywheel = (power / io.voltage() * 12.0).coerceIn(-1.0..1.0)
    }

    fun resetPID() {
        pid.reset()
    }

    // ========================================================================
    // Hood
    // ========================================================================

    private fun updateHood(dt: Duration) {
        val angle = if (autoAdjust) {
            computeHoodAngle(
                distance = targetDistance,
                flywheelVelocity = io.flywheelVelocity()
            )
        } else {
            manualHoodAngle
        }

        computedHoodAngle = angle
        hoodServoPosition = hoodAngleToServo(angle)
        io.hood = hoodServoPosition
    }

    /**
     * Compute the hood angle. Prefers saved tuning data (interpolated),
     * falls back to projectile motion trig.
     */
    fun computeHoodAngle(distance: Double, flywheelVelocity: Double): Double {
        val interpolatedDeg = recommendedHoodAngleDeg
        if (interpolatedDeg != null) {
            usingInterpolatedData = true
            return Math.toRadians(interpolatedDeg)
        }

        usingInterpolatedData = false
        return computeOptimalAngleTrig(distance, flywheelVelocity)
    }

    /**
     * Compute optimal launch angle using projectile motion.
     * theta = atan((v^2 - sqrt(v^4 - g(g*d^2 + 2h*v^2))) / (g*d))
     */
    fun computeOptimalAngleTrig(distance: Double, flywheelVelocity: Double): Double {
        val v = flywheelVelocity * ShooterConfig.flywheelRadius * ShooterConfig.launchEfficiency
        val d = distance
        val h = ShooterConfig.goalHeight - ShooterConfig.launchHeight
        val g = ShooterConfig.gravity

        if (v < 0.5 || d < 0.1) {
            return Math.toRadians(ShooterConfig.defaultAngleDeg)
        }

        val v2 = v * v
        val v4 = v2 * v2
        val discriminant = v4 - g * (g * d * d + 2.0 * h * v2)

        if (discriminant < 0.0) {
            return Math.toRadians(ShooterConfig.maxAngleDeg)
        }

        val numerator = v2 - sqrt(discriminant)
        val denominator = g * d
        val angle = atan2(numerator, denominator)

        return angle.coerceIn(
            Math.toRadians(ShooterConfig.minAngleDeg),
            Math.toRadians(ShooterConfig.maxAngleDeg)
        )
    }

    private fun hoodAngleToServo(angle: Double): Double {
        val minRad = Math.toRadians(ShooterConfig.minAngleDeg)
        val maxRad = Math.toRadians(ShooterConfig.maxAngleDeg)
        return ((angle - minRad) / (maxRad - minRad)).coerceIn(0.0, 1.0)
    }

    fun hoodServoToAngle(servo: Double): Double {
        val minRad = Math.toRadians(ShooterConfig.minAngleDeg)
        val maxRad = Math.toRadians(ShooterConfig.maxAngleDeg)
        return minRad + servo * (maxRad - minRad)
    }

    companion object {
        /** Calculate target RPM based on distance. */
        fun calculateTargetRPM(dist: Double): Double {
            // TODO: add these calculations
            return 5000.0
        }
    }
}

object ShooterConfig {
    @JvmField var goalHeight = 0.75
    @JvmField var launchHeight = 0.22
    @JvmField var gravity = 9.81
    @JvmField var flywheelRadius = 0.05
    @JvmField var launchEfficiency = 0.3
    @JvmField var minAngleDeg = 15.0
    @JvmField var maxAngleDeg = 70.0
    @JvmField var defaultAngleDeg = 45.0
}
