package sigmacorns.subsystem

import sigmacorns.io.SigmaIO
import sigmacorns.sim.LinearDcMotor
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.sqrt
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

/**
 * Unified flywheel + hood shooter subsystem.
 *
 * The flywheel and hood are a single mechanical unit: the flywheel spins up to
 * launch balls, and the hood servo controls the launch angle. Both take `SigmaIO`
 * and write their respective outputs each loop.
 *
 * Flywheel uses a 1st-order deadbeat controller to reach the target velocity
 * in one timestep. Hood computes the optimal launch angle from distance and
 * flywheel velocity, preferring interpolated tuning data over trig fallback.
 *
 * Input properties [targetDistance], [recommendedHoodAngleDeg], and [flywheelTarget]
 * are set by the AimingSystem each loop.
 */
class Shooter(
    val motor: LinearDcMotor,
    var inertia: Double,
    val io: SigmaIO,
    val lag: Duration = 0.seconds
) {

    // --- Flywheel state ---

    /** Target flywheel velocity in rad/s (set by AimingSystem or opmode). */
    var flywheelTarget: Double = 0.0

    /** Use simple hold mode instead of deadbeat controller. */
    var hold: Boolean = false

    private var lastU = 0.0

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
    // Flywheel
    // ========================================================================

    /*
    math:
        dv/dt = u tau_max/inertia (1-v/(u v_max))
        a = tau_max/inertia
        dv/dt = u a - a/v_max v
        v(t)= u v_max + (v_0 - u v_max)e^(-a/v_max t)

        -- need to find u such that v(DT) = v_target
        (v_target - v_0*e^(-a/v_max DT))/(v_max - v_max*e^(-a/v_max DT)) = u
     */

    private fun calculateFlywheelPower(v0: Double, dt: Duration): Double {
        val e = exp(-motor.stallTorque / inertia / motor.freeSpeed * dt.toDouble(DurationUnit.SECONDS))
        val power = (flywheelTarget - v0 * e) / (motor.freeSpeed * (1.0 - e))
        return power * motor.vRef
    }

    private fun updateFlywheel(curV: Double, dt: Duration) {
        if (dt <= Duration.ZERO) return

        val V = if (hold) {
            flywheelTarget / motor.freeSpeed * motor.vRef
        } else {
            val v0 = integrateFlywheelVelocity(curV, lastU, lag)
            calculateFlywheelPower(v0, dt)
        }

        io.flywheel = (V / io.voltage()).coerceIn(-1.0..1.0)
        lastU = V.coerceIn(-io.voltage()..io.voltage())
    }

    fun integrateFlywheelVelocity(v0: Double, u: Double, t: Duration): Double {
        val a = motor.stallTorque / inertia
        val s = u * motor.freeSpeed
        return s + (v0 - s) * exp(-a / motor.freeSpeed * t.toDouble(DurationUnit.SECONDS))
    }

    fun spinupTime(cur: Double, target: Double, dt: Duration): Duration {
        /*
        If the target cannot be reached in DT, this controller will saturate motor limits
        therefore an upper bound for the spinup time is the time to reach target under full power + DT

        v_target = v_max + (v_0 - v_max)e^(-a/v_max t)
        ln((v_target - v_max)/(v_0 - v_max))/(-a/v_max) = t
        */
        val t = ln((target - motor.freeSpeed) / (cur - motor.freeSpeed)) / (-motor.stallTorque / inertia / motor.freeSpeed)
        return t.seconds + dt + lag
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
