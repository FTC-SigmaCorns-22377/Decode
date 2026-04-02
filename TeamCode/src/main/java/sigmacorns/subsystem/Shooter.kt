package sigmacorns.subsystem

import sigmacorns.constants.FLYWHEEL_INERTIA
import sigmacorns.io.SigmaIO
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.sqrt
import kotlin.time.Duration
import kotlin.time.DurationUnit

/**
 * Unified flywheel + hood shooter subsystem.
 *
 * The flywheel and hood are a single mechanical unit: the flywheel spins up to
 * launch balls, and the hood servo controls the launch angle. Both take `SigmaIO`
 * and write their respective outputs each loop.
 *
 * Flywheel uses a discrete-time LQR controller to reach the target velocity.
 * Hood computes the optimal launch angle from distance and flywheel velocity,
 * preferring interpolated tuning data over trig fallback.
 *
 * Input properties [targetDistance], [recommendedHoodAngleDeg], and [flywheelTarget]
 * are set by the AimingSystem each loop.
 */
class Shooter(
    val io: SigmaIO
) {

    // --- Flywheel state ---

    /** Target flywheel velocity in rad/s (set by AimingSystem or opmode). */
    var flywheelTarget: Double = 0.0

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
    // Flywheel (LQR)
    // ========================================================================

    private fun updateFlywheel(curV: Double, dt: Duration) {
        if (dt <= Duration.ZERO) return

        if (flywheelTarget == 0.0) {
            io.flywheel = 0.0
            return
        }

        io.flywheel = calculateFlywheelSpeed(
            io.voltage(),
            flywheelTarget,
            curV,
            dt.toDouble(DurationUnit.SECONDS)
        )
    }

    /**
     * Compute flywheel motor power using a discrete-time LQR controller.
     *
     * Models the DC motor as: J*dω/dt = (T_stall/V_ref)*V - (T_stall/(V_ref*ω_free))*ω
     * Discretizes the continuous dynamics with a zero-order hold, then solves the
     * discrete algebraic Riccati equation (DARE) for the optimal gain K.
     *
     * @param hubVoltage current battery voltage (V)
     * @param targetVelocity desired flywheel angular velocity (rad/s)
     * @param currentVelocity measured flywheel angular velocity (rad/s)
     * @param dt time step (s)
     * @return motor power in [-1, 1]
     */
    private fun calculateFlywheelSpeed(
        hubVoltage: Double,
        targetVelocity: Double,
        currentVelocity: Double,
        dt: Double,
    ): Double {
        if (hubVoltage <= 0.0) return 0.0

        val q = 1.0 // q
        val r = 120.0 // r

        val inertia = FLYWHEEL_INERTIA
        val referenceVoltage = 12.0
        val stallTorque = 1.47 * 0.0980665 // 1.47 kg·cm -> N·m
        val freeSpeed = 5650.0 * (12/12.41) / 60 * 2 * PI // rad/s

        // encoder position resolution
        val encoderRes = 2*PI/28.0

        /**
         * Chub uses period measurement:
         *
         * omega_measured = 2pi/28 / T_measured
         *
         * where T_measured is the measured time in between ticks
         *
         * Assume a timer with resolution dt is used. Let T be the true inter-tick time
         *
         * T = 2pi/28/omega
         * omega_measured = 2pi/28 / (T +- δt)
         *
         * angular resolution (from differentiating):
         *
         * dw=2pi/28 * dt/T^2
         *
         * substituting T= 2pi/28/omega
         *
         * dw = 28 * w^2 * dt / 2pi
         *
         * when testing encoder switched between 399 rad/s and 403 rad/s, so at omega = 401 rad/s dw=4 rad/s
         *
         * dt = dw * 2pi / (28* w^2)
         *
         * dt = 4 * 2pi / (28 * (401)^2)
         *
         * dt = 5.58 * 10^-6 -> means timer is running at ~180 Mhz
         */

        val dTimer = 0.00000558204

        val velResolution = (28.0 * currentVelocity*currentVelocity * dTimer)/ (2.0*PI)

        var xErr = currentVelocity - targetVelocity

        if ((currentVelocity-targetVelocity).absoluteValue < velResolution) {
            // deadzone at encoder resolution to prevent chatter
            xErr = 0.0
        }

        // Continuous-time state-space: dω/dt = stateCoeff*ω + inputCoeff*V
        val stateCoeff = -stallTorque / (freeSpeed * referenceVoltage * inertia)
        val inputCoeff = stallTorque / (inertia * referenceVoltage)

        // Zero-order hold discretization
        val discreteState = exp(stateCoeff * dt)
        val discreteInput = (inputCoeff * (discreteState - 1)) / stateCoeff

        // Solve DARE for steady-state cost P
        val riccatiLinear = q * (1 - discreteState * discreteState) - r * discreteInput * discreteInput
        val riccatiCost = (-riccatiLinear + sqrt(
            riccatiLinear * riccatiLinear + 4 * discreteInput * discreteInput * r * q
        )) / (2 * discreteInput * discreteInput)

        // Optimal LQR gain
        val gain = (discreteState * discreteInput * riccatiCost) /
            (r + discreteInput * discreteInput * riccatiCost)

        val feedforward = referenceVoltage * targetVelocity / freeSpeed
        val voltage = -gain * xErr + feedforward
        return (voltage / hubVoltage).coerceIn(-1.0..1.0)
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
