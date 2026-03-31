package sigmacorns.subsystem

import sigmacorns.constants.FLYWHEEL_INERTIA
import sigmacorns.io.SigmaIO
import kotlin.math.PI
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
     * @param batteryVoltage current battery voltage (V)
     * @param targetVelocity desired flywheel angular velocity (rad/s)
     * @param currentVelocity measured flywheel angular velocity (rad/s)
     * @param dt time step (s)
     * @return motor power in [-1, 1]
     */
    private fun calculateFlywheelSpeed(
        batteryVoltage: Double,
        targetVelocity: Double,
        currentVelocity: Double,
        dt: Double,
    ): Double {
        if (batteryVoltage <= 0.0) return 0.0

        val stateCost = 0.5
        val inputCost = 0.5

        val inertia = FLYWHEEL_INERTIA
        val referenceVoltage = 12.0
        val stallTorque = 1.47 * 0.0980665 // 1.47 kg·cm -> N·m
        val freeSpeed = 5000.0 / 60 * 2 * PI // 5000 RPM -> rad/s

        // Continuous-time state-space: dω/dt = stateCoeff*ω + inputCoeff*V
        val stateCoeff = -stallTorque / (freeSpeed * referenceVoltage * inertia)
        val inputCoeff = stallTorque / (inertia * referenceVoltage)

        // Zero-order hold discretization
        val discreteState = exp(stateCoeff * dt)
        val discreteInput = (inputCoeff * (discreteState - 1)) / stateCoeff

        // Solve DARE for steady-state cost P
        val riccatiLinear = stateCost * (1 - discreteState * discreteState) - inputCost * discreteInput * discreteInput
        val riccatiCost = (-riccatiLinear + sqrt(
            riccatiLinear * riccatiLinear + 4 * discreteInput * discreteInput * inputCost * stateCost
        )) / (2 * discreteInput * discreteInput)

        // Optimal LQR gain
        val gain = (discreteState * discreteInput * riccatiCost) /
            (inputCost + discreteInput * discreteInput * riccatiCost)

        val voltage = -gain * (currentVelocity - targetVelocity)
        return (voltage / batteryVoltage).coerceIn(-1.0..1.0)
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
