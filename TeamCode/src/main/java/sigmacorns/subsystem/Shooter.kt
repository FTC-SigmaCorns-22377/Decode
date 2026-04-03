package sigmacorns.subsystem

import sigmacorns.constants.FLYWHEEL_INERTIA
import sigmacorns.constants.flywheelMotor
import sigmacorns.io.SigmaIO
import kotlin.math.PI
import kotlin.math.absoluteValue
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
 * Hood angle is set directly by AimingSystem via [hoodAngle] (solver output),
 * or manually via [manualHoodAngle] when [autoAdjust] is false.
 *
 * Input properties [hoodAngle] and [flywheelTarget] are set by AimingSystem each loop.
 */
class Shooter(
    val io: SigmaIO
) {

    // --- Flywheel state ---

    /** Target flywheel velocity in rad/s (set by AimingSystem or opmode). */
    var flywheelTarget: Double = 0.0

    // --- Hood state ---

    /** Whether the hood uses the solver-set [hoodAngle] (true) or [manualHoodAngle] (false). */
    var autoAdjust: Boolean = true

    /** Manual override angle in radians (used when autoAdjust is false). */
    var manualHoodAngle: Double = Math.toRadians(ShooterConfig.defaultAngleDeg)

    /** Hood angle in radians set directly by AimingSystem from ShotSolver output. */
    var hoodAngle: Double = Math.toRadians(ShooterConfig.defaultAngleDeg)

    /** The commanded hood angle in radians (for telemetry). */
    var computedHoodAngle: Double = Math.toRadians(ShooterConfig.defaultAngleDeg)
        private set

    /** Current hood servo position (for telemetry). */
    var hoodServoPosition: Double = 0.5
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
        val stallTorque = flywheelMotor.stallTorque
        val freeSpeed = flywheelMotor.freeSpeed

        val encoderTickPerRad = 28.0/(2.0*PI)

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

        val velResolution = encoderTickPerRad * currentVelocity*currentVelocity * dTimer

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
        val angle = if (autoAdjust) hoodAngle else manualHoodAngle
        computedHoodAngle = angle
        hoodServoPosition = hoodAngleToServo(angle)
        io.hood = hoodServoPosition
    }

    private fun hoodAngleToServo(angle: Double): Double {
        val minRad = Math.toRadians(ShooterConfig.minAngleDeg)
        val maxRad = Math.toRadians(ShooterConfig.maxAngleDeg)
        val t = ((angle - minRad) / (maxRad - minRad)).coerceIn(0.0, 1.0)
        return ShooterConfig.minServo + t * (ShooterConfig.maxServo - ShooterConfig.minServo)
    }

    fun hoodServoToAngle(servo: Double): Double {
        val minRad = Math.toRadians(ShooterConfig.minAngleDeg)
        val maxRad = Math.toRadians(ShooterConfig.maxAngleDeg)
        val t = (servo - ShooterConfig.minServo) / (ShooterConfig.maxServo - ShooterConfig.minServo)
        return minRad + t * (maxRad - minRad)
    }

}

object ShooterConfig {
    @JvmField var minAngleDeg = 28.441752
    @JvmField var maxAngleDeg = 70.0
    @JvmField var defaultAngleDeg = 45.0
    @JvmField var minServo = 0.29
    @JvmField var maxServo = 1.0
}
