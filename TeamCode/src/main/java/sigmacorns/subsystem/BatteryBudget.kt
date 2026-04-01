package sigmacorns.subsystem

import org.joml.*
import sigmacorns.constants.bareMotorTopSpeed
import sigmacorns.constants.driveGearRatio
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.HardwareIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumDynamics
import sigmacorns.sim.MecanumState
import kotlin.math.abs
import kotlin.math.exp

/**
 * Tracks battery state and available power budget on real hardware.
 *
 * Only instantiated when running on a physical robot (HardwareIO), since
 * simulation environments provide ideal voltage and don't model battery sag.
 *
 * Uses the hub's reported voltage reading directly as the battery voltage for
 * budgeting, combined with a current draw model for each motor to estimate
 * total system current and enforce a current limit.
 *
 * Current draw model for a DC motor:
 *   I = I_stall * (V_motor / V_ref) - I_stall * (ω / ω_free)
 * where ω is the bare motor velocity (before gearbox).
 */
class BatteryBudget(val io: HardwareIO) {

    private val dynamics = MecanumDynamics(drivetrainParameters)

    /** Maximum configurable allowable current draw in amps. */
    val MAX_CURRENT: Double = 18.0

    // --- Voltage estimates (updated each loop) ---

    /** Raw hub voltage reading (V_hub, includes sag from internal resistance). */
    var hubVoltage: Double = 0.0
        private set

    // --- Motor voltages (updated each loop, using estimated V_batt) ---

    var driveFLVolts: Double = 0.0
        private set
    var driveFRVolts: Double = 0.0
        private set
    var driveBLVolts: Double = 0.0
        private set
    var driveBRVolts: Double = 0.0
        private set
    var flywheel1Volts: Double = 0.0
        private set
    var flywheel2Volts: Double = 0.0
        private set
    var intake1Volts: Double = 0.0
        private set
    var intake2Volts: Double = 0.0
        private set

    // --- Motor currents in amps (updated each loop) ---

    var driveFLCurrent: Double = 0.0
        private set
    var driveFRCurrent: Double = 0.0
        private set
    var driveBLCurrent: Double = 0.0
        private set
    var driveBRCurrent: Double = 0.0
        private set
    var flywheel1Current: Double = 0.0
        private set
    var flywheel2Current: Double = 0.0
        private set
    var intake1Current: Double = 0.0
        private set
    var intake2Current: Double = 0.0
        private set
    var totalCurrent: Double = 0.0
        private set

    fun update() {
        hubVoltage = io.voltage()

        // Motor voltages using estimated true battery voltage
        driveFLVolts = io.driveFL * hubVoltage
        driveFRVolts = io.driveFR * hubVoltage
        driveBLVolts = io.driveBL * hubVoltage
        driveBRVolts = io.driveBR * hubVoltage
        flywheel1Volts = io.flywheel * hubVoltage
        flywheel2Volts = io.flywheel * hubVoltage
        intake1Volts = io.intake * hubVoltage
        intake2Volts = io.intake * hubVoltage

        // Motor currents using corrected voltages
        driveFLCurrent = motorCurrent(io.driveFL, driveFLVolts, io.cachedDriveFLVelocity)
        driveFRCurrent = motorCurrent(io.driveFR, driveFRVolts, io.cachedDriveFRVelocity)
        driveBLCurrent = motorCurrent(io.driveBL, driveBLVolts, io.cachedDriveBLVelocity)
        driveBRCurrent = motorCurrent(io.driveBR, driveBRVolts, io.cachedDriveBRVelocity)
        flywheel1Current = motorCurrent(io.flywheel, flywheel1Volts, io.flywheelVelocity())
        flywheel2Current = motorCurrent(io.flywheel, flywheel2Volts, io.cachedFlywheel2Velocity)
        intake1Current = motorCurrent(io.intake, intake1Volts, io.cachedIntake1Velocity)
        intake2Current = motorCurrent(io.intake, intake2Volts, io.cachedIntake2Velocity)

        totalCurrent = driveFLCurrent + driveFRCurrent + driveBLCurrent + driveBRCurrent +
            flywheel1Current + flywheel2Current + intake1Current + intake2Current
    }

    /**
     * Predict the circuit state after applying new motor powers.
     *
     * Uses the hub voltage and forward-predicts motor velocities by one timestep
     * using a first-order motor response model. This prevents over-estimating
     * current for motors accelerating from standstill. When power is zero,
     * current is zero (H-bridge not driving).
     *
     * Any motor power left as `null` uses the current commanded value from IO.
     *
     * @param dt prediction time horizon in seconds (default: one 50Hz loop)
     * @return predicted circuit state snapshot
     */
    fun predictState(
        driveFLPower: Double? = null,
        driveFRPower: Double? = null,
        driveBLPower: Double? = null,
        driveBRPower: Double? = null,
        flywheelPower: Double? = null,
        intakePower: Double? = null,
        dt: Double = DEFAULT_PREDICT_DT,
    ): PredictedState {
        val pFL = driveFLPower ?: io.driveFL
        val pFR = driveFRPower ?: io.driveFR
        val pBL = driveBLPower ?: io.driveBL
        val pBR = driveBRPower ?: io.driveBR
        val pFlywheel = flywheelPower ?: io.flywheel
        val pIntake = intakePower ?: io.intake

        // Per-motor voltages
        val flVolts = pFL * hubVoltage
        val frVolts = pFR * hubVoltage
        val blVolts = pBL * hubVoltage
        val brVolts = pBR * hubVoltage
        val fw1Volts = pFlywheel * hubVoltage
        val fw2Volts = pFlywheel * hubVoltage
        val in1Volts = pIntake * hubVoltage
        val in2Volts = pIntake * hubVoltage

        // Forward-predict velocities after one timestep
        val omegaFL = predictVelocity(io.cachedDriveFLVelocity, flVolts, dt)
        val omegaFR = predictVelocity(io.cachedDriveFRVelocity, frVolts, dt)
        val omegaBL = predictVelocity(io.cachedDriveBLVelocity, blVolts, dt)
        val omegaBR = predictVelocity(io.cachedDriveBRVelocity, brVolts, dt)
        val omegaFw1 = predictVelocity(io.flywheelVelocity(), fw1Volts, dt)
        val omegaFw2 = predictVelocity(io.cachedFlywheel2Velocity, fw2Volts, dt)
        val omegaIn1 = predictVelocity(io.cachedIntake1Velocity, in1Volts, dt)
        val omegaIn2 = predictVelocity(io.cachedIntake2Velocity, in2Volts, dt)

        // Per-motor currents with predicted velocities
        val flCurrent = motorCurrent(pFL, flVolts, omegaFL)
        val frCurrent = motorCurrent(pFR, frVolts, omegaFR)
        val blCurrent = motorCurrent(pBL, blVolts, omegaBL)
        val brCurrent = motorCurrent(pBR, brVolts, omegaBR)
        val fw1Current = motorCurrent(pFlywheel, fw1Volts, omegaFw1)
        val fw2Current = motorCurrent(pFlywheel, fw2Volts, omegaFw2)
        val in1Current = motorCurrent(pIntake, in1Volts, omegaIn1)
        val in2Current = motorCurrent(pIntake, in2Volts, omegaIn2)

        val total = flCurrent + frCurrent + blCurrent + brCurrent +
            fw1Current + fw2Current + in1Current + in2Current

        return PredictedState(
            totalCurrent = total,
            driveFLCurrent = flCurrent,
            driveFRCurrent = frCurrent,
            driveBLCurrent = blCurrent,
            driveBRCurrent = brCurrent,
            flywheel1Current = fw1Current,
            flywheel2Current = fw2Current,
            intake1Current = in1Current,
            intake2Current = in2Current,
        )
    }

    /**
     * Request drivetrain motor powers with current-limited protection.
     *
     * Uses the full mecanum dynamics model to simulate the robot forward 0.2s
     * under the requested powers, accounting for robot mass, wheel kinematics,
     * viscous drag, and motor back-EMF. Computes predicted current draw at the
     * end of the simulation window.
     *
     * If predicted total current (drive + other motors) is under [MAX_CURRENT],
     * powers are applied as-is. Otherwise, drive powers are scaled down uniformly.
     *
     * @param driveFLPower front-left wheel power [-1, 1]
     * @param driveBLPower back-left wheel power [-1, 1]
     * @param driveBRPower back-right wheel power [-1, 1]
     * @param driveFRPower front-right wheel power [-1, 1]
     * @return the scale factor applied (1.0 = no scaling, <1.0 = throttled)
     */
    fun requestDrivetrainPower(
        driveFLPower: Double,
        driveBLPower: Double,
        driveBRPower: Double,
        driveFRPower: Double,
    ): Double {
        val nonDriveCurrent = flywheel1Current + flywheel2Current + intake1Current + intake2Current
        val currentBudget = MAX_CURRENT - nonDriveCurrent

        if (currentBudget <= 0.0) {
            io.driveFL = 0.0
            io.driveBL = 0.0
            io.driveBR = 0.0
            io.driveFR = 0.0
            return 0.0
        }

        val fullCurrent = predictDrivetrainCurrent(driveFLPower, driveBLPower, driveBRPower, driveFRPower)
        if (fullCurrent <= currentBudget) {
            io.driveFL = driveFLPower
            io.driveBL = driveBLPower
            io.driveBR = driveBRPower
            io.driveFR = driveFRPower
            return 1.0
        }

        // Binary search for scale factor (10 iterations -> ~0.1% precision)
        var lo = 0.0
        var hi = 1.0
        repeat(10) {
            val mid = (lo + hi) / 2.0
            val current = predictDrivetrainCurrent(
                driveFLPower * mid, driveBLPower * mid,
                driveBRPower * mid, driveFRPower * mid,
            )
            if (current <= currentBudget) lo = mid else hi = mid
        }

        io.driveFL = driveFLPower * lo
        io.driveBL = driveBLPower * lo
        io.driveBR = driveBRPower * lo
        io.driveFR = driveFRPower * lo
        return lo
    }

    /**
     * Simulate the drivetrain forward [PREDICT_HORIZON]s and compute the total
     * drive motor current draw at the end of the window.
     */
    private fun predictDrivetrainCurrent(
        pFL: Double, pBL: Double, pBR: Double, pFR: Double,
    ): Double {
        val currentState = MecanumState(
            vel = io.velocity(),
            pos = io.position(),
        )

        val powers = doubleArrayOf(pFL, pBL, pBR, pFR)
        val predicted = dynamics.integrate(PREDICT_HORIZON, PREDICT_DT, powers, currentState)

        // Rotate predicted field velocity into robot frame to get wheel velocities
        val robotVel = Pose2d(predicted.vel.v.x, predicted.vel.v.y, predicted.vel.rot)
        robotVel.v = Matrix2d().rotate(-predicted.pos.rot) * robotVel.v
        val wheelVels = dynamics.mecanumInverseVelKinematics(robotVel)

        // Convert geared wheel velocities to bare motor velocities
        val bareFL = wheelVels.x * driveGearRatio
        val bareBL = wheelVels.y * driveGearRatio
        val bareBR = wheelVels.z * driveGearRatio
        val bareFR = wheelVels.w * driveGearRatio

        val vHub = hubVoltage
        return motorCurrent(pFL, pFL * vHub, bareFL) +
            motorCurrent(pBL, pBL * vHub, bareBL) +
            motorCurrent(pBR, pBR * vHub, bareBR) +
            motorCurrent(pFR, pFR * vHub, bareFR)
    }

    companion object {
        /** Default prediction horizon: one loop at 50Hz. */
        const val DEFAULT_PREDICT_DT = 0.02

        /** Drivetrain simulation horizon in seconds. */
        const val PREDICT_HORIZON = 0.2

        /** Drivetrain simulation timestep in seconds. */
        const val PREDICT_DT = 0.005

        /** RS-555 bare motor stall current at 12V. */
        const val STALL_CURRENT = 9.2

        /** RS-555 bare motor reference voltage. */
        const val REFERENCE_VOLTAGE = 12.0

        /**
         * Electrical time constant of the motor+load system in seconds.
         * Used to predict velocity after one timestep via first-order response.
         * Conservative estimate — real system may respond faster.
         */
        const val MOTOR_TIME_CONSTANT = 0.05

        /**
         * Compute current draw from the battery for a single DC motor.
         *
         * When power is zero, the H-bridge is not driving the motor and no
         * current flows from the battery (coasting or brake mode dissipates
         * through the motor windings, not the battery).
         *
         * I = I_stall * (V_motor / V_ref) - I_stall * (ω / ω_free)
         */
        fun motorCurrent(power: Double, motorVoltage: Double, bareMotorVelocity: Double): Double {
            if (power == 0.0) return 0.0
            return abs(STALL_CURRENT * (motorVoltage / REFERENCE_VOLTAGE) -
                STALL_CURRENT * (bareMotorVelocity / bareMotorTopSpeed))
        }

        /**
         * Predict motor velocity after one timestep using a first-order response.
         *
         * The motor approaches its steady-state velocity `ω_ss = ω_free * (V/V_ref)`
         * exponentially with time constant [MOTOR_TIME_CONSTANT]:
         *   ω_predicted = ω_ss + (ω_current - ω_ss) * exp(-dt / τ)
         *
         * This avoids needing per-motor load inertia while giving a reasonable
         * estimate of how quickly current drops after initial acceleration.
         */
        fun predictVelocity(
            currentVelocity: Double,
            motorVoltage: Double,
            dt: Double,
        ): Double {
            val steadyState = bareMotorTopSpeed * (motorVoltage / REFERENCE_VOLTAGE)
            return steadyState + (currentVelocity - steadyState) * exp(-dt / MOTOR_TIME_CONSTANT)
        }
    }
}

/**
 * Predicted circuit state snapshot from [BatteryBudget.predictState].
 */
data class PredictedState(
    val totalCurrent: Double,
    val driveFLCurrent: Double,
    val driveFRCurrent: Double,
    val driveBLCurrent: Double,
    val driveBRCurrent: Double,
    val flywheel1Current: Double,
    val flywheel2Current: Double,
    val intake1Current: Double,
    val intake2Current: Double,
)
