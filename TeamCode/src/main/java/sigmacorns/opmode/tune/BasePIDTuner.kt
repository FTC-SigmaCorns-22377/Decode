package sigmacorns.opmode.tune

import com.bylazar.telemetry.PanelsTelemetry
import sigmacorns.control.PIDController
import sigmacorns.control.SlewRateLimiter
import sigmacorns.opmode.SigmaOpMode

/**
 * Base class for PID tuning opmodes. Provides common structure for
 * tuning position or velocity PID controllers.
 */
abstract class BasePIDTuner : SigmaOpMode() {

    /** Name displayed in telemetry (e.g., "Spindexer", "Turret", "Flywheel") */
    abstract val tunerName: String

    /** Unit label for telemetry (e.g., "rad", "rad/s") */
    abstract val unitLabel: String

    /** Whether to show degree conversion in telemetry (useful for position, not velocity) */
    open val showDegrees: Boolean = true

    /** Step size for dpad discrete adjustments */
    abstract val discreteStep: Double

    /** Continuous adjustment rate per second when using stick */
    open val continuousRate: Double = 2.0

    /** Optional slew rate limiter for motor output. Override to enable. */
    open val slewRateLimiter: SlewRateLimiter? = null

    /** Optional slew rate limiter for motor output. Override to enable. */
    open val outputSlewRateLimiter: SlewRateLimiter? = null

    /** Get the current slew rate. Override to make it configurable. */
    open fun getSlewRate(): Double = slewRateLimiter?.maxRate ?: 0.0

    /** Get the current output slew rate. Override to make it configurable. */
    open fun getOutputSlewRate(): Double = outputSlewRateLimiter?.maxRate ?: 0.0

    /** Get the current PID gains */
    abstract fun getKp(): Double
    abstract fun getKd(): Double
    abstract fun getKi(): Double

    /** Get feedforward power based on target value. Override for velocity controllers. */
    open fun getFeedforward(target: Double): Double = 0.0

    /** Read the current measured value (position or velocity) */
    abstract fun readCurrentValue(): Double

    /** Apply motor power output */
    abstract fun applyMotorPower(power: Double)

    override fun runOpMode() {
        var targetValue = 0.0

        val pid = PIDController(
            kp = getKp(),
            kd = getKd(),
            ki = getKi(),
            setpoint = targetValue
        )

        telemetry.addLine("$tunerName PID Tuner")
        telemetry.addLine("Use Panels dashboard to adjust kP, kD, kI")
        telemetry.addLine("Use gamepad to control target")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Update PID gains from configurables
            pid.kp = getKp()
            pid.kd = getKd()
            pid.ki = getKi()

            // Update slew rate from configurables
            slewRateLimiter?.maxRate = getSlewRate()
            outputSlewRateLimiter?.maxRate = getOutputSlewRate()

            // Continuous adjustment with right stick Y
            targetValue += -gamepad1.right_stick_y * continuousRate * dt.inWholeMilliseconds / 1000.0

            // Discrete steps with dpad
            if (gamepad1.dpad_up) {
                targetValue += discreteStep
                while (gamepad1.dpad_up && opModeIsActive()) { idle() }
            }
            if (gamepad1.dpad_down) {
                targetValue -= discreteStep
                while (gamepad1.dpad_down && opModeIsActive()) { idle() }
            }

            // Reset with A button
            if (gamepad1.a) {
                targetValue = 0.0
                pid.reset()
                slewRateLimiter?.reset()
                outputSlewRateLimiter?.reset()
                while (gamepad1.a && opModeIsActive()) { idle() }
            }

            // Get current value
            val currentValue = readCurrentValue()

            val slewLimitedTarget = slewRateLimiter?.calculate(targetValue,dt) ?: targetValue

            // Update PID and compute output
            pid.setpoint = slewLimitedTarget
            val feedforward = getFeedforward(slewLimitedTarget)
            val rawMotorPower = (pid.update(currentValue, dt) + feedforward).coerceIn(-1.0, 1.0)
            val limitedMotorPower = outputSlewRateLimiter?.calculate(rawMotorPower, dt) ?: rawMotorPower

            // Apply motor power
            applyMotorPower(limitedMotorPower)

            // Calculate error
            val error = targetValue - currentValue

            val telemetry = PanelsTelemetry.telemetry

            // Telemetry
            telemetry.addLine("=== $tunerName PID Tuner ===")
            telemetry.addLine("")
            telemetry.addData("kP", getKp())
            telemetry.addData("kD", getKd())
            telemetry.addData("kI", getKi())
            telemetry.addLine("")
            telemetry.addData("Target ($unitLabel)", targetValue)
            if (showDegrees) {
                telemetry.addData("Target (deg)", Math.toDegrees(targetValue))
            }
            telemetry.addData("Current ($unitLabel)", currentValue)
            if (showDegrees) {
                telemetry.addData("Current (deg)", Math.toDegrees(currentValue))
            }
            telemetry.addLine("")
            telemetry.addData("Error ($unitLabel)", error)
            if (showDegrees) {
                telemetry.addData("Error (deg)", Math.toDegrees(error))
            }
            telemetry.addData("Motor Power", rawMotorPower)
            if (feedforward != 0.0) {
                telemetry.addData("Feedforward", feedforward)
            }
            if (slewRateLimiter != null) {
                telemetry.addData("Raw Power", rawMotorPower)
                telemetry.addData("Slew Rate", slewRateLimiter!!.maxRate)
            }
            if (outputSlewRateLimiter != null) {
                telemetry.addData("Output Slew Rate", outputSlewRateLimiter!!.maxRate)
            }
            telemetry.addLine("")
            telemetry.addLine("Controls:")
            telemetry.addLine("  Right Stick Y: Adjust target")
            telemetry.addLine("  D-Pad Up/Down: Step +/- ${formatStep()}")
            telemetry.addLine("  A: Reset to 0")
            telemetry.update()

            false // continue loop
        }
    }

    private fun formatStep(): String {
        return if (showDegrees) {
            "%.0f°".format(Math.toDegrees(discreteStep))
        } else {
            "%.1f $unitLabel".format(discreteStep)
        }
    }
}
