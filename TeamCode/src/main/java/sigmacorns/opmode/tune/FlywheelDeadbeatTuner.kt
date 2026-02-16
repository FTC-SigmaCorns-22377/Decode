package sigmacorns.opmode.tune

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.constants.flywheelMotor
import sigmacorns.control.subsystem.Flywheel
import sigmacorns.opmode.SigmaOpMode
import kotlin.time.Duration.Companion.milliseconds

/**
 * Configurable parameters for the flywheel deadbeat controller.
 * Adjust these in the Panels dashboard to tune the controller.
 */
@Configurable
object FlywheelDeadbeatConfig {
    /** Flywheel inertia in kg*m^2 */
    @JvmField var inertia = 0.2 // recalculte this bc the flywheels add 0.8 kg of weight

    /** Expected sensor-to-actuator lag in milliseconds */
    @JvmField var lagMs = 10.0

    /** Whether to use hold mode (simple feedforward) instead of deadbeat */
    @JvmField var holdMode = false

    /** Step size for target velocity (rad/s) */
    @JvmField var stepSize = 500.0
}

/**
 * Tuning OpMode for the Flywheel deadbeat controller.
 *
 * Uses Panels dashboard to graph flywheel velocity vs target and adjust parameters.
 *
 * Controls:
 * - D-pad up/down: Increase/decrease step size by 10 rad/s
 * - A button: Toggle target between 0 and step size
 * - Right stick Y: Continuous manual target adjustment
 * - B button: Reset target to 0
 * - X button: Toggle hold mode
 */
@TeleOp(name = "Flywheel Deadbeat Tuner", group = "Tune")
class FlywheelDeadbeatTuner : SigmaOpMode() {

    private lateinit var flywheel: Flywheel

    // Target state
    private var target = 0.0
    private var stepTarget = 0.0  // The step value (0 or stepSize)
    private var isAtStep = false  // Whether we're at step or 0

    // Continuous adjustment rate (rad/s per second of full stick deflection)
    private val continuousRate = 50.0

    override fun runOpMode() {
        // Create the flywheel controller
        flywheel = Flywheel(
            motor = flywheelMotor,
            inertia = FlywheelDeadbeatConfig.inertia,
            io = io,
            lag = FlywheelDeadbeatConfig.lagMs.milliseconds
        )

        telemetry.addLine("Flywheel Deadbeat Tuner")
        telemetry.addLine("Use Panels dashboard to adjust inertia and lag")
        telemetry.addLine("Controls:")
        telemetry.addLine("  D-pad up/down: Adjust step size")
        telemetry.addLine("  A: Toggle target (0 <-> step)")
        telemetry.addLine("  Right stick Y: Manual adjustment")
        telemetry.addLine("  B: Reset target to 0")
        telemetry.addLine("  X: Toggle hold mode")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Update flywheel controller parameters from configurables
            flywheel = Flywheel(
                motor = flywheelMotor,
                inertia = FlywheelDeadbeatConfig.inertia,
                io = io,
                lag = FlywheelDeadbeatConfig.lagMs.milliseconds
            )
            flywheel.hold = FlywheelDeadbeatConfig.holdMode

            // D-pad: adjust step size
            if (gamepad1.dpad_up) {
                FlywheelDeadbeatConfig.stepSize += 10.0
                while (gamepad1.dpad_up && opModeIsActive()) { idle() }
            }
            if (gamepad1.dpad_down) {
                FlywheelDeadbeatConfig.stepSize = (FlywheelDeadbeatConfig.stepSize - 10.0).coerceAtLeast(0.0)
                while (gamepad1.dpad_down && opModeIsActive()) { idle() }
            }

            // A button: toggle between 0 and step size
            if (gamepad1.a) {
                isAtStep = !isAtStep
                stepTarget = if (isAtStep) FlywheelDeadbeatConfig.stepSize else 0.0
                target = stepTarget
                while (gamepad1.a && opModeIsActive()) { idle() }
            }

            // B button: reset to 0
            if (gamepad1.b) {
                target = 0.0
                stepTarget = 0.0
                isAtStep = false
                while (gamepad1.b && opModeIsActive()) { idle() }
            }

            // X button: toggle hold mode
            if (gamepad1.x) {
                FlywheelDeadbeatConfig.holdMode = !FlywheelDeadbeatConfig.holdMode
                while (gamepad1.x && opModeIsActive()) { idle() }
            }

            // Right stick Y: continuous manual adjustment
            target += -gamepad1.right_stick_y * continuousRate * dt.inWholeMilliseconds / 1000.0

            // Update flywheel controller
            flywheel.target = target
            val currentVelocity = io.flywheelVelocity()
            flywheel.update(currentVelocity, dt)

            // Calculate error
            val error = target - currentVelocity

            // Telemetry for Panels
            val tel = PanelsTelemetry.telemetry

            tel.addLine("=== Flywheel Deadbeat Tuner ===")
            tel.addLine("")

            // These will be graphed in Panels
            tel.addData("Target (rad/s)", target)
            tel.addData("Velocity (rad/s)", currentVelocity)
            tel.addData("Error (rad/s)", error)
            tel.addData("Motor Power", io.shooter)
            tel.addLine("")

            tel.addLine("--- Config (adjust in Panels) ---")
            tel.addData("Inertia (kg*m^2)", FlywheelDeadbeatConfig.inertia)
            tel.addData("Lag (ms)", FlywheelDeadbeatConfig.lagMs)
            tel.addData("Hold Mode", FlywheelDeadbeatConfig.holdMode)
            tel.addData("Step Size (rad/s)", FlywheelDeadbeatConfig.stepSize)
            tel.addLine("")

            tel.addLine("--- State ---")
            tel.addData("Step Target", stepTarget)
            tel.addData("Is At Step", isAtStep)
            tel.addLine("")

            tel.addLine("--- Controls ---")
            tel.addLine("  D-pad up/down: Step size +/- 10")
            tel.addLine("  A: Toggle 0 <-> step")
            tel.addLine("  Right stick Y: Manual adjust")
            tel.addLine("  B: Reset to 0")
            tel.addLine("  X: Toggle hold mode")

            tel.update()

            false // continue loop
        }
    }
}
