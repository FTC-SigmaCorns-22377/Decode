package sigmacorns.opmode.tune

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.constants.FlywheelPIDConstants
import sigmacorns.subsystem.Shooter
import sigmacorns.opmode.SigmaOpMode

/**
 * Configurable parameters for the flywheel deadbeat controller.
 * Adjust these in the Panels dashboard to tune the controller.
 */
@Configurable
object FlywheelDeadbeatConfig {
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

    private lateinit var shooter: Shooter

    // Target state
    private var target = 0.0
    private var stepTarget = 0.0  // The step value (0 or stepSize)
    private var isAtStep = false  // Whether we're at step or 0

    // Continuous adjustment rate (rad/s per second of full stick deflection)
    private val continuousRate = 50.0

    override fun runOpMode() {
        // Create the flywheel controller
        shooter = Shooter(
            FlywheelPIDConstants.kP,
            FlywheelPIDConstants.kD,
            FlywheelPIDConstants.kI,
            FlywheelPIDConstants.maxVelocity,
            io
        )

        telemetry.addLine("Flywheel Deadbeat Tuner")
        telemetry.addLine("Use Panels dashboard to adjust inertia and lag")
        telemetry.addLine("Controls:")
        telemetry.addLine("  D-pad up/down: Adjust step size")
        telemetry.addLine("  A: Toggle target (0 <-> step)")
        telemetry.addLine("  Right stick Y: Manual adjustment")
        telemetry.addLine("  B: Reset target to 0")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Update flywheel controller parameters from configurables
            shooter = Shooter(
                FlywheelPIDConstants.kP,
                FlywheelPIDConstants.kD,
                FlywheelPIDConstants.kI,
                FlywheelPIDConstants.maxVelocity,
                io
            )

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

            // Right stick Y: continuous manual adjustment
            target += -gamepad1.right_stick_y * continuousRate * dt.inWholeMilliseconds / 1000.0

            // Update flywheel controller
            shooter.flywheelTarget = target
            val currentVelocity = io.flywheelVelocity()
            shooter.update(dt)

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
            tel.addData("Motor Power", io.flywheel)
            tel.addLine("")

            tel.addLine("--- Config (adjust in Panels) ---")
            tel.addData("kP", FlywheelPIDConstants.kP)
            tel.addData("kD", FlywheelPIDConstants.kD)
            tel.addData("kI", FlywheelPIDConstants.kI)
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
            tel.update()

            false // continue loop
        }
    }
}
