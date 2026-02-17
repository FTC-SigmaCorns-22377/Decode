package sigmacorns.opmode.tune

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@Configurable
object SpindexerPIDConfig {
    // These are no longer used for software PID (motor uses RUN_TO_POSITION),
    // but kept for reference / future use
    @JvmField var kP = 0.8
    @JvmField var kD = 0.0
    @JvmField var kI = 0.0

    @JvmField var maxVelocity = PI*2.0
    @JvmField var maxAcceleration = 10.0
}

@TeleOp(name = "Spindexer Position Tuner", group = "Test")
class SpindexerPIDTuner : SigmaOpMode() {

    private val ticksPerRev = (((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0
    private val ticksPerRadian = ticksPerRev / (2 * PI)

    private val discreteStep = (2 * PI) / 3  // 120 degrees (spindexer slot)
    private val continuousRate = 2.0

    override fun runOpMode() {
        var targetValue = 0.0

        telemetry.addLine("Spindexer Position Tuner (RUN_TO_POSITION)")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
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
                while (gamepad1.a && opModeIsActive()) { idle() }
            }

            // Get current value
            val currentPositionTicks = io.spindexerPosition()
            val currentValue = currentPositionTicks / ticksPerRadian

            // Set target position in ticks (motor controller handles PID)
            io.spindexer = targetValue * ticksPerRadian

            // Calculate error
            val error = targetValue - currentValue

            val telemetry = PanelsTelemetry.telemetry

            // Telemetry
            telemetry.addLine("=== Spindexer Position Tuner ===")
            telemetry.addLine("(Motor RUN_TO_POSITION mode)")
            telemetry.addLine("")
            telemetry.addData("Target (rad)", targetValue)
            telemetry.addData("Target (deg)", Math.toDegrees(targetValue))
            telemetry.addData("Target (ticks)", targetValue * ticksPerRadian)
            telemetry.addData("Current (rad)", currentValue)
            telemetry.addData("Current (deg)", Math.toDegrees(currentValue))
            telemetry.addLine("")
            telemetry.addData("Error (rad)", error)
            telemetry.addData("Error (deg)", Math.toDegrees(error))
            telemetry.addLine("")
            telemetry.addLine("Controls:")
            telemetry.addLine("  Right Stick Y: Adjust target")
            telemetry.addLine("  D-Pad Up/Down: Step +/- 120°")
            telemetry.addLine("  A: Reset to 0")
            telemetry.update()

            false // continue loop
        }
    }
}
