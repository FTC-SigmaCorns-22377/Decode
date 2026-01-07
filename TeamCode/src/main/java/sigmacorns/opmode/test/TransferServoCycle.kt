package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode

@TeleOp(name = "Transfer Servo Cycle", group = "Test")
class TransferServoCycle : SigmaOpMode() {

    override fun runOpMode() {
        // For continuous servo: -1 to 1, 0 = stopped
        var power = 0.0  // Start stopped

        waitForStart()

        ioLoop { state, dt ->
            // Dpad for direct power control
            power = when {
                gamepad1.dpad_up -> 1.0      // Full power up (transfer)
                gamepad1.dpad_down -> -1.0   // Full power down (reset)
                gamepad1.dpad_left -> -0.5   // Slow down
                gamepad1.dpad_right -> 0.5   // Slow up
                gamepad1.a -> 0.0            // Stop
                else -> power                // Hold current power
            }

            // Manual fine control with bumpers
            if (gamepad1.right_bumper) {
                power = (power + 1.0 * dt.inWholeMilliseconds / 1000.0).coerceIn(-1.0, 1.0)
            }
            if (gamepad1.left_bumper) {
                power = (power - 1.0 * dt.inWholeMilliseconds / 1000.0).coerceIn(-1.0, 1.0)
            }

            io.transfer = power

            telemetry.addData("Transfer Power", "%.3f", power)
            telemetry.addData("Direction", when {
                power > 0.1 -> "UP (Transfer)"
                power < -0.1 -> "DOWN (Reset)"
                else -> "STOPPED"
            })
            telemetry.addLine()
            telemetry.addLine("Controls (Continuous Servo):")
            telemetry.addLine("  Dpad Up: Full power up (1.0)")
            telemetry.addLine("  Dpad Down: Full power down (-1.0)")
            telemetry.addLine("  Dpad L/R: Slow down/up")
            telemetry.addLine("  A: Stop (0.0)")
            telemetry.addLine("  Bumpers: Fine adjust")
            telemetry.update()

            false
        }
    }
}
