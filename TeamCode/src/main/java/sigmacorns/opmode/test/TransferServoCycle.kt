package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode

@TeleOp(name = "Transfer Servo Cycle", group = "Test")
class TransferServoCycle : SigmaOpMode() {
    //this might be broken right now bc I changed some things withing out really fully refining them but is just a test opmode
    override fun runOpMode() {
        // For discrete 180 degree servo: 0 to 1, 0 means completey contracted and 1 means fully extended
        var position = 0.0  // Start stopped

        waitForStart()

        var prevRightBumper = false
        var prevLeftBumper = false

        ioLoop { state, dt ->
            // Dpad for direct power control
            position = when {
                gamepad1.dpad_up -> 0.5      // Full power up (transfer)
                gamepad1.dpad_down -> 0.0   // Full power down (reset)/ stop
                gamepad1.dpad_left -> 0.2   // Half Way
                else -> position                // Hold current power
            }

            // Manual fine control with bumpers (discrete 0.05 increments on press)
            if (gamepad1.right_bumper && !prevRightBumper) {
                position = (position + 0.05).coerceIn(0.0, 0.15)
            }
            if (gamepad1.left_bumper && !prevLeftBumper) {
                position = (position - 0.05).coerceIn(0.0, 0.15)
            }
            prevRightBumper = gamepad1.right_bumper
            prevLeftBumper = gamepad1.left_bumper

            io.transfer = position

            telemetry.addData("Transfer Power", "%.3f", position)
            telemetry.addData("Direction", when {
                position > 0.10 -> "UP (Transfer)"
                position < 0.03 -> "DOWN (Reset)"
                else -> "STOPPED"
            })
            telemetry.addLine()
            telemetry.addLine("Controls (Discrete Servo):")
            telemetry.addLine("  Dpad Up: Fully extended (0.15)")
            telemetry.addLine("  Dpad Down: Fully retracted (0.0)")
            telemetry.addLine("  Dpad Left: Half way (0.07)")
            telemetry.addLine("  Bumpers: +/- 0.05 increment")
            telemetry.update()

            false
        }
    }
}
