package sigmacorns.opmode.tune

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer

@TeleOp(name = "Blocker Tune", group = "Tune")
class BlockerTune : SigmaOpMode() {

    override fun runOpMode() {
        var position = IntakeTransfer.BLOCKER_ENGAGED

        waitForStart()

        ioLoop { _, _ ->
            position += gamepad1.right_trigger * 0.001
            position -= gamepad1.left_trigger * 0.001
            position = position.coerceIn(0.0, 1.0)

            io.blocker = position

            telemetry.addLine("=== BLOCKER TUNE ===")
            telemetry.addData("Position", "%.4f", position)
            telemetry.addLine("")
            telemetry.addData("ENGAGED const", "%.4f", IntakeTransfer.BLOCKER_ENGAGED)
            telemetry.addData("DISENGAGED const", "%.4f", IntakeTransfer.BLOCKER_DISENGAGED)
            telemetry.addLine("")
            telemetry.addLine("  RT: increase (+0.001/loop)")
            telemetry.addLine("  LT: decrease (-0.001/loop)")
            telemetry.update()

            false
        }
    }
}
