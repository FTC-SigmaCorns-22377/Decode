package sigmacorns.opmode.tune

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer

/**
 * Manually adjust blocker servo position.
 *
 * GAMEPAD 1:
 *   Right bumper  - Increase position (+0.01)
 *   Left bumper   - Decrease position (-0.01)
 *   Right trigger - Fine increase (+0.001)
 *   Left trigger  - Fine decrease (-0.001)
 *   A             - Set as ENGAGED value
 *   B             - Set as DISENGAGED value
 */
@TeleOp(name = "Blocker Tune", group = "Tune")
class BlockerTune : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false)
        robot.init(Pose2d(), apriltagTracking = false)

        var position = IntakeTransfer.BLOCKER_ENGAGED

        var lastLB = false
        var lastRB = false

        waitForStart()

        ioLoop { _, _ ->
            if (gamepad1.right_bumper && !lastRB) position += 0.01
            lastRB = gamepad1.right_bumper

            if (gamepad1.left_bumper && !lastLB) position -= 0.01
            lastLB = gamepad1.left_bumper

            position += gamepad1.right_trigger * 0.001
            position -= gamepad1.left_trigger * 0.001

            position = position.coerceIn(0.0, 1.0)

            io.blocker = position
            io.update()

            val tel = PanelsTelemetry.telemetry
            telemetry.addLine("=== BLOCKER TUNE ===")
            telemetry.addData("Position", "%.4f", position)
            telemetry.addData("BLOCKER_ENGAGED (const)", "%.4f", IntakeTransfer.BLOCKER_ENGAGED)
            telemetry.addData("BLOCKER_DISENGAGED (const)", "%.4f", IntakeTransfer.BLOCKER_DISENGAGED)
            telemetry.addLine("")
            telemetry.addLine("  RB/LB: +/- 0.01")
            telemetry.addLine("  RT/LT: +/- 0.001")
            telemetry.update()

            tel.addData("Blocker Position", position)
            tel.update()

            false
        }

        robot.close()
    }
}