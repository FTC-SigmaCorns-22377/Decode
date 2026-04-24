package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.max

/**
 * Per-motor drivetrain test. Spin each wheel independently so you can confirm
 * wiring and direction before trusting the mecanum math.
 *
 * Either gamepad works (inputs OR'd between gamepad1 and gamepad2):
 *   A held  → driveFL
 *   B held  → driveFR
 *   X held  → driveBL
 *   Y held  → driveBR
 *   LB held → all four forward (should drive straight)
 *   RB held → all four reverse
 *   LT      → scale power up to 0.6 (default 0.3)
 */
@TeleOp(name = "Drivetrain Motor Test Bot 2", group = "Test")
class DrivetrainMotorTestBot2 : SigmaOpMode() {

    override fun runOpMode() {
        telemetry.addLine("Drivetrain Motor Test Bot 2")
        telemetry.addLine("Either gamepad:")
        telemetry.addLine("A/B/X/Y = FL/FR/BL/BR")
        telemetry.addLine("LB = all fwd, RB = all rev, LT scales power")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val a  = gamepad1.a  || gamepad2.a
            val b  = gamepad1.b  || gamepad2.b
            val x  = gamepad1.x  || gamepad2.x
            val y  = gamepad1.y  || gamepad2.y
            val lb = gamepad1.left_bumper  || gamepad2.left_bumper
            val rb = gamepad1.right_bumper || gamepad2.right_bumper
            val lt = max(gamepad1.left_trigger, gamepad2.left_trigger)

            val scale = 0.3 + 0.3 * lt
            var fl = 0.0; var fr = 0.0; var bl = 0.0; var br = 0.0

            if (a) fl = scale
            if (b) fr = scale
            if (x) bl = scale
            if (y) br = scale
            if (lb) { fl = scale; fr = scale; bl = scale; br = scale }
            if (rb) { fl = -scale; fr = -scale; bl = -scale; br = -scale }

            io.driveFL = fl
            io.driveFR = fr
            io.driveBL = bl
            io.driveBR = br
            io.update()

            telemetry.addData("scale", "%.2f".format(scale))
            telemetry.addData("FL", "%+.2f".format(fl))
            telemetry.addData("FR", "%+.2f".format(fr))
            telemetry.addData("BL", "%+.2f".format(bl))
            telemetry.addData("BR", "%+.2f".format(br))
            telemetry.update()
        }

        io.driveFL = 0.0; io.driveFR = 0.0; io.driveBL = 0.0; io.driveBR = 0.0
        io.update()
    }
}
