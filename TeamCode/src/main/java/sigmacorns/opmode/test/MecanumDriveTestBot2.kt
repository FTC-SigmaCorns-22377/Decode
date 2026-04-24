package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.Drivetrain
import kotlin.math.abs

/**
 * Standard mecanum teleop — forces robot-centric driving and full power, so
 * "forward stick" always means "robot forward" regardless of pinpoint heading.
 * (The project's Drivetrain defaults to field-centric, which rotates the input
 * by io.position().rot — if pinpoint has a stale heading, pushing forward
 * sends partial power sideways and feels weak. That's the "wheels barely
 * moving" symptom — not a power-cap issue.)
 *
 * Either gamepad works — whichever has the larger stick magnitude this tick
 * drives. Idle gamepads are ignored.
 *
 *   Left stick  → translation (robot-centric: y-up = forward)
 *   Right stick → rotation (X)
 *   LT          → 50% slow mode while held (fine control)
 *   RB          → 150% boost while held (if wheels need extra torque to break
 *                 static friction; applied BEFORE the Drivetrain's own
 *                 normalization, so actual output is still capped at 1.0)
 */
@TeleOp(name = "Mecanum Drive Test Bot 2", group = "Test")
class MecanumDriveTestBot2 : SigmaOpMode() {

    override fun runOpMode() {
        val drive = Drivetrain().apply {
            fieldCentric = false
        }

        telemetry.addLine("Mecanum Drive Test Bot 2 (ROBOT-CENTRIC)")
        telemetry.addLine("Either gamepad: LS translate, RS X rotate")
        telemetry.addLine("LT = 50% slow, RB = 150% boost")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val gp1Mag = abs(gamepad1.left_stick_x) + abs(gamepad1.left_stick_y) +
                         abs(gamepad1.right_stick_x)
            val gp2Mag = abs(gamepad2.left_stick_x) + abs(gamepad2.left_stick_y) +
                         abs(gamepad2.right_stick_x)
            val active = if (gp2Mag > gp1Mag) gamepad2 else gamepad1

            val slow  = active.left_trigger > 0.1
            val boost = active.right_bumper
            val scale = when {
                slow  -> 0.5
                boost -> 1.5    // Drivetrain.drive normalizes back to <=1.0 per wheel
                else  -> 1.0
            }

            val fwd  = -active.left_stick_y.toDouble() * scale
            val left = -active.left_stick_x.toDouble() * scale
            val yaw  = -active.right_stick_x.toDouble() * scale

            drive.drive(Pose2d(fwd, left, yaw), io)
            io.update()

            telemetry.addData("active", if (active === gamepad2) "gamepad2" else "gamepad1")
            telemetry.addData("scale", "%.2f".format(scale))
            telemetry.addData("cmd", "fwd=%+.2f left=%+.2f yaw=%+.2f".format(fwd, left, yaw))
            telemetry.addData("FL", "%+.2f".format(io.driveFL))
            telemetry.addData("FR", "%+.2f".format(io.driveFR))
            telemetry.addData("BL", "%+.2f".format(io.driveBL))
            telemetry.addData("BR", "%+.2f".format(io.driveBR))
            val pose = io.position()
            telemetry.addData("pose", "x=%.2f y=%.2f θ=%.2f".format(pose.v.x, pose.v.y, pose.rot))
            telemetry.update()
        }

        io.driveFL = 0.0; io.driveFR = 0.0; io.driveBL = 0.0; io.driveBR = 0.0
        io.update()
    }
}
