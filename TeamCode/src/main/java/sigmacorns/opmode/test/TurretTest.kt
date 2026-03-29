package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@TeleOp(name = "Turret Servo Test", group = "Test")
class TurretTest : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false)
        robot.init(Pose2d(), apriltagTracking = false)
        robot.aimTurret = false

        var angle = 0.0

        waitForStart()

        ioLoop { state, dt ->
            angle += 0.5 * (gamepad1.right_stick_x) * dt.inWholeMilliseconds / 1000.0
            angle = angle.coerceIn(-PI / 2.0, PI / 2.0)

            robot.turret.fieldRelativeMode = false
            robot.turret.targetAngle = angle
            robot.turret.update(dt)

            io.update()

            telemetry.addData("Turret Target (deg)", "%.1f", Math.toDegrees(angle))
            telemetry.addData("Turret Pos (deg)", "%.1f", Math.toDegrees(robot.turret.pos))
            telemetry.addData("Servo Position", "%.3f", robot.turret.currentServoPosition)
            telemetry.addData("Servo L/R", "%.3f / %.3f", io.turretLeft, io.turretRight)
            telemetry.addLine("Use Right Stick X to move turret")
            telemetry.update()

            false
        }

        robot.close()
    }
}
