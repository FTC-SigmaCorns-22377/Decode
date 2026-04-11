package sigmacorns.opmode.tune

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.PI

/**
 * Manually adjust hood angle and observe servo position.
 *
 * GAMEPAD 1:
 *   Right bumper  - Increase hood angle (+1°)
 *   Left bumper   - Decrease hood angle (-1°)
 *   Right trigger - Fine increase (+0.1°)
 *   Left trigger  - Fine decrease (-0.1°)
 */
@TeleOp(name = "Hood Tune", group = "Tune")
class HoodTune : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false)
        robot.init(Pose2d(), apriltagTracking = false)

        var hoodAngleDeg = ShooterConfig.defaultAngleDeg

        var lastLB = false
        var lastRB = false

        waitForStart()

        ioLoop { _, _ ->
            if (gamepad1.right_bumper && !lastRB) hoodAngleDeg += 1.0
            lastRB = gamepad1.right_bumper

            if (gamepad1.left_bumper && !lastLB) hoodAngleDeg -= 1.0
            lastLB = gamepad1.left_bumper

            hoodAngleDeg += gamepad1.right_trigger * 0.1
            hoodAngleDeg -= gamepad1.left_trigger * 0.1

            hoodAngleDeg = hoodAngleDeg.coerceIn(ShooterConfig.minAngleDeg, ShooterConfig.maxAngleDeg)

            robot.shooter.autoAdjust = false
            robot.shooter.manualHoodAngle = Math.toRadians(hoodAngleDeg)
            robot.shooter.flywheelTarget = 0.0
            robot.aimFlywheel = false

            robot.update()

            val tel = PanelsTelemetry.telemetry
            telemetry.addLine("=== HOOD TUNE ===")
            telemetry.addData("Angle (deg)", "%.2f°", hoodAngleDeg)
            telemetry.addData("Servo", "%.4f", robot.shooter.hoodServoPosition)
            telemetry.addLine("")
            telemetry.addLine("  RB/LB: +/- 1°")
            telemetry.addLine("  RT/LT: +/- 0.1°")
            telemetry.update()

            tel.addData("Hood Angle (deg)", hoodAngleDeg)
            tel.addData("Hood Servo", robot.shooter.hoodServoPosition)
            tel.update()

            false
        }

        robot.close()
    }
}