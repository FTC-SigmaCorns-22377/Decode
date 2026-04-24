package sigmacorns.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode

@Configurable
object FlywheelLQRConfig {
    @JvmField var targetVelocity = 200.0
    @JvmField var autoSpinUp = true
    @JvmField var hoodAngleDeg = 45.0
}

@TeleOp(name = "Flywheel LQR Test", group = "Test")
class FlywheelLQRTest : SigmaOpMode() {

    private var targetVel = 200.0
    private var lastLoopTime = 0L
    private var loopCount = 0

    override fun runOpMode() {
        val robot = Robot(io, blue = false)
        robot.init(Pose2d(), apriltagTracking = false)
        robot.aimTurret = false

        telemetry.addLine("Flywheel LQR Test")
        telemetry.addLine("A: Spin up | B: Stop | Right Stick: Manual velocity control")
        telemetry.update()

        waitForStart()

        lastLoopTime = System.currentTimeMillis()

        ioLoop { state, dt ->
            val now = System.currentTimeMillis()
            val loopTimeMs = now - lastLoopTime
            lastLoopTime = now
            loopCount++

            // Manual target control
            if (gamepad1.right_stick_y != 0f) {
                targetVel -= gamepad1.right_stick_y * 50.0 * dt.inWholeMilliseconds / 1000.0
                targetVel = targetVel.coerceIn(0.0, 800.0)
            }

            // Gamepad controls
            if (gamepad1.a && FlywheelLQRConfig.autoSpinUp) {
                targetVel = FlywheelLQRConfig.targetVelocity
                while (gamepad1.a && opModeIsActive()) { idle() }
            }

            if (gamepad1.b) {
                targetVel = 0.0
                while (gamepad1.b && opModeIsActive()) { idle() }
            }

            // Update hood angle
            val hoodRad = Math.toRadians(FlywheelLQRConfig.hoodAngleDeg)
            robot.shooter.manualHoodAngle = hoodRad
            robot.shooter.autoAdjust = false

            // Set target and update
            robot.shooter.flywheelTarget = targetVel
            robot.shooter.update(dt)

            io.update()

            // Telemetry via Panels
            val tel = PanelsTelemetry.telemetry
            tel.addLine("=== Flywheel LQR Controller ===")
            tel.addData("Loop Time (ms)", loopTimeMs)
            tel.addData("Loop Count", loopCount)
            tel.addLine("")
            tel.addData("Target Vel (rad/s)", targetVel)
            tel.addData("Current Vel (rad/s)", io.flywheelVelocity())
            tel.addData("Error (rad/s)", io.flywheelVelocity() - targetVel)
            tel.addData("Motor Power", io.flywheel)
            tel.addLine("")
            tel.addData("Hub Voltage (V)", io.voltage())
            tel.addData("Hood Angle (deg)", FlywheelLQRConfig.hoodAngleDeg)
            tel.addData("Hood Servo Pos", robot.shooter.hoodServoPosition)
            tel.addLine("")
            tel.addData("Ready to Shoot", io.flywheelVelocity() > targetVel * 0.95)
            tel.update()

            false
        }

        robot.close()
    }
}
