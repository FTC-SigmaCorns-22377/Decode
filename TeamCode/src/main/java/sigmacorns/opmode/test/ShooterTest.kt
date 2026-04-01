package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.PI

/**
 * Test OpMode for flywheel + hood only.
 *
 * GAMEPAD 1:
 *   Right bumper  - Increase target RPM (+25 rad/s)
 *   Left bumper   - Decrease target RPM (-25 rad/s)
 *   A             - Toggle flywheel on/off
 */
@TeleOp(name = "Shooter Test", group = "Test")
class ShooterTest : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false)
        robot.init(Pose2d(), apriltagTracking = false)

        var flywheelTargetSpeed = 400.0
        val flywheelSpeedStep = 25.0
        var flywheelEnabled = false

        var lastA = false
        var lastLB = false
        var lastRB = false

        telemetry.addLine("Shooter Test initialized. Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // --- Toggle flywheel on/off (A) ---
            if (gamepad1.a && !lastA) {
                flywheelEnabled = !flywheelEnabled
            }
            lastA = gamepad1.a

            // --- Adjust target RPM (bumpers) ---
            if (gamepad1.right_bumper && !lastRB) {
                flywheelTargetSpeed = (flywheelTargetSpeed + flywheelSpeedStep).coerceAtMost(628.0)
            }
            lastRB = gamepad1.right_bumper

            if (gamepad1.left_bumper && !lastLB) {
                flywheelTargetSpeed = (flywheelTargetSpeed - flywheelSpeedStep).coerceAtLeast(0.0)
            }
            lastLB = gamepad1.left_bumper

            // --- Apply flywheel target ---
            robot.shooter.flywheelTarget = if (flywheelEnabled) flywheelTargetSpeed else 0.0
            robot.aimFlywheel = false

            // --- Robot update ---
            robot.update()
            io.update()

            // --- Telemetry ---
            val flywheelVel = io.flywheelVelocity()
            val flywheelRPM = flywheelVel * 60.0 / (2.0 * PI)

            telemetry.addLine("=== SHOOTER TEST ===")
            telemetry.addData("Flywheel", if (flywheelEnabled) "ON" else "OFF")
            telemetry.addData("Target", "%.1f rad/s", flywheelTargetSpeed)
            telemetry.addData("Velocity", "%.1f rad/s", flywheelVel)
            telemetry.addData("RPM", "%.0f", flywheelRPM)
            telemetry.addData("Motor Power", "%.3f", io.flywheel)

            telemetry.addLine("")
            telemetry.addLine("=== HOOD ===")
            telemetry.addData("Angle", "%.1f°", Math.toDegrees(robot.shooter.computedHoodAngle))
            telemetry.addData("Mode", if (robot.shooter.autoAdjust) "AUTO" else "MANUAL")
            telemetry.addData("Servo", "%.3f", robot.shooter.hoodServoPosition)

            telemetry.addLine("")
            telemetry.addLine("=== CONTROLS ===")
            telemetry.addLine("  A: Toggle flywheel")
            telemetry.addLine("  RB/LB: Target +/- 25 rad/s")
            telemetry.update()

            false
        }

        robot.close()
    }
}
