package sigmacorns.opmode.test

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.PI
import kotlin.math.abs
import kotlin.time.Duration

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

        // Spinup time measurement
        var spinupStartTime: Duration? = null
        var spinupDuration: Double? = null // persists after disable
        val spinupTolerance = 4.0 // rad/s

        telemetry.addLine("Shooter Test initialized. Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // --- Toggle flywheel on/off (A) ---
            if (gamepad1.a && !lastA) {
                flywheelEnabled = !flywheelEnabled
                if (flywheelEnabled) {
                    // Reset timer on enable
                    spinupStartTime = state.timestamp
                    spinupDuration = null
                }
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

            // --- Telemetry (both driver station and Panels) ---
            val flywheelVel = io.flywheelVelocity()
            val flywheelRPM = flywheelVel * 60.0 / (2.0 * PI)
            val tel = PanelsTelemetry.telemetry

            // --- Spinup time measurement ---
            if (flywheelEnabled && spinupDuration == null && spinupStartTime != null) {
                if (abs(flywheelVel - flywheelTargetSpeed) <= spinupTolerance) {
                    spinupDuration = (state.timestamp - spinupStartTime!!).inWholeMilliseconds / 1000.0
                }
            }

            telemetry.addLine("=== SHOOTER TEST ===")
            telemetry.addData("Flywheel", if (flywheelEnabled) "ON" else "OFF")
            telemetry.addData("Target", "%.1f rad/s", flywheelTargetSpeed)
            telemetry.addData("Velocity", "%.1f rad/s", flywheelVel)
            telemetry.addData("RPM", "%.0f", flywheelRPM)
            telemetry.addData("Motor Power", "%.3f", io.flywheel)
            telemetry.addData("Spinup Time", if (spinupDuration != null) "%.3f s".format(spinupDuration) else if (flywheelEnabled) "measuring..." else "---")
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

            tel.addData("Target (rad/s)", flywheelTargetSpeed)
            tel.addData("Velocity (rad/s)", flywheelVel)
            tel.addData("RPM", flywheelRPM)
            tel.addData("Motor Power", io.flywheel)
            tel.addData("Hood Angle (deg)", Math.toDegrees(robot.shooter.computedHoodAngle))
            tel.addData("Hood Servo", robot.shooter.hoodServoPosition)
            tel.update()

            false
        }

        robot.close()
    }
}
