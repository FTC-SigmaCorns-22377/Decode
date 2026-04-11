package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.constants.flywheelMotor
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import sigmacorns.subsystem.TurretServoConfig
import kotlin.math.PI
import kotlin.time.DurationUnit
import kotlin.time.times

/**
 * Demo TeleOp for showing off the bot with limited capabilities.
 *
 * GAMEPAD 1 (Driver):
 *   Left stick          - Translate (mecanum drive, limited speed)
 *   Right stick X       - Rotate
 *   Right trigger       - Flywheel velocity target (scaled by trigger value)
 *   Right bumper        - Shoot (transfer ball into flywheel)
 *   Left trigger        - Hold to intake balls
 *   B                   - Hold to reverse intake (spit balls out)
 *   Hood angle fixed
 */
@TeleOp(name = "Demo TeleOp", group = "Demo")
class DemoTeleOp : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = false)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = false)

        // Demo parameters
        val demoSpeedLimit = 0.4  // 40% of max speed
        val fixedHoodAngleDeg = 70.0
        val flywheelMinSpeed = 0.0
        val flywheelMaxSpeed = flywheelMotor.freeSpeed*0.7
        val flywheelSpeedStep = 5.0  // rad/s per unit trigger

        // State
        var flywheelTargetSpeed = 350.0
        var turretAngle = 0.0

        telemetry.addLine("Demo TeleOp initialized. Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // ============================================================
            // GAMEPAD 1: Driver (limited driving + shooter control)
            // ============================================================

            // Drivetrain: left stick = translate (limited speed), right stick x = rotate
            var inputX = -gamepad1.left_stick_y.toDouble() * demoSpeedLimit
            var inputY = -gamepad1.left_stick_x.toDouble() * demoSpeedLimit
            val inputRot = -gamepad1.right_stick_x.toDouble() * demoSpeedLimit

            robot.drive.drive(Pose2d(inputX, inputY, inputRot), io)

            // --- Hood: fixed position ---
            robot.shooter.autoAdjust = false
            robot.shooter.manualHoodAngle = Math.toRadians(fixedHoodAngleDeg)

            // --- Flywheel: right trigger controls target speed ---
            val triggerValue = gamepad1.right_trigger.toDouble()
            if (triggerValue > 0.05) {
                flywheelTargetSpeed = flywheelMinSpeed + (flywheelMaxSpeed - flywheelMinSpeed) * triggerValue
            } else {
                flywheelTargetSpeed = 0.0
            }

            robot.shooter.flywheelTarget = flywheelTargetSpeed
            robot.aimFlywheel = false
            robot.aimTurret = false

            if(gamepad1.dpad_left) {
                turretAngle += 0.5*dt.toDouble(DurationUnit.SECONDS)
            }

            if(gamepad1.dpad_right) {
                turretAngle -= 0.5*dt.toDouble(DurationUnit.SECONDS)
            }

            turretAngle = turretAngle.coerceIn(robot.turret.angleLimits)

            robot.turret.fieldRelativeMode = true
            robot.turret.fieldTargetAngle = turretAngle

            // --- Intake: hold left trigger ---
            if (gamepad1.left_trigger > 0.1 && !gamepad1.b && !gamepad1.right_bumper) {
                robot.intakeCoordinator.startIntake()
            } else if (!gamepad1.b && !gamepad1.right_bumper) {
                if (robot.intakeTransfer.state == IntakeTransfer.State.INTAKING ||
                    robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
            }

            // --- Outtake: hold B to reverse intake ---
            if (gamepad1.b) {
                robot.intakeTransfer.state = IntakeTransfer.State.REVERSING
            } else if (robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }

            // --- Shoot: hold right bumper to transfer ball into flywheel ---
            if (gamepad1.right_bumper) {
                robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
            } else if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING) {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }

            // ============================================================
            // Robot update
            // ============================================================
            robot.update()
            io.update()

            // ============================================================
            // Telemetry
            // ============================================================
            val flywheelVel = io.flywheelVelocity()
            val flywheelRPM = flywheelVel * 60.0 / (2.0 * PI)

            telemetry.addLine("=== DEMO MODE ===")
            telemetry.addLine("Speed Limited: ${(demoSpeedLimit * 100).toInt()}% max")
            telemetry.addLine("Hood Angle: ${fixedHoodAngleDeg.toInt()}° (fixed)")

            telemetry.addLine("")
            telemetry.addLine("=== CONTROLS ===")
            telemetry.addData("Left Stick", "Drive (limited speed)")
            telemetry.addData("Right Stick X", "Rotate")
            telemetry.addData("Right Trigger", "Flywheel: %.0f %%", gamepad1.right_trigger * 100.0)
            telemetry.addData("Right Bumper", "Shoot")
            telemetry.addData("Left Trigger", "Intake")
            telemetry.addData("B", "Reverse intake")

            telemetry.addLine("")
            telemetry.addLine("=== SHOOTER STATUS ===")
            telemetry.addData("Flywheel RPM", "%.0f", flywheelRPM)
            telemetry.addData("Flywheel Target", "%.1f rad/s", flywheelTargetSpeed)
            telemetry.addData("Hood Angle", "%.1f°", Math.toDegrees(robot.shooter.computedHoodAngle))
            telemetry.addData("Hood Servo", "%.3f", robot.shooter.hoodServoPosition)

            telemetry.addLine("")
            telemetry.addLine("=== BALLS / INTAKE ===")
            telemetry.addData("Balls Held", robot.beamBreak.ballCount)
            telemetry.addData("State", when (robot.intakeTransfer.state) {
                IntakeTransfer.State.INTAKING -> "INTAKING"
                IntakeTransfer.State.REVERSING -> "REVERSING"
                IntakeTransfer.State.TRANSFERRING -> "SHOOTING"
                else -> "IDLE"
            })

            telemetry.addLine("")
            telemetry.addLine("=== POSITION ===")
            val pos = io.position()
            telemetry.addData("Position", "x=%.3f y=%.3f θ=%.1f°",
                pos.v.x, pos.v.y, Math.toDegrees(pos.rot))

            telemetry.addLine("")
            telemetry.addData("Loop Time", "%d ms", dt.inWholeMilliseconds)
            telemetry.update()

            false // continue loop
        }

        robot.close()
    }
}
