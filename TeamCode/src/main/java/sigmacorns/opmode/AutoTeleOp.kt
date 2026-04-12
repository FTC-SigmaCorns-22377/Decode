package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.logic.AutomationManager
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max

/**
 * Full-featured competition TeleOp.
 *
 * GAMEPAD 1 (Driver):
 *   Left stick          - Translate (mecanum drive)
 *   Right stick X       - Rotate
 *   D-pad up/down       - Speed mode toggle (full / precision)
 *   X                    - Toggle field-centric drive
 *   Y                    - Toggle auto shoot
 *
 *   Left trigger         - Hold to intake balls
 *   B                    - Hold to reverse intake (spit balls out)
 */
@TeleOp(name = "Auto TeleOp", group = "Competition")
class AutoTeleOp : SigmaOpMode() {
    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = true)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = !SIM)
        robot.startApriltag()

        val automationManager = AutomationManager(robot)

        // State
        robot.intakeCoordinator.autoShootEnabled = true
        robot.aimTurret = true
        robot.aimFlywheel = true

        var lastX1 = false
        var lastY2 = false

        telemetry.addLine("Main TeleOp initialized. Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // ============================================================
            // GAMEPAD 1: Driver (driving ONLY) + Debug/Tuning
            // ============================================================

            // Drivetrain: left stick = translate, right stick x = rotate, dpad = speed mode
            // X button toggles field-centric mode
            if (gamepad1.x && !lastX1) robot.drive.fieldCentric = !robot.drive.fieldCentric
            lastX1 = gamepad1.x

            if(gamepad1.right_bumper && !automationManager.hasControl) {
                automationManager.shootFarZone()
            }

            if(!automationManager.update(gamepad1)) {
                robot.drive.fieldCentricHeading = robot.aim.autoAim.fusedPose.rot - 0.5*PI
                robot.drive.update(gamepad1, io)
            }

            // --- Intake: hold left trigger ---
            if ( (max(gamepad2.left_trigger,gamepad1.left_trigger) > 0.1 && max(gamepad2.right_trigger,gamepad1.right_trigger) <= 0.1) ) {
                robot.intakeCoordinator.startIntake()
            } else if (!gamepad2.b || !gamepad1.b) {
                if (robot.intakeTransfer.state == IntakeTransfer.State.INTAKING ||
                    robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
            }

            // --- Outtake: hold B to reverse intake ---
            if (gamepad2.b || gamepad1.b) {
                robot.intakeTransfer.state = IntakeTransfer.State.REVERSING
            } else if (robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }

            // --- Toggle: Auto-shoot zones (Y button) ---
            if (gamepad2.y && !lastY2) {
                robot.intakeCoordinator.autoShootEnabled = !robot.intakeCoordinator.autoShootEnabled
            }
            lastY2 = gamepad2.y

            // ============================================================
            // Robot update
            // ============================================================
            robot.update()
            io.update()

            // ============================================================
            // Telemetry
            // ============================================================
            val fusedPose = robot.aim.autoAim.fusedPose
            val flywheelVel = io.flywheelVelocity()
            val flywheelRPM = flywheelVel * 60.0 / (2.0 * PI)

            telemetry.addLine("=== BALL STATUS ===")
            telemetry.addData("Balls Held", robot.beamBreak.ballCount)
            telemetry.addData("Beam 1|2|3", robot.beamBreak.slots.map { if (it == true) "O" else "-" })

            telemetry.addLine("")
            telemetry.addLine("=== POSITION ===")
            telemetry.addData("Fused Pose", "x=%.3f y=%.3f θ=%.1f°",
                fusedPose.v.x, fusedPose.v.y, Math.toDegrees(fusedPose.rot))
            telemetry.addData("Odometry", "x=%.3f y=%.3f θ=%.1f°",
                io.position().v.x, io.position().v.y, Math.toDegrees(io.position().rot))
            telemetry.addData("Goal Distance", "%.2f m", robot.aim.targetDistance)

            telemetry.addLine("")
            telemetry.addLine("=== VISION ===")
            telemetry.addData("Vision Target", robot.aim.autoAim.hasVisionTarget)

            telemetry.addLine("")
            telemetry.addLine("=== SHOOTER ===")
            telemetry.addData("Flywheel RPM", "%.0f", flywheelRPM)
            telemetry.addData("Flywheel Vel", "%.1f rad/s", flywheelVel)
            telemetry.addData("Flywheel Target", "%.1f rad/s (GP2 D-pad L/R)", robot.shooter.flywheelTarget)
            telemetry.addData("Hood Angle", "%.1f°", Math.toDegrees(robot.shooter.computedHoodAngle))
            telemetry.addData("Hood", if (robot.shooter.autoAdjust) "AUTO" else "MANUAL")
            telemetry.addData("Hood Servo", "%.3f", robot.shooter.hoodServoPosition)

            telemetry.addLine("")
            telemetry.addLine("=== TURRET ===")
            telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(robot.turret.pos))
            telemetry.addData("Turret Target", "%.1f°", Math.toDegrees(robot.turret.effectiveTargetAngle))
            telemetry.addData("Turret Servo Actual", "%.3f", robot.turret.currentServoPosition)
            telemetry.addData("Turret Servo Target", "%.3f", robot.io.turretRight)
            telemetry.addData("Field-Relative", robot.turret.fieldRelativeMode)

            telemetry.addLine("")
            telemetry.addLine("=== SUBSYSTEMS ===")
            val intake1RPM = io.intake1RPM()
            telemetry.addData("Intake", when {
                robot.intakeTransfer.state == IntakeTransfer.State.REVERSING -> "REVERSING"
                robot.intakeTransfer.state == IntakeTransfer.State.INTAKING -> "RUNNING"
                else -> "IDLE"
            })
            telemetry.addData("Intake1 RPM", "%.0f", intake1RPM)
            telemetry.addData("Transfer", if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING) "RUNNING" else "IDLE")
            telemetry.addData("Blocker", if (io.blocker == 0.0) "ENGAGED" else "DISENGAGED")
            telemetry.addData("Auto-Shoot", if (robot.intakeCoordinator.autoShootEnabled) "ON" else "OFF")
            telemetry.addData("In Shoot Zone", robot.intakeCoordinator.inShootingZone)
            telemetry.addData("Speed Mode", if (robot.drive.getSpeedMultiplier() == 1.0) "FULL" else "PRECISION")
            telemetry.addData("Field-Centric", if (robot.drive.fieldCentric) "ON" else "OFF")
            telemetry.addData("Loop Time", "%d ms", dt.inWholeMilliseconds)
            telemetry.update()

            false // continue loop
        }

        robot.close()
    }
}
