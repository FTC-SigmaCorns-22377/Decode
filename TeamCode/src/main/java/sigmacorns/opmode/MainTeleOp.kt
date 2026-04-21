package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
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
 *
 *   Left trigger         - Hold to intake balls
 *   B                    - Hold to reverse intake (spit balls out)
 *   Right trigger        - Hold to shoot (flywheel + transfer)
 *
 * GAMEPAD 2 (Operator - all subsystem controls):
 *   Left stick           - Translate (mecanum drive, when GP2 drive enabled)
 *   Right stick X        - Rotate (when GP2 drive enabled) / Manual turret override (when drive disabled)
 *   X                    - Toggle GP2 mecanum drive on/off
 *   Left trigger         - Hold to intake (only when GP2 drive enabled)
 *   B                    - Hold to reverse intake (spit balls out)
 *   Right trigger        - Hold to shoot (only when GP2 drive enabled)
 *   Left bumper          - Hold to spin up flywheel (without shooting)
 *   Right bumper         - Toggle flywheel always-on (0.4 power)
 *   A                    - Toggle auto-aim on/off
 *   Right stick Y        - Manual hood angle override
 *   Y                    - Toggle auto-shoot (zone-based)
 *   D-pad up             - Toggle hood auto-adjust
 *   D-pad left           - Decrease flywheel target speed (-25 rad/s)
 *   D-pad right          - Increase flywheel target speed (+25 rad/s)
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
class   MainTeleOp : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = true)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = !SIM)
        robot.startApriltag()

        // State
        var autoAimEnabled = true
        var flywheelTargetSpeed = 400.0
        val flywheelSpeedStep = 25.0
        var flywheelAlwaysOn = true
        var gp2DriveEnabled = false

        // Toggle debounce flags (GP1)
        var lastX1 = false

        // Toggle debounce flags (GP2)
        var lastA2 = false
        var lastY2 = false
        var lastX2 = false
        var lastRightBumper2 = false
        var lastDpadUp2 = false
        var lastDpadLeft2 = false
        var lastDpadRight2 = false

        // Start in robot-centric mode
        robot.drive.fieldCentric = false

        telemetry.addLine("Main TeleOp initialized. Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // ============================================================
            // GAMEPAD 1: Driver (driving + intake/shoot)
            // ============================================================

            // X button toggles field-centric mode
            if (gamepad1.x && !lastX1) robot.drive.fieldCentric = !robot.drive.fieldCentric
            lastX1 = gamepad1.x

            robot.drive.fieldCentricHeading = robot.aim.autoAim.fusedPose.rot - 0.5*PI

            // Drive from GP1 always active; GP2 drive only when enabled
            if (gp2DriveEnabled) {
                robot.drive.update(gamepad2, io)
            } else {
                robot.drive.update(gamepad1, io)
            }

            // ============================================================
            // GAMEPAD 2: Operator (ALL subsystem controls)
            // ============================================================

            // --- Toggle: GP2 mecanum drive (X button) ---
            if (gamepad2.x && !lastX2) {
                gp2DriveEnabled = !gp2DriveEnabled
            }
            lastX2 = gamepad2.x

            // --- Toggle: Flywheel always-on (right bumper) ---
            if (gamepad2.right_bumper && !lastRightBumper2) {
                flywheelAlwaysOn = !flywheelAlwaysOn
            }
            lastRightBumper2 = gamepad2.right_bumper

            // --- Flywheel speed adjustment (D-pad left/right) ---
            if (gamepad2.dpad_left && !lastDpadLeft2) {
                flywheelTargetSpeed = (flywheelTargetSpeed - flywheelSpeedStep).coerceAtLeast(0.0)
            }
            lastDpadLeft2 = gamepad2.dpad_left

            if (gamepad2.dpad_right && !lastDpadRight2) {
                flywheelTargetSpeed = (flywheelTargetSpeed + flywheelSpeedStep).coerceAtMost(628.0)
            }
            lastDpadRight2 = gamepad2.dpad_right

            // --- Intake: GP1 left trigger, or GP2 left trigger when GP2 drive active ---
            val intakeTriggered = gamepad1.left_trigger > 0.1 ||
                (gp2DriveEnabled && gamepad2.left_trigger > 0.1)
            val shootTriggered = gamepad1.right_trigger > 0.1 ||
                (gp2DriveEnabled && gamepad2.right_trigger > 0.1)
            if (intakeTriggered && !shootTriggered) {
                robot.intakeCoordinator.startIntake()
            } else if (!gamepad1.b && !gamepad2.b) {
                if (robot.intakeTransfer.state == IntakeTransfer.State.INTAKING ||
                    robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
            }

            // --- Outtake: hold B (GP1 or GP2) ---
            if (gamepad1.b || gamepad2.b) {
                robot.intakeTransfer.state = IntakeTransfer.State.REVERSING
            } else if (robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }

            // --- Toggle: Auto-aim (A button) ---
            if (gamepad2.a && !lastA2) {
                autoAimEnabled = !autoAimEnabled
            }
            lastA2 = gamepad2.a

            // --- Toggle: Auto-shoot zones (Y button) ---
            if (gamepad2.y && !lastY2) {
                robot.intakeCoordinator.autoShootEnabled =
                    !robot.intakeCoordinator.autoShootEnabled
            }
            lastY2 = gamepad2.y

            // --- Toggle: Hood auto-adjust (D-pad up) ---
            if (gamepad2.dpad_up && !lastDpadUp2) {
                robot.shooter.autoAdjust = !robot.shooter.autoAdjust
            }
            lastDpadUp2 = gamepad2.dpad_up

            // --- Turret control (only when GP2 drive is disabled) ---
            if (!gp2DriveEnabled) {
                val manualTurretInput = gamepad2.right_stick_x.toDouble()
                if (abs(manualTurretInput) > 0.05) {
                    robot.aimTurret = false
                    robot.turret.fieldRelativeMode = false
                    robot.turret.targetAngle += manualTurretInput * 2.0 * dt.inWholeMilliseconds / 1000.0
                    robot.turret.targetAngle = robot.turret.targetAngle.coerceIn(-PI, PI)
                } else if (autoAimEnabled) {
                    robot.aimTurret = true
                }
            }

            // --- Hood manual override (right stick Y) ---
            val manualHoodInput = -gamepad2.right_stick_y.toDouble()
            if (abs(manualHoodInput) > 0.05) {
                robot.shooter.autoAdjust = false
                robot.shooter.manualHoodAngle += manualHoodInput * Math.toRadians(30.0) * dt.inWholeMilliseconds / 1000.0
                robot.shooter.manualHoodAngle = robot.shooter.manualHoodAngle.coerceIn(
                    Math.toRadians(ShooterConfig.minAngleDeg),
                    Math.toRadians(ShooterConfig.maxAngleDeg)
                )
            }

            // --- Shooting (right trigger) or flywheel spin-up (left bumper) ---
            if (shootTriggered) {
                if(autoAimEnabled) {
                    robot.aimFlywheel = true
                    robot.aim.shotRequested = true
                } else {
                    robot.shooter.flywheelTarget = flywheelTargetSpeed
                    robot.aimFlywheel = false
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                }
            } else if (gamepad2.left_bumper) {
                // Spin-up only: flywheel on, no transfer
                robot.shooter.flywheelTarget = flywheelTargetSpeed
                robot.aimFlywheel = false
                robot.aim.shotRequested = false
            } else {
                robot.aimFlywheel = false
                robot.aim.shotRequested = false
                if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
                if (flywheelAlwaysOn) {
                    robot.shooter.flywheelTarget = 0.0
                } else {
                    robot.shooter.flywheelTarget = 0.0
                }
            }

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
            telemetry.addData("Auto-Aim", if (autoAimEnabled) "ON" else "OFF")

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
            telemetry.addData("Flywheel Always-On", if (flywheelAlwaysOn) "ON" else "OFF")
            telemetry.addData("GP2 Drive", if (gp2DriveEnabled) "ON" else "OFF")
            telemetry.addData("Speed Mode", if (robot.drive.getSpeedMultiplier() == 1.0) "FULL" else "PRECISION")
            telemetry.addData("Field-Centric", if (robot.drive.fieldCentric) "ON" else "OFF")
            telemetry.addData("Loop Time", "%d ms", dt.inWholeMilliseconds)
            telemetry.update()

            false // continue loop
        }

        robot.close()
    }
}
