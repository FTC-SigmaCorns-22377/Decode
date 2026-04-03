package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.PI
import kotlin.math.abs

/**
 * Full-featured competition TeleOp.
 *
 * GAMEPAD 1 (Driver):
 *   Left stick          - Translate (mecanum drive)
 *   Right stick X       - Rotate
 *   D-pad up/down       - Speed mode toggle (full / precision)
 *
 * GAMEPAD 2 (Operator - all subsystem controls):
 *   Left trigger         - Hold to intake balls
 *   B                    - Hold to reverse intake (spit balls out)
 *   Right trigger        - Hold to shoot (flywheel + transfer)
 *   Left bumper          - Hold to spin up flywheel (without shooting)
 *   A                    - Toggle auto-aim on/off
 *   Right stick X        - Manual turret override (disables auto-aim while held)
 *   Right stick Y        - Manual hood angle override
 *   X                    - Toggle shoot-while-move
 *   Y                    - Toggle auto-shoot (zone-based)
 *   D-pad up             - Toggle hood auto-adjust
 *   D-pad left           - Decrease flywheel target speed (-25 rad/s)
 *   D-pad right          - Increase flywheel target speed (+25 rad/s)
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
class   MainTeleOp : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false)
        robot.init(Pose2d(), apriltagTracking = true)
        robot.startApriltag()

        // State
        var autoAimEnabled = false
        var flywheelTargetSpeed = 400.0
        val flywheelSpeedStep = 25.0

        // Toggle debounce flags (GP2)
        var lastA2 = false
        var lastY2 = false
        var lastDpadUp2 = false
        var lastDpadLeft2 = false
        var lastDpadRight2 = false

        telemetry.addLine("Main TeleOp initialized. Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // ============================================================
            // GAMEPAD 1: Driver (driving ONLY) + Debug/Tuning
            // ============================================================

            // Drivetrain: left stick = translate, right stick x = rotate, dpad = speed mode
            robot.drive.update(gamepad1, io)

            // ============================================================
            // GAMEPAD 2: Operator (ALL subsystem controls)
            // ============================================================

            // --- Flywheel speed adjustment (D-pad left/right) ---
            if (gamepad2.dpad_left && !lastDpadLeft2) {
                flywheelTargetSpeed = (flywheelTargetSpeed - flywheelSpeedStep).coerceAtLeast(0.0)
            }
            lastDpadLeft2 = gamepad2.dpad_left

            if (gamepad2.dpad_right && !lastDpadRight2) {
                flywheelTargetSpeed = (flywheelTargetSpeed + flywheelSpeedStep).coerceAtMost(628.0)
            }
            lastDpadRight2 = gamepad2.dpad_right

            // --- Intake: hold left trigger ---
            if (gamepad2.left_trigger > 0.1 && gamepad2.right_trigger <= 0.1) {
                robot.intakeCoordinator.startIntake()
            } else if (!gamepad2.b) {
                if (robot.intakeTransfer.state == IntakeTransfer.State.INTAKING ||
                    robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
            }

            // --- Outtake: hold B to reverse intake ---
            if (gamepad2.b) {
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

            // --- Turret control ---
            val manualTurretInput = gamepad2.right_stick_x.toDouble()
            if (abs(manualTurretInput) > 0.05) {
                // Manual override: robot-relative turret control
                robot.aimTurret = false
                robot.turret.fieldRelativeMode = false
                robot.turret.targetAngle += manualTurretInput * 2.0 * dt.inWholeMilliseconds / 1000.0
                robot.turret.targetAngle = robot.turret.targetAngle.coerceIn(-PI, PI)
            } else if (autoAimEnabled) {
                robot.aimTurret = true
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

//            robot.io.flywheel = if (gamepad2.right_trigger > 0.1) { 0.88 } else { 0.0 }

            // --- Shooting (right trigger) or flywheel spin-up (left bumper) ---
            if (gamepad2.right_trigger > 0.1) {
                // Shooting: spin flywheel + transfer (blocker delay handled by state machine)
                robot.shooter.flywheelTarget = flywheelTargetSpeed
                robot.aimFlywheel = false
                robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
            } else if (gamepad2.left_bumper) {
                // Spin-up only: flywheel on, blocker stays engaged
                robot.shooter.flywheelTarget = flywheelTargetSpeed
                robot.aimFlywheel = false
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            } else {
                // Idle: stop flywheel, only reset transfer (not intake/reverse)
                robot.shooter.flywheelTarget = 0.0
                robot.aimFlywheel = false
                if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
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
            telemetry.addData("Turret Servo", "%.3f", robot.turret.currentServoPosition)
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
            telemetry.addData("Speed Mode", if (robot.drive.getSpeedMultiplier() == 1.0) "FULL" else "PRECISION")
            telemetry.update()

            false // continue loop
        }

        robot.close()
    }
}
