package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.HoodConfig
import kotlin.math.PI
import kotlin.math.abs

/**
 * Full-featured competition TeleOp.
 *
 * GAMEPAD 1 (Driver + Debug/Tuning):
 *   Left stick          - Translate (mecanum drive)
 *   Right stick X       - Rotate
 *   D-pad up/down       - Speed mode toggle (full / precision)
 *   Left bumper         - Decrease flywheel target speed (-25 rad/s)
 *   Right bumper        - Increase flywheel target speed (+25 rad/s)
 *
 * GAMEPAD 2 (Operator - all subsystem controls):
 *   Left trigger         - Hold to intake balls
 *   B                    - Hold to reverse intake (spit balls out)
 *   Right trigger        - Hold to shoot (flywheel + transfer)
 *   Left trigger + RT    - Hold left trigger to spin-up flywheel only (if RT not held)
 *   Left bumper          - Hold to spin up flywheel (without shooting)
 *   A                    - Toggle auto-aim on/off
 *   Right stick X        - Manual turret override (disables auto-aim while held)
 *   Right stick Y        - Manual hood angle override
 *   X                    - Toggle shoot-while-move
 *   Y                    - Toggle auto-shoot (zone-based)
 *   D-pad up             - Toggle hood auto-adjust
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
class MainTeleOp : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false)
        robot.init(Pose2d(), apriltagTracking = true)
        robot.startApriltag()

        // State
        var autoAimEnabled = true
        var shootWhileMoveEnabled = false
        var flywheelTargetSpeed = 400.0
        val flywheelSpeedStep = 25.0

        // Coroutine job tracking for async transfer
        var transferJob: Job? = null

        // Toggle debounce flags (GP2)
        var lastA2 = false
        var lastX2 = false
        var lastY2 = false
        var lastDpadUp2 = false
        // Debounce flags (GP1 - debug/tuning)
        var lastLB1 = false
        var lastRB1 = false

        telemetry.addLine("Main TeleOp initialized. Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // ============================================================
            // GAMEPAD 1: Driver (driving ONLY) + Debug/Tuning
            // ============================================================

            // Drivetrain: left stick = translate, right stick x = rotate, dpad = speed mode
            robot.drive.update(gamepad1, io)

            // Debug: flywheel speed adjustment (bumpers)
            if (gamepad1.left_bumper && !lastLB1) {
                flywheelTargetSpeed = (flywheelTargetSpeed - flywheelSpeedStep).coerceAtLeast(0.0)
            }
            lastLB1 = gamepad1.left_bumper

            if (gamepad1.right_bumper && !lastRB1) {
                flywheelTargetSpeed = (flywheelTargetSpeed + flywheelSpeedStep).coerceAtMost(628.0)
            }
            lastRB1 = gamepad1.right_bumper

            // ============================================================
            // GAMEPAD 2: Operator (ALL subsystem controls)
            // ============================================================

            // --- Intake: hold left trigger ---
            if (gamepad2.left_trigger > 0.1 && gamepad2.right_trigger <= 0.1) {
                robot.intake.startIntake()
            } else if (!gamepad2.b) {
                robot.intake.stopIntake()
            }

            // --- Outtake: hold B to reverse intake ---
            if (gamepad2.b) {
                robot.intake.startReverse()
            } else {
                robot.intake.stopReverse()
            }

            // --- Toggle: Auto-aim (A button) ---
            if (gamepad2.a && !lastA2) {
                autoAimEnabled = !autoAimEnabled
            }
            lastA2 = gamepad2.a

            // --- Toggle: Shoot-while-move (X button) ---
            if (gamepad2.x && !lastX2) {
                shootWhileMoveEnabled = !shootWhileMoveEnabled
            }
            lastX2 = gamepad2.x

            // --- Toggle: Auto-shoot zones (Y button) ---
            if (gamepad2.y && !lastY2) {
                robot.transfer.autoShoot = !robot.transfer.autoShoot
            }
            lastY2 = gamepad2.y

            // --- Toggle: Hood auto-adjust (D-pad up) ---
            if (gamepad2.dpad_up && !lastDpadUp2) {
                robot.hood.autoAdjust = !robot.hood.autoAdjust
            }
            lastDpadUp2 = gamepad2.dpad_up

            // --- Turret control ---
            val manualTurretInput = gamepad2.right_stick_x.toDouble()
            if (abs(manualTurretInput) > 0.05) {
                // Manual override: robot-relative turret control
                robot.aimTurret = false
                robot.turret.fieldRelativeMode = false
                robot.turret.targetAngle += manualTurretInput * 2.0 * dt.inWholeMilliseconds / 1000.0
                robot.turret.targetAngle = robot.turret.targetAngle.coerceIn(-PI / 2.0, PI / 2.0)
            } else if (autoAimEnabled) {
                robot.aimTurret = true
            }

            // --- Hood manual override (right stick Y) ---
            val manualHoodInput = -gamepad2.right_stick_y.toDouble()
            if (abs(manualHoodInput) > 0.05) {
                robot.hood.autoAdjust = false
                robot.hood.manualAngle += manualHoodInput * Math.toRadians(30.0) * dt.inWholeMilliseconds / 1000.0
                robot.hood.manualAngle = robot.hood.manualAngle.coerceIn(
                    Math.toRadians(HoodConfig.minAngleDeg),
                    Math.toRadians(HoodConfig.maxAngleDeg)
                )
            }

            // --- Shooting (right trigger) or flywheel spin-up (left bumper) ---
            if (gamepad2.right_trigger > 0.1) {
                // Shooting: spin flywheel + async transfer (with blocker delay)
                robot.flywheel.target = flywheelTargetSpeed
                robot.aimFlywheel = true
                if (transferJob == null || transferJob!!.isCompleted) {
                    transferJob = robot.scope.launch { robot.transfer.startTransfer() }
                }
            } else if (gamepad2.left_bumper) {
                // Spin-up only: flywheel on, blocker stays engaged
                robot.flywheel.target = flywheelTargetSpeed
                robot.aimFlywheel = true
                if (robot.transfer.isRunning) {
                    transferJob = robot.scope.launch { robot.transfer.stopTransfer() }
                }
            } else {
                // Idle: stop flywheel and transfer
                robot.flywheel.target = 0.0
                robot.aimFlywheel = true
                if (robot.transfer.isRunning) {
                    transferJob = robot.scope.launch { robot.transfer.stopTransfer() }
                }
            }

            // --- Shoot-while-move ---
            if (shootWhileMoveEnabled && autoAimEnabled) {
                robot.aim.updateVelocityComponents()
                robot.turret.targetAngleOffset = robot.aim.turretLeadAngle
            } else {
                robot.turret.targetAngleOffset = 0.0
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
            telemetry.addData("Slots", robot.beamBreak.slots.map { if (it == true) "BALL" else "---" })
            telemetry.addData("Beam 1|2|3", "${io.beamBreak1()}|${io.beamBreak2()}|${io.beamBreak3()}")

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
            telemetry.addData("Flywheel Target", "%.1f rad/s (GP1 bumpers to adj)", robot.flywheel.target)
            telemetry.addData("Hood Angle", "%.1f°", Math.toDegrees(robot.hood.computedAngle))
            telemetry.addData("Hood", "${if (robot.hood.autoAdjust) "AUTO" else "MANUAL"} ${if (robot.hood.usingInterpolatedData) "(data)" else "(trig)"}")
            telemetry.addData("Hood Servo", "%.3f", robot.hood.currentServoPosition)

            telemetry.addLine("")
            telemetry.addLine("=== TURRET ===")
            telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(robot.turret.pos))
            telemetry.addData("Turret Target", "%.1f°", Math.toDegrees(robot.turret.effectiveTargetAngle))
            telemetry.addData("Turret Servo", "%.3f", robot.turret.currentServoPosition)
            telemetry.addData("Field-Relative", robot.turret.fieldRelativeMode)
            telemetry.addData("Shoot-While-Move", if (shootWhileMoveEnabled) "ON" else "OFF")

            telemetry.addLine("")
            telemetry.addLine("=== SUBSYSTEMS ===")
            val intake1Vel = io.intake1Velocity()
            telemetry.addData("Intake", when {
                robot.intake.isReversing -> "REVERSING"
                robot.intake.isRunning -> "RUNNING"
                else -> "IDLE"
            })
            telemetry.addData("Intake1 RPM", "%.0f", intake1Vel)
            telemetry.addData("Transfer", if (robot.transfer.isRunning) "RUNNING" else "IDLE")
            telemetry.addData("Blocker", if (io.blocker == 0.0) "ENGAGED" else "DISENGAGED")
            telemetry.addData("Auto-Shoot", if (robot.transfer.autoShoot) "ON" else "OFF")
            telemetry.addData("Speed Mode", if (robot.drive.getSpeedMultiplier() == 1.0) "FULL" else "PRECISION")
            telemetry.update()

            false // continue loop
        }

        robot.close()
    }
}
