package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.logic.AutomationManager
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.PI
import kotlin.math.abs

@TeleOp(name = "New Controls Blue", group = "Competition")
class NewControlsTeleOpBlue : SigmaOpMode() {
    override fun runOpMode() {
        val robot = Robot(io, blue = true, useNativeAim = true)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = !SIM)
        robot.startApriltag()

        val automationManager = AutomationManager(robot)

        robot.intakeCoordinator.autoShootEnabled = true
        robot.aimTurret = true
        robot.aimFlywheel = true

        var autoAimEnabled = true
        var flywheelTargetSpeed = 400.0
        val flywheelSpeedStep = 25.0
        var flywheelAlwaysOn = true
        var gp2DriveEnabled = false

        val turretOffsetStep = Math.toRadians(0.5)
        val hoodOffsetStep = Math.toRadians(0.25)

        var lastX1 = false
        var lastRB1 = false
        var lastLB1 = false

        var lastA2 = false
        var lastY2 = false
        var lastX2 = false
        var lastRightBumper2 = false
        var lastBack2 = false
        var lastStart2 = false
        var lastLeftStickButton2 = false
        var lastRightStickButton2 = false
        var lastDpadUp2 = false
        var lastDpadDown2 = false
        var lastDpadLeft2 = false
        var lastDpadRight2 = false

        robot.drive.fieldCentric = false

        telemetry.addLine("New Controls TeleOp Blue initialized. Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            if (gamepad1.x && !lastX1) robot.drive.fieldCentric = !robot.drive.fieldCentric
            lastX1 = gamepad1.x

            if (gamepad1.right_bumper && !lastRB1) {
                automationManager.shootFarZone()
            } else if (gamepad1.left_bumper && !lastLB1) {
                automationManager.shootGoalZone()
            }
            lastRB1 = gamepad1.right_bumper
            lastLB1 = gamepad1.left_bumper

            if (gp2DriveEnabled) {
                if (!automationManager.update(gamepad2)) {
                    robot.drive.fieldCentricHeading = robot.aim.autoAim.fusedPose.rot - 0.5 * PI
                    robot.drive.update(gamepad2, io)
                }
            } else {
                if (!automationManager.update(gamepad1)) {
                    robot.drive.fieldCentricHeading = robot.aim.autoAim.fusedPose.rot - 0.5 * PI
                    robot.drive.update(gamepad1, io)
                }
            }

            if (gamepad2.x && !lastX2) {
                gp2DriveEnabled = !gp2DriveEnabled
            }
            lastX2 = gamepad2.x

            if (gamepad2.right_bumper && !lastRightBumper2) {
                flywheelAlwaysOn = !flywheelAlwaysOn
            }
            lastRightBumper2 = gamepad2.right_bumper

            if (gamepad2.dpad_up && !lastDpadUp2) {
                flywheelTargetSpeed = (flywheelTargetSpeed + flywheelSpeedStep).coerceAtMost(628.0)
            }
            lastDpadUp2 = gamepad2.dpad_up

            if (gamepad2.dpad_down && !lastDpadDown2) {
                flywheelTargetSpeed = (flywheelTargetSpeed - flywheelSpeedStep).coerceAtLeast(0.0)
            }
            lastDpadDown2 = gamepad2.dpad_down

            if (gamepad2.dpad_left && !lastDpadLeft2) {
                robot.turret.manualOffset -= turretOffsetStep
            }
            lastDpadLeft2 = gamepad2.dpad_left

            if (gamepad2.dpad_right && !lastDpadRight2) {
                robot.turret.manualOffset += turretOffsetStep
            }
            lastDpadRight2 = gamepad2.dpad_right

            if (gamepad2.left_stick_button && !lastLeftStickButton2) {
                robot.shooter.manualOffset -= hoodOffsetStep
            }
            lastLeftStickButton2 = gamepad2.left_stick_button

            if (gamepad2.right_stick_button && !lastRightStickButton2) {
                robot.shooter.manualOffset += hoodOffsetStep
            }
            lastRightStickButton2 = gamepad2.right_stick_button

            if (gamepad2.start && !lastStart2) {
                robot.turret.manualOffset = 0.0
                robot.shooter.manualOffset = 0.0
            }
            lastStart2 = gamepad2.start

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

            if (gamepad1.b || gamepad2.b) {
                robot.intakeTransfer.state = IntakeTransfer.State.REVERSING
            } else if (robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }

            if (gamepad2.a && !lastA2) {
                autoAimEnabled = !autoAimEnabled
                if (!autoAimEnabled) {
                    flywheelTargetSpeed = 363.0
                    robot.shooter.flywheelTarget = flywheelTargetSpeed
                    robot.shooter.autoAdjust = false
                    robot.shooter.manualHoodAngle = Math.toRadians(40.0)
                    robot.aimTurret = false
                } else {
                    robot.aimTurret = true
                    robot.shooter.autoAdjust = true
                }
            }
            lastA2 = gamepad2.a

            if (gamepad2.y && !lastY2) {
                robot.intakeCoordinator.autoShootEnabled = !robot.intakeCoordinator.autoShootEnabled
            }
            lastY2 = gamepad2.y

            if (gamepad2.back && !lastBack2) {
                robot.shooter.autoAdjust = !robot.shooter.autoAdjust
            }
            lastBack2 = gamepad2.back

            if (!autoAimEnabled) {
                val manualTurretInput = gamepad2.left_stick_x.toDouble()
                if (abs(manualTurretInput) > 0.05) {
                    robot.aimTurret = false
                    robot.turret.fieldRelativeMode = false
                    robot.turret.targetAngle += manualTurretInput * 2.0 * dt.inWholeMilliseconds / 1000.0
                    robot.turret.targetAngle = robot.turret.targetAngle.coerceIn(-PI, PI)
                }
            }

            if (!autoAimEnabled) {
                val manualHoodInput = -gamepad2.right_stick_y.toDouble()
                if (abs(manualHoodInput) > 0.05) {
                    robot.shooter.autoAdjust = false
                    robot.shooter.manualHoodAngle += manualHoodInput * Math.toRadians(30.0) * dt.inWholeMilliseconds / 1000.0
                    robot.shooter.manualHoodAngle = robot.shooter.manualHoodAngle.coerceIn(
                        Math.toRadians(ShooterConfig.minAngleDeg),
                        Math.toRadians(ShooterConfig.maxAngleDeg)
                    )
                }
            }

            if (shootTriggered) {
                if (autoAimEnabled) {
                    robot.aimFlywheel = true
                    robot.aim.shotRequested = true
                } else {
                    robot.shooter.flywheelTarget = flywheelTargetSpeed
                    robot.aimFlywheel = false
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                }
            } else if (gamepad2.left_bumper) {
                robot.shooter.flywheelTarget = flywheelTargetSpeed
                robot.aimFlywheel = false
                robot.aim.shotRequested = false
            } else {
                val autoZoneShotActive = autoAimEnabled && robot.intakeCoordinator.autoShootEnabled
                if (autoAimEnabled) {
                    robot.aimFlywheel = true
                    robot.shooter.flywheelTarget = if (flywheelAlwaysOn) 240.0 else 0.0
                } else {
                    robot.aimFlywheel = false
                    robot.shooter.flywheelTarget = 363.0
                }
                if (!autoZoneShotActive) {
                    robot.aim.shotRequested = false
                }
                if (!autoZoneShotActive && robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
            }

            robot.update()
            io.update()

            val fusedPose = robot.aim.autoAim.fusedPose
            val flywheelVel = io.flywheelVelocity()
            val flywheelRPM = flywheelVel * 60.0 / (2.0 * PI)

            telemetry.addLine("=== BALL STATUS ===")
            telemetry.addData("Balls Held", robot.beamBreak.ballCount)
            telemetry.addData("Beam 1|2|3", robot.beamBreak.slots.map { if (it) "O" else "-" })

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
            telemetry.addData("Flywheel Target", "%.1f rad/s (GP2 D-pad U/D)", robot.shooter.flywheelTarget)
            telemetry.addData("Hood Angle", "%.1f°", Math.toDegrees(robot.shooter.computedHoodAngle))
            telemetry.addData("Hood", if (robot.shooter.autoAdjust) "AUTO" else "MANUAL")
            telemetry.addData("Hood Offset", "%.2f° (GP2 L3/R3, Start=reset)", Math.toDegrees(robot.shooter.manualOffset))
            telemetry.addData("Hood Servo", "%.3f", robot.shooter.hoodServoPosition)

            telemetry.addLine("")
            telemetry.addLine("=== TURRET ===")
            telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(robot.turret.pos))
            telemetry.addData("Turret Target", "%.1f°", Math.toDegrees(robot.turret.effectiveTargetAngle))
            telemetry.addData("Turret Offset", "%.2f° (GP2 D-pad L/R, Start=reset)", Math.toDegrees(robot.turret.manualOffset))
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

            false
        }

        robot.close()
    }
}
