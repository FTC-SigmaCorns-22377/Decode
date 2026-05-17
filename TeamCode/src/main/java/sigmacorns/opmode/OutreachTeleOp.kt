package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.logic.IntakeCoordinator
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.PI
import kotlin.math.abs

/**
 * Outreach TeleOp: manual control only, no field-dependent automation.
 *
 * GAMEPAD 1 (Driver):
 *   Left stick          - Translate (mecanum drive)
 *   Right stick X       - Rotate
 *   Left trigger        - Intake
 *   Right trigger       - Shoot (flywheel + transfer)
 *   Right bumper        - Spin flywheel (no transfer)
 *   B                   - Reverse intake
 *
 * GAMEPAD 2 (Override):
 *   X                   - Toggle GP2 takeover (drive + intake + shoot)
 *   Left bumper         - Toggle speed limit
 *   D-pad left/right    - Flywheel target speed adjust
 *   Left stick X        - Turret manual (when GP2 takeover OFF)
 *   Right stick Y       - Hood manual (when GP2 takeover OFF)
 */
@TeleOp(name = "Outreach TeleOp", group = "Outreach")
class OutreachTeleOp : SigmaOpMode() {
    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = false)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = false)

        // Manual-only settings
        robot.drive.fieldCentric = false
        robot.aimTurret = false
        robot.aimFlywheel = false
        robot.shooter.autoAdjust = false
        robot.turret.fieldRelativeMode = false

        val speedLimitMultiplier = 0.4
        val flywheelSpeedStep = 25.0

        var flywheelTargetSpeed = 363.0
        var turretTargetAngle = 0.0
        var hoodAngle = Math.toRadians(40.0)

        var gp2Takeover = false
        var speedLimitEnabled = false

        var lastX2 = false
        var lastLB2 = false
        var lastDpadLeft2 = false
        var lastDpadRight2 = false

        telemetry.addLine("Outreach TeleOp initialized. Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { _, dt ->
            // ============================================================
            // GAMEPAD 2: Overrides
            // ============================================================
            if (gamepad2.x && !lastX2) {
                gp2Takeover = !gp2Takeover
            }
            lastX2 = gamepad2.x

            if (gamepad2.left_bumper && !lastLB2) {
                speedLimitEnabled = !speedLimitEnabled
            }
            lastLB2 = gamepad2.left_bumper

            if (gamepad2.dpad_left && !lastDpadLeft2) {
                flywheelTargetSpeed = (flywheelTargetSpeed - flywheelSpeedStep).coerceAtLeast(0.0)
            }
            lastDpadLeft2 = gamepad2.dpad_left

            if (gamepad2.dpad_right && !lastDpadRight2) {
                flywheelTargetSpeed = (flywheelTargetSpeed + flywheelSpeedStep).coerceAtMost(628.0)
            }
            lastDpadRight2 = gamepad2.dpad_right

            // ============================================================
            // Driving
            // ============================================================
            val drivePad = if (gp2Takeover) gamepad2 else gamepad1
            val driveScale = if (speedLimitEnabled) speedLimitMultiplier else 1.0
            val inputX = -drivePad.left_stick_y.toDouble() * driveScale
            val inputY = -drivePad.left_stick_x.toDouble() * driveScale
            val inputRot = -drivePad.right_stick_x.toDouble() * driveScale
            robot.drive.drive(Pose2d(inputX, inputY, inputRot), io)

            // ============================================================
            // Manual turret + hood (only when GP2 takeover is OFF)
            // ============================================================
            if (!gp2Takeover) {
                val manualTurretInput = gamepad2.left_stick_x.toDouble()
                if (abs(manualTurretInput) > 0.05) {
                    turretTargetAngle += manualTurretInput * 2.0 * dt.inWholeMilliseconds / 1000.0
                    turretTargetAngle = turretTargetAngle.coerceIn(-PI, PI)
                }

                val manualHoodInput = -gamepad2.right_stick_y.toDouble()
                if (abs(manualHoodInput) > 0.05) {
                    hoodAngle += manualHoodInput * Math.toRadians(30.0) * dt.inWholeMilliseconds / 1000.0
                    hoodAngle = hoodAngle.coerceIn(
                        Math.toRadians(ShooterConfig.minAngleDeg),
                        Math.toRadians(ShooterConfig.maxAngleDeg)
                    )
                }
            }

            robot.turret.fieldRelativeMode = false
            robot.turret.targetAngle = turretTargetAngle
            robot.shooter.autoAdjust = false
            robot.shooter.manualHoodAngle = hoodAngle

            // ============================================================
            // Intake / Shoot (primary gamepad only)
            // ============================================================
            val intakeTriggered = drivePad.left_trigger > 0.1
            val shootTriggered = drivePad.right_trigger > 0.1

            if (intakeTriggered && !shootTriggered) {
                robot.intakeCoordinator.startIntake()
            } else if (!drivePad.b) {
                if (robot.intakeTransfer.state == IntakeTransfer.State.INTAKING ||
                    robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
            }

            if (drivePad.b) {
                robot.intakeTransfer.state = IntakeTransfer.State.REVERSING
            } else if (robot.intakeTransfer.state == IntakeTransfer.State.REVERSING) {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }

            val flywheelVel = io.flywheelVelocity()

            if (shootTriggered) {
                robot.shooter.flywheelTarget = flywheelTargetSpeed
                val shooterAtSpeed = flywheelTargetSpeed > 0.0 &&
                    abs(flywheelVel - flywheelTargetSpeed) < IntakeCoordinator.FLYWHEEL_MIN_VEL
                if (shooterAtSpeed && robot.intakeTransfer.state != IntakeTransfer.State.REVERSING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                } else if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
            } else if (drivePad.right_bumper) {
                robot.shooter.flywheelTarget = flywheelTargetSpeed
            } else {
                robot.shooter.flywheelTarget = 0.0
                if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
            }

            robot.update()
            io.update()
            val flywheelRPM = flywheelVel * 60.0 / (2.0 * PI)

            telemetry.addLine("=== OUTREACH MODE ===")
            telemetry.addData("GP2 Takeover", if (gp2Takeover) "ON" else "OFF")
            telemetry.addData("Speed Limit", if (speedLimitEnabled) "ON" else "OFF")
            telemetry.addData("Drive Scale", "%.0f %%", driveScale * 100.0)

            telemetry.addLine("")
            telemetry.addLine("=== CONTROLS ===")
            telemetry.addData("Drive", "GP${if (gp2Takeover) 2 else 1}")
            telemetry.addData("Flywheel Target", "%.1f rad/s (GP2 D-pad L/R)", flywheelTargetSpeed)
            telemetry.addData("Hood Angle", "%.1f deg", Math.toDegrees(hoodAngle))
            telemetry.addData("Turret Target", "%.1f deg", Math.toDegrees(turretTargetAngle))

            telemetry.addLine("")
            telemetry.addLine("=== SHOOTER ===")
            telemetry.addData("Flywheel RPM", "%.0f", flywheelRPM)
            telemetry.addData("Flywheel Vel", "%.1f rad/s", flywheelVel)

            telemetry.addLine("")
            telemetry.addLine("=== BALLS / INTAKE ===")
            telemetry.addData("Balls Held", robot.beamBreak.ballCount)
            telemetry.addData("Intake State", when (robot.intakeTransfer.state) {
                IntakeTransfer.State.INTAKING -> "INTAKING"
                IntakeTransfer.State.REVERSING -> "REVERSING"
                IntakeTransfer.State.TRANSFERRING -> "SHOOTING"
                else -> "IDLE"
            })

            telemetry.addLine("")
            telemetry.addData("Loop Time", "%d ms", dt.inWholeMilliseconds)
            telemetry.update()

            false
        }

        robot.close()
    }
}
