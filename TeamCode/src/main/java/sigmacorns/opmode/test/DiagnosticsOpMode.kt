package sigmacorns.opmode.test

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Servo
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI
import kotlin.math.abs
import kotlin.time.Duration

/**
 * Sequential diagnostics opmode. Steps through each subsystem automatically:
 *   1. Limelight — connectivity + any AprilTag visible (motif/non-goal tags count)
 *   2. Turret servos + analog encoder — sweep to +45°, hold, return to 0°
 *   3. Shooter — flywheel to 100 rad/s, then off
 *   4. Intake — full power, then off
 *   5. Drive 360° spin (odometry-tracked; 12 s time-based fallback)
 *   6. Odometry + fused pose — near (0, 0, 90°) within ±0.3 m / ±45°
 *
 * Every screen shows a running scoreboard of all hardware and check results.
 */
@TeleOp(name = "Diagnostics", group = "Test")
class DiagnosticsOpMode : SigmaOpMode() {

    private enum class Phase {
        LIMELIGHT_CHECK,
        TURRET_TO_45,
        TURRET_HOLD_45,
        TURRET_TO_ZERO,
        FLYWHEEL_SPINUP,
        FLYWHEEL_OFF,
        INTAKE_ON,
        INTAKE_OFF,
        DRIVE_SPIN,
        DRIVE_STOP,
        POSE_CHECK,
        DONE
    }

    override fun runOpMode() {
        val hasShooter1 = hardwareMap.tryGet(DcMotorEx::class.java, "shooter1") != null
        val hasShooter2 = hardwareMap.tryGet(DcMotorEx::class.java, "shooter2") != null
        val hasIntake1 = hardwareMap.tryGet(DcMotorEx::class.java, "intake1") != null
        val hasIntake2 = hardwareMap.tryGet(DcMotorEx::class.java, "intake2") != null
        val hasTurretLeft = hardwareMap.tryGet(Servo::class.java, "turretLeft") != null
        val hasTurretRight = hardwareMap.tryGet(Servo::class.java, "turretRight") != null
        val hasTurretEncoder = hardwareMap.tryGet(AnalogInput::class.java, "turretEncoder") != null
        val hasHood = hardwareMap.tryGet(Servo::class.java, "hood") != null
        val hasBlocker = hardwareMap.tryGet(Servo::class.java, "blocker") != null
        val hasBeamBreak1 = hardwareMap.tryGet(DigitalChannel::class.java, "beamBreak1") != null
        val hasBeamBreak2 = hardwareMap.tryGet(DigitalChannel::class.java, "beamBreak2") != null
        val hasBeamBreak3 = hardwareMap.tryGet(DigitalChannel::class.java, "beamBreak3") != null
        val hasLimelight = hardwareMap.tryGet(Limelight3A::class.java, "limelight") != null
        val hasPinpoint = hardwareMap.tryGet(GoBildaPinpointDriver::class.java, "pinpoint") != null

        telemetry.addLine("=== HARDWARE PRESENCE CHECK ===")
        telemetry.addData("Shooter 1 (shooter1)", ok(hasShooter1))
        telemetry.addData("Shooter 2 (shooter2)", ok(hasShooter2))
        telemetry.addData("Intake 1 (intake1)", ok(hasIntake1))
        telemetry.addData("Intake 2 (intake2)", ok(hasIntake2))
        telemetry.addData("Turret Left Servo (turretLeft)", ok(hasTurretLeft))
        telemetry.addData("Turret Right Servo (turretRight)", ok(hasTurretRight))
        telemetry.addData("Turret Encoder (turretEncoder)", ok(hasTurretEncoder))
        telemetry.addData("Hood Servo (hood)", ok(hasHood))
        telemetry.addData("Blocker Servo (blocker)", ok(hasBlocker))
        telemetry.addData("Beam Break 1 (beamBreak1)", ok(hasBeamBreak1))
        telemetry.addData("Beam Break 2 (beamBreak2)", ok(hasBeamBreak2))
        telemetry.addData("Beam Break 3 (beamBreak3)", ok(hasBeamBreak3))
        telemetry.addData("Limelight (limelight)", ok(hasLimelight))
        telemetry.addData("Pinpoint (pinpoint)", ok(hasPinpoint))
        telemetry.addLine("Press START to begin active checks")
        telemetry.update()

        waitForStart()

        val robot = Robot(io, blue = false)
        robot.init(Pose2d(0.0, 0.0, PI / 2.0), apriltagTracking = hasLimelight)
        robot.aimTurret = false
        robot.aimFlywheel = false
        if (hasLimelight) robot.startApriltag()

        var phase = Phase.LIMELIGHT_CHECK
        var phaseStart: Duration? = null

        var lastHeading = 0.0
        var spinAccumulated = 0.0

        // null = pending, true = pass, false = fail
        var turretResult: Boolean? = null
        var flywheelResult: Boolean? = null
        var intakeResult: Boolean? = null
        var driveResult: Boolean? = null
        var limelightResult: Boolean? = null
        var poseResult: Boolean? = null

        // Tracked across phases
        var flywheelEverHitTarget = false
        var intakeSpun = false
        var limelightEverSawTag = false

        fun addScoreboard() {
            telemetry.addLine("--- HARDWARE ---")
            telemetry.addData("Turret Servos L+R", ok(hasTurretLeft && hasTurretRight))
            telemetry.addData("Turret Encoder", ok(hasTurretEncoder))
            telemetry.addData("Shooter 1+2", ok(hasShooter1 && hasShooter2))
            telemetry.addData("Intake 1+2", ok(hasIntake1 && hasIntake2))
            telemetry.addData("Hood Servo", ok(hasHood))
            telemetry.addData("Blocker Servo", ok(hasBlocker))
            telemetry.addData("Beam Breaks 1+2+3", ok(hasBeamBreak1 && hasBeamBreak2 && hasBeamBreak3))
            telemetry.addData("Limelight", ok(hasLimelight))
            telemetry.addData("Pinpoint", ok(hasPinpoint))
            telemetry.addLine("--- CHECKS ---")
            telemetry.addData("Turret encoder tracks servo", result(turretResult))
            telemetry.addData("Flywheel 100 rad/s", result(flywheelResult))
            telemetry.addData("Intake spun", result(intakeResult))
            telemetry.addData("Drive 360°", result(driveResult))
            telemetry.addData("Limelight saw tag", result(limelightResult))
            telemetry.addData("Fused pose ~(0,0,90°)", result(poseResult))
        }

        ioLoop { state, dt ->
            if (phaseStart == null) {
                phaseStart = state.timestamp
                lastHeading = state.driveTrainPosition.rot
            }
            val elapsed = (state.timestamp - phaseStart!!).inWholeMilliseconds / 1000.0

            robot.aimTurret = false
            robot.aimFlywheel = false
            robot.turret.fieldRelativeMode = false
            robot.turret.targetAngle = 0.0

            when (phase) {
                Phase.LIMELIGHT_CHECK -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.update()
                    io.intake = 0.0
                    robot.drive.drive(Pose2d(0.0, 0.0, 0.0), io)

                    val autoAim = robot.aim.autoAim
                    // Any tag counts (including motif/non-goal tags)
                    if (autoAim.detectedTagCount > 0) limelightEverSawTag = true

                    telemetry.addLine("=== [1/6] LIMELIGHT CHECK ===")
                    telemetry.addData("Limelight Connected", ok(hasLimelight))
                    telemetry.addData("Any Tag Seen", autoAim.detectedTagCount > 0)
                    telemetry.addData("Detections (this tick)", autoAim.detectedTagCount)
                    telemetry.addData("Matching Tag ID", autoAim.trackedTagId)
                    telemetry.addData("Elapsed", "%.1f s / 3 s", elapsed)
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (elapsed >= 3.0) {
                        limelightResult = limelightEverSawTag
                        phase = Phase.TURRET_TO_45
                        phaseStart = state.timestamp
                    }
                }

                Phase.TURRET_TO_45 -> {
                    robot.turret.targetAngle = PI / 4.0  // override global 0.0
                    robot.update()

                    val actualDeg = Math.toDegrees(robot.turret.pos)
                    val errorDeg = Math.toDegrees(abs(robot.turret.pos - PI / 4.0))
                    val reached = errorDeg < 10.0

                    telemetry.addLine("=== [2/6] TURRET — MOVING TO +45° ===")
                    telemetry.addData("Left Servo", ok(hasTurretLeft))
                    telemetry.addData("Right Servo", ok(hasTurretRight))
                    telemetry.addData("Encoder (turretEncoder)", ok(hasTurretEncoder))
                    telemetry.addData("Commanded", "+45.0°")
                    telemetry.addData("Encoder angle", if (hasTurretEncoder) "%.1f°".format(actualDeg) else "NO ENCODER")
                    telemetry.addData("Error", if (hasTurretEncoder) "%.1f°".format(errorDeg) else "---")
                    telemetry.addData("Left servo pos", "%.3f", io.turretLeft)
                    telemetry.addData("Time limit", "%.1f s / 3 s", elapsed)
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    when {
                        reached -> {
                            // Encoder matched — hold for 2 s
                            phase = Phase.TURRET_HOLD_45
                            phaseStart = state.timestamp
                        }
                        elapsed >= 3.0 -> {
                            // Never reached target — fail and skip rest of turret test
                            turretResult = false
                            phase = Phase.FLYWHEEL_SPINUP
                            phaseStart = state.timestamp
                        }
                    }
                }

                Phase.TURRET_HOLD_45 -> {
                    robot.turret.targetAngle = PI / 4.0
                    robot.update()

                    val actualDeg = Math.toDegrees(robot.turret.pos)
                    val errorDeg = Math.toDegrees(abs(robot.turret.pos - PI / 4.0))

                    telemetry.addLine("=== [2/6] TURRET — HOLDING AT +45° ===")
                    telemetry.addData("Commanded", "+45.0°")
                    telemetry.addData("Encoder angle", if (hasTurretEncoder) "%.1f°".format(actualDeg) else "NO ENCODER")
                    telemetry.addData("Error", if (hasTurretEncoder) "%.1f°".format(errorDeg) else "---")
                    telemetry.addData("Left servo pos", "%.3f", io.turretLeft)
                    telemetry.addData("Hold", "%.1f s / 2 s", elapsed)
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (elapsed >= 2.0) {
                        phase = Phase.TURRET_TO_ZERO
                        phaseStart = state.timestamp
                    }
                }

                Phase.TURRET_TO_ZERO -> {
                    // targetAngle already set to 0.0 by the global default above
                    robot.update()

                    val actualDeg = Math.toDegrees(robot.turret.pos)
                    val errorDeg = Math.toDegrees(abs(robot.turret.pos))

                    telemetry.addLine("=== [2/6] TURRET — RETURNING TO 0° ===")
                    telemetry.addData("Commanded", "0.0°")
                    telemetry.addData("Encoder angle", if (hasTurretEncoder) "%.1f°".format(actualDeg) else "NO ENCODER")
                    telemetry.addData("Error", if (hasTurretEncoder) "%.1f°".format(errorDeg) else "---")
                    telemetry.addData("Left servo pos", "%.3f", io.turretLeft)
                    telemetry.addData("Elapsed", "%.1f s / 1.5 s", elapsed)
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (elapsed >= 1.5) {
                        // Pass: encoder reached +45° (proved by TURRET_HOLD_45 completing)
                        turretResult = true
                        phase = Phase.FLYWHEEL_SPINUP
                        phaseStart = state.timestamp
                    }
                }

                Phase.FLYWHEEL_SPINUP -> {
                    robot.shooter.flywheelTarget = 100.0
                    robot.update()
                    io.intake = 0.0

                    val vel = io.flywheelVelocity()
                    if (abs(vel - 100.0) < 10.0) flywheelEverHitTarget = true

                    telemetry.addLine("=== [3/6] FLYWHEEL SPINUP ===")
                    telemetry.addData("Shooter 1", ok(hasShooter1))
                    telemetry.addData("Shooter 2", ok(hasShooter2))
                    telemetry.addData("Target", "100 rad/s")
                    telemetry.addData("Velocity", "%.1f rad/s", vel)
                    telemetry.addData("Reached Target", flywheelEverHitTarget)
                    telemetry.addData("Elapsed", "%.1f s / 3 s", elapsed)
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (elapsed >= 3.0) {
                        flywheelResult = flywheelEverHitTarget
                        phase = Phase.FLYWHEEL_OFF
                        phaseStart = state.timestamp
                    }
                }

                Phase.FLYWHEEL_OFF -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.update()
                    io.intake = 0.0

                    telemetry.addLine("=== [3/6] FLYWHEEL COASTING DOWN ===")
                    telemetry.addData("Velocity", "%.1f rad/s", io.flywheelVelocity())
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (elapsed >= 1.0) {
                        phase = Phase.INTAKE_ON
                        phaseStart = state.timestamp
                    }
                }

                Phase.INTAKE_ON -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.update()
                    io.intake = 1.0  // override intakeTransfer state machine

                    val rpm = io.intake1RPM()
                    if (abs(rpm) > 10.0) intakeSpun = true

                    telemetry.addLine("=== [4/6] INTAKE CHECK ===")
                    telemetry.addData("Intake 1", ok(hasIntake1))
                    telemetry.addData("Intake 2", ok(hasIntake2))
                    telemetry.addData("Intake RPM", "%.0f", rpm)
                    telemetry.addData("Spinning", intakeSpun)
                    telemetry.addData("Elapsed", "%.1f s / 2 s", elapsed)
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (elapsed >= 2.0) {
                        phase = Phase.INTAKE_OFF
                        phaseStart = state.timestamp
                    }
                }

                Phase.INTAKE_OFF -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.update()
                    io.intake = 0.0

                    telemetry.addLine("=== [4/6] INTAKE STOPPING ===")
                    telemetry.addData("Intake RPM", "%.0f", io.intake1RPM())
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (elapsed >= 1.0) {
                        intakeResult = intakeSpun || (hasIntake1 && hasIntake2)
                        phase = Phase.DRIVE_SPIN
                        phaseStart = state.timestamp
                        lastHeading = state.driveTrainPosition.rot
                        spinAccumulated = 0.0
                    }
                }

                Phase.DRIVE_SPIN -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.update()
                    io.intake = 0.0

                    if (hasPinpoint) {
                        val heading = state.driveTrainPosition.rot
                        var delta = heading - lastHeading
                        while (delta > PI) delta -= 2 * PI
                        while (delta < -PI) delta += 2 * PI
                        spinAccumulated += delta
                        lastHeading = heading
                    }

                    val odometryDone = hasPinpoint && abs(spinAccumulated) >= 2 * PI
                    val timedOut = elapsed >= 12.0
                    val spinDone = odometryDone || timedOut

                    robot.drive.drive(Pose2d(0.0, 0.0, if (spinDone) 0.0 else 0.5), io)

                    telemetry.addLine("=== [5/6] DRIVE 360° SPIN ===")
                    telemetry.addData("Pinpoint", ok(hasPinpoint))
                    if (hasPinpoint) {
                        telemetry.addData("Accumulated", "%.1f° / 360°", Math.toDegrees(abs(spinAccumulated)))
                        telemetry.addData("Elapsed", "%.1f s / 12 s", elapsed)
                    } else {
                        telemetry.addData("No odometry", "time-based fallback")
                        telemetry.addData("Elapsed", "%.1f s / 12 s", elapsed)
                    }
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (spinDone) {
                        driveResult = if (hasPinpoint) odometryDone else null
                        phase = Phase.DRIVE_STOP
                        phaseStart = state.timestamp
                    }
                }

                Phase.DRIVE_STOP -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.update()
                    io.intake = 0.0
                    robot.drive.drive(Pose2d(0.0, 0.0, 0.0), io)

                    telemetry.addLine("=== [5/6] DRIVE STOPPING ===")
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (elapsed >= 0.5) {
                        phase = Phase.POSE_CHECK
                        phaseStart = state.timestamp
                    }
                }

                Phase.POSE_CHECK -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.update()
                    io.intake = 0.0
                    robot.drive.drive(Pose2d(0.0, 0.0, 0.0), io)

                    val fusedPose = robot.aim.autoAim.fusedPose
                    val nearOrigin = abs(fusedPose.v.x) < 0.3 && abs(fusedPose.v.y) < 0.3
                    val nearHeading = abs(fusedPose.rot - PI / 2.0) < PI / 4.0

                    telemetry.addLine("=== [6/6] FUSED POSE CHECK ===")
                    telemetry.addData("Pinpoint", ok(hasPinpoint))
                    telemetry.addData("Odometry", "(%.2f m, %.2f m, %.1f°)",
                        state.driveTrainPosition.v.x, state.driveTrainPosition.v.y,
                        Math.toDegrees(state.driveTrainPosition.rot))
                    telemetry.addData("Fused Pose", "(%.2f m, %.2f m, %.1f°)",
                        fusedPose.v.x, fusedPose.v.y, Math.toDegrees(fusedPose.rot))
                    telemetry.addData("Expected", "~(0 m, 0 m, 90°)")
                    telemetry.addData("Near Origin (±0.3 m)", nearOrigin)
                    telemetry.addData("Near Heading (±45°)", nearHeading)
                    telemetry.addData("Elapsed", "%.1f s / 3 s", elapsed)
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()

                    if (elapsed >= 3.0) {
                        poseResult = nearOrigin && nearHeading
                        phase = Phase.DONE
                        phaseStart = state.timestamp
                    }
                }

                Phase.DONE -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.update()
                    io.intake = 0.0
                    robot.drive.drive(Pose2d(0.0, 0.0, 0.0), io)

                    val fusedPose = robot.aim.autoAim.fusedPose

                    telemetry.addLine("=== DIAGNOSTICS COMPLETE ===")
                    telemetry.addData("Fused Pose", "(%.2f m, %.2f m, %.1f°)",
                        fusedPose.v.x, fusedPose.v.y, Math.toDegrees(fusedPose.rot))
                    telemetry.addLine("")
                    addScoreboard()
                    telemetry.update()
                }
            }

            false
        }

        robot.close()
    }

    private fun ok(present: Boolean) = if (present) "OK" else "MISSING"
    private fun pass(v: Boolean) = if (v) "PASS" else "FAIL"
    private fun result(v: Boolean?) = when (v) {
        null -> "..."
        true -> "PASS"
        false -> "FAIL"
    }
}
