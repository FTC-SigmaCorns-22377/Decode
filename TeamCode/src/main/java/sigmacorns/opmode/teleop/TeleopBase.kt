package sigmacorns.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import sigmacorns.control.Robot
import sigmacorns.globalFieldState
import sigmacorns.io.PosePersistence
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.DurationUnit

@TeleOp(name = "TeleopBlue", group = "Competition")
class TeleopBlue(fromAuto: Boolean = false): TeleopBase(true, fromAuto)

@TeleOp(name = "TeleopRed", group = "Competition")
class TeleopRed(fromAuto: Boolean = false): TeleopBase(false, fromAuto)

open class TeleopBase(
    val blue: Boolean,
    val fromAuto: Boolean = false
) : SigmaOpMode() {
    private lateinit var robot: Robot

    // State
    private var dVoltage = 1.0
    private var lockedShotPower: Double? = null
    private var manualOverridePower: Double? = null

    // Tracking for edge detection on triggers
    private var wasIntaking = false
    private var wasShooting = false
    private var wasAutoAimToggle = false
    private var wasLeftBumper = false
    private var wasRightBumper = false

    // Profiling
    private var profileVisionTime = 0L
    private var profileDrivetrainTime = 0L
    private var profileTurretTime = 0L
    private var profileIntakeTime = 0L
    private var profileTelemetryTime = 0L
    private var profileTotalLoopTime = 0L

    private lateinit var gm1: Gamepad
    private lateinit var gm2: Gamepad

    override fun runOpMode() {
        gm1 = gamepad1
        gm2 = gamepad2

        robot = Robot(io,blue)
        robot.init(
            PosePersistence.loadPose(storageDir()) ?: Pose2d(0.0,0.0,PI/2.0),
            true
        )
        robot.startApriltag()

        telemetry.update()

        try {
            robot.logic.autoSort = false
            waitForStart()

            var loopStartTime = System.nanoTime()
            ioLoop { state, dt ->
                dVoltage = 12.0 / io.voltage()

                // Process all controls with profiling
                val driveStartTime = System.nanoTime()
                processDrivetrain(dt)
                profileDrivetrainTime = (System.nanoTime() - driveStartTime) / 1_000_000

                val turretStartTime = System.nanoTime()
                processTurret(dt)
                profileTurretTime = (System.nanoTime() - turretStartTime) / 1_000_000

                val intakeStartTime = System.nanoTime()
                processIntakeAndShooting(dt)
                profileIntakeTime = (System.nanoTime() - intakeStartTime) / 1_000_000

                val telemetryStartTime = System.nanoTime()
                updateTelemetry(state)
                profileTelemetryTime = (System.nanoTime() - telemetryStartTime) / 1_000_000

                profileTotalLoopTime = (System.nanoTime() - loopStartTime) / 1_000_000

                // Emergency stop
                if (gm1.a) {
                    io.driveFL = 0.0
                    io.driveBL = 0.0
                    io.driveFR = 0.0
                    io.driveBR = 0.0
                    io.shooter = 0.0
                    io.intake = 0.0
                    io.turret = 0.0
                    io.spindexer = 0.0
                }

                if(gm1.b) {
                    io.intake = 1.0
                }

                if(gm2.y) {
                    io.setPosition(Pose2d(0.0,0.0, PI/2.0))
                }

                robot.update()

                loopStartTime = System.nanoTime()
                false // continue loop
            }
        } finally {
            robot.close()
        }
    }

    private fun processDrivetrain(dt: Duration) {
        robot.drive.update(gm1, io)
    }

    private fun processTurret(dt: Duration) {
        // Auto-aim toggle (operator back button)
        val autoAimToggle = gm2.back
        val autoAim = robot.aim.autoAim
        if (autoAimToggle && !wasAutoAimToggle) {
            autoAim.enabled = !autoAim.enabled
        }
        wasAutoAimToggle = autoAimToggle

        // Turret yaw control — check for manual control input
        val yawInput = -gm2.left_stick_x.toDouble()

        robot.aim.positionOverride = (robot.aim.positionOverride ?: 0.0).let {
            it + yawInput * Math.toRadians(60.0) * dt.toDouble(DurationUnit.SECONDS)
        }.takeIf { yawInput.absoluteValue > 0.1 || gm2.left_stick_button }

        // Manual flywheel override (operator right stick)
        if (gm2.right_stick_y.absoluteValue > 0.1 || gm2.right_stick_button) {
            io.shooter = -gm2.right_stick_y.toDouble() * dVoltage
        }
    }

    private fun processIntakeAndShooting(dt: Duration) {
        if(gm2.aWasPressed()) robot.logic.autoSort = !robot.logic.autoSort

        // Intake control (driver left trigger)
        val intaking = gm1.left_trigger > 0.1
        if (intaking && !wasIntaking)
            robot.logic.startIntaking()
        else if (!intaking)
            robot.logic.stopIntaking()

        wasIntaking = intaking

        // Spindexer Nudge Controls (Driver Bumpers)
        if (gm1.left_bumper && !wasLeftBumper) {
            robot.logic.nudge(true)
        }
        wasLeftBumper = gm1.left_bumper

        if (gm1.right_bumper && !wasRightBumper) {
            robot.logic.nudge(false)
        }
        wasRightBumper = gm1.right_bumper

        // Shooting controls — Driver right trigger
        val isShooting = gm1.right_trigger > 0.5
        robot.logic.shootingRequested = isShooting

        if (isShooting && !wasShooting) {
            robot.logic.shoot()
        }
        if (!isShooting && wasShooting) {
            manualOverridePower = null
        }
        wasShooting = isShooting
    }

    private var lastTimestep = 0.milliseconds
    private fun updateTelemetry(state: sigmacorns.State) {
        val turret = robot.aim.turret
        val autoAim = robot.aim.autoAim
        val targetDistance = robot.aim.targetDistance

        // Driver telemetry (essential info)
        telemetry.addLine("=== DRIVER ===")
        telemetry.addData("Spindexer", robot.logic.currentState.name)
        telemetry.addData("Balls", "${robot.logic.spindexerState}")
        telemetry.addData("Speed", if (robot.drive.getSpeedMultiplier() == 1.0) "FULL" else "PRECISION")

        val looptime = state.timestamp - lastTimestep
        lastTimestep = state.timestamp
        telemetry.addData("Looptime", "%.1f ms", looptime.inWholeMilliseconds.toDouble())

        telemetry.addLine("")

        // Operator telemetry (detailed)
        telemetry.addLine("=== OPERATOR ===")
        telemetry.addData("Field-Relative", if (turret.fieldRelativeMode) "ON" else "OFF")
        telemetry.addData("Turret Yaw", "%.1f deg", Math.toDegrees(turret.pos))
        if (turret.fieldRelativeMode) {
            telemetry.addData("Field Target", "%.1f deg", Math.toDegrees(turret.fieldTargetAngle))
            telemetry.addData("Effective Target", "%.1f deg", Math.toDegrees(turret.effectiveTargetAngle))
        }
        telemetry.addData("Turret Pitch", "%.2f", turret.targetPitch)
        telemetry.addData("Distance", "%.1f m", targetDistance)

        telemetry.addData("Mode", "ADAPTIVE")
        telemetry.addData("Target Vel", "%.0f rad/s", robot.logic.shotVelocity ?: 0.0)
        telemetry.addData("Power Locked", if (lockedShotPower != null) "YES" else "NO")
        telemetry.addData("Manual Override", if (manualOverridePower != null) "YES" else "NO")

        telemetry.addData("Flywheel", "%.0f%%", io.shooter * 100)
        telemetry.addData("Tuner Points", robot.aim.adaptiveTuner.pointCount())

        telemetry.addLine("")

        // Profiling section
        telemetry.addLine("=== LOOP PROFILING ===")
        telemetry.addData("Total Loop", "%d ms", profileTotalLoopTime)
        telemetry.addData("Vision+AutoAim", "%d ms", profileVisionTime)
        telemetry.addData("Drivetrain", "%d ms", profileDrivetrainTime)
        telemetry.addData("Turret", "%d ms", profileTurretTime)
        telemetry.addData("Intake/Shoot", "%d ms", profileIntakeTime)
        telemetry.addData("Telemetry", "%d ms", profileTelemetryTime)
        val profilerSum = profileVisionTime + profileDrivetrainTime + profileTurretTime + profileIntakeTime + profileTelemetryTime
        telemetry.addData("Sum Measured", "%d ms", profilerSum)
        telemetry.addData("IO+Other", "%d ms", profileTotalLoopTime - profilerSum)

        telemetry.addLine("")

        // Auto-aim logging section
        telemetry.addLine("=== AUTO-AIM GTSAM ===")
        telemetry.addData("Enabled", if (autoAim.enabled) "ON" else "OFF")
        telemetry.addData("Limelight Valid", autoAim.lastResultValid)
        telemetry.addData("Tags Detected", autoAim.detectedTagCount)
        telemetry.addData("Uncertainty", "%.0f%%", autoAim.uncertainty * 100)
        telemetry.addData("Target Status", when {
            autoAim.hasVisionTarget && autoAim.uncertainty < autoAim.aimConfig.maxAcceptableUncertainty -> "VISION"
            autoAim.hasVisionTarget -> "VISION (REJECTED)"
            autoAim.usingPrediction -> "PREDICTED"
            else -> "SEARCHING"
        })
        if (autoAim.hasTarget) {
            telemetry.addData("Tracked Tag ID", autoAim.trackedTagId)
            telemetry.addData("Raw TX", "%.2f deg", autoAim.rawTxDegrees)
            telemetry.addData("Adjusted TX", "%.2f deg", Math.toDegrees(autoAim.targetTx))
            telemetry.addData("Target Distance", "%.2f m", targetDistance)

            val fusedPose = autoAim.fusedPose
            telemetry.addData("Fused Pose (m, m, rad)",
                "%.2f, %.2f, %.2f",
                fusedPose.v.x,
                fusedPose.v.y,
                fusedPose.rot
            )

            telemetry.addData("Current Turret", "%.1f deg", Math.toDegrees(turret.pos))
            val timeSinceDetection = System.currentTimeMillis() - autoAim.lastDetectionTimeMs
            telemetry.addData("Last Detection", "${timeSinceDetection}ms ago")
        }

        telemetry.addLine("")

        // Position info (Raw Odometry)
        telemetry.addData(
            "Raw Odom (m, m, rad)",
            "%.2f, %.2f, %.2f",
            io.position().v.x,
            io.position().v.y,
            io.position().rot
        )

        // Practice game ramp
        telemetry.addData("Ramp", globalFieldState.ramp.contentToString())

        //testing for autosorting
        telemetry.addData("if nay balls were found:", robot.logic.foundAnyBall)
        telemetry.addData("current ball being detected:", io.colorSensorGetBallColor())
        telemetry.update()
    }
}
