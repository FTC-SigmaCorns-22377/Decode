package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.joml.Quaterniond
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.control.subsystem.DriveController
import sigmacorns.control.MotorRangeMapper
import sigmacorns.control.subsystem.ShotPowers
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.control.subsystem.Turret
import sigmacorns.control.aim.AutoAimGTSAM
import sigmacorns.control.aim.VisionTracker
import sigmacorns.globalFieldState
import sigmacorns.io.HardwareIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.test.AutoAimGTSAMTest.Companion.applyRuntimeConfig
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.DurationUnit

@TeleOp(name = "TestSortingBlue", group = "Competition")
class TeleopBlue: TestSortedShooter(true)

@TeleOp(name = "TestSortingRed", group = "Competition")
class TeleopRed: TestSortedShooter(false)

open class TestSortedShooter(val blue: Boolean) : SigmaOpMode() {

    // Subsystems
    private lateinit var spindexerLogic: SpindexerLogic
    private lateinit var turret: Turret
    private lateinit var autoAim: AutoAimGTSAM
    private lateinit var visionTracker: VisionTracker
    private val driveController = DriveController()

    val ticksPerRad = (1.0 + (46.0 / 11.0)) * 28.0 / (2*PI) * 76 / 19

    private val turretRange = MotorRangeMapper(
        limits = -PI/2.0..PI/2.0,           // turret can rotate +/- 190 degrees
        limitsTick = -PI/2.0*ticksPerRad..PI/2.0*ticksPerRad,           // turret can rotate +/- 190 degrees
        slowdownDist = 0.3           // slow down within 0.3 rad of limits
    )

    // State
    private var targetDistance = 3.0  // default 3m for flywheel speed
    private var dVoltage = 1.0        // voltage compensation factor
    private var lockedShotPower: Double? = null
    private var manualOverridePower: Double? = null
    private var goalPosition: Vector2d? = null

    // Tracking for edge detection on triggers
    private var wasIntaking = false
    private var wasShooting = false
    private var wasAutoAimToggle = false
    private var wasFieldRelativeToggle = false
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
        // Initialize subsystems
        spindexerLogic = SpindexerLogic(io)
        turret = Turret(turretRange, io)

        io.configurePinpoint()
        io.setPosition(Pose2d(0.0,0.0,PI/2.0))

        val ll = (io as? HardwareIO)?.limelight
        ll?.pipelineSwitch(0)
        ll?.start()


        // Initialize auto-aim
        val hardwareIO = io as? HardwareIO
        val limelight = hardwareIO?.limelight

        val q20 = Quaterniond().rotateX(-PI/2.0).rotateLocalZ(Math.toRadians(54.046000))
        val ypr20 = Vector3d()
        q20.getEulerAnglesZYX(ypr20)

        val q24 = Quaterniond().rotateX(-PI/2.0).rotateLocalZ(-Math.toRadians(54.046000))
        val ypr24 = Vector3d()
        q24.getEulerAnglesZYX(ypr24)

        val landmarks = mapOf(
            20 to AutoAimGTSAM.LandmarkSpec(
                Vector3d(-1.413321, 1.481870, 0.7493),
                pitch = ypr20.y,
                roll = ypr20.x,
                yaw = ypr20.z,
                size = 0.165
            ),
            24 to AutoAimGTSAM.LandmarkSpec(
                Vector3d(1.413321, 1.481870, 0.7493),
                pitch = ypr24.y,
                roll = ypr24.x,
                yaw = ypr24.z,
                size = 0.165
            ),
            22 to AutoAimGTSAM.LandmarkSpec(
                position = Vector3d(0.0,1.818888,0.459341),
                roll = -PI/2.0,
                pitch = 0.0,
                yaw = 0.0,
                size = 0.165
            )
        )

        // Goal position based on alliance
        goalPosition = if (blue) {
            Vector2d(-1.480126, 1.598982)
        } else {
            Vector2d(1.480126, 1.598982)
        }

        autoAim = AutoAimGTSAM(
            landmarkPositions = landmarks,
            goalPosition = goalPosition!!,
            initialPose = io.position(), // Use current pose as initial
            estimatorConfig = AutoAimGTSAMTest.buildEstimatorConfig()
        )

        visionTracker = VisionTracker(
            limelight = limelight,
            allowedTagIds = landmarks.keys
        )

        applyRuntimeConfig(autoAim)
        visionTracker.configure(pipeline = AutoAimGTSAMTest.AutoAimGTSAMTestConfig.pipeline)
        autoAim.enabled = true

        try {
            waitForStart()

            var loopStartTime = System.nanoTime()
            ioLoop { state, dt ->

                // Update voltage compensation
                dVoltage = 12.0 / io.voltage()

                applyRuntimeConfig(autoAim)

                // Update auto-aim system with profiling
                val visionStartTime = System.nanoTime()
                val visionResult = visionTracker.read()
                autoAim.update(io.position(), turret.pos, visionResult)
                profileVisionTime = (System.nanoTime() - visionStartTime) / 1_000_000  // convert to ms

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

                loopStartTime = System.nanoTime()
                false // continue loop
            }
        } finally {
            autoAim.close()
            visionTracker.stop()
        }
    }

    private fun processDrivetrain(dt: Duration) {
        driveController.update(gm1, io)
    }

    private fun processTurret(dt: Duration) {
        // Update robot heading for field-relative aiming
        turret.robotHeading = autoAim.fusedPose.rot
        turret.robotAngularVelocity = io.velocity().rot

        // Auto-aim toggle (operator back button)
        val autoAimToggle = gm2.back
        if (autoAimToggle && !wasAutoAimToggle) {
            autoAim.enabled = !autoAim.enabled
        }
        wasAutoAimToggle = autoAimToggle

        // Field-relative mode toggle (operator start button)
        val fieldRelativeToggle = gm2.start
        if (fieldRelativeToggle && !wasFieldRelativeToggle) {
            turret.fieldRelativeMode = !turret.fieldRelativeMode
            // When switching to field-relative, initialize field target from current position
            if (turret.fieldRelativeMode) {
                turret.fieldTargetAngle = turret.pos + turret.robotHeading
            }
        }
        wasFieldRelativeToggle = fieldRelativeToggle

        // Calculate target distance using fused pose and goal position
        val pose = autoAim.fusedPose
        goalPosition?.let { goal ->
            targetDistance = hypot(goal.x - pose.v.x, goal.y - pose.v.y)
        }

        // Turret yaw control
        // Check for manual control input
        val yawInput = -gm2.left_stick_x.toDouble()
        val isManualControl = yawInput.absoluteValue > 0.1 || gm2.left_stick_button

        if (autoAim.enabled && autoAim.hasTarget && !isManualControl) {
            // Auto-aim mode: use sensor fusion for target tracking
            if (turret.fieldRelativeMode) {
                // Use field-relative target angle from sensor fusion
                autoAim.getTargetFieldAngle()?.let { fieldAngle ->
                    turret.fieldTargetAngle = fieldAngle
                }
            } else {
                // Use robot-relative target angle from sensor fusion
                autoAim.getTargetTurretAngle()?.let { robotAngle ->
                    turret.targetAngle = robotAngle
                }
            }
            // Limit target distance for flywheel calculations
            targetDistance = targetDistance.coerceIn(0.1, 10.0)
        } else if (isManualControl) {
            // Manual turret yaw control (operator left stick X)
            if (turret.fieldRelativeMode) {
                // In field-relative mode, adjust field target angle
                turret.fieldTargetAngle += yawInput * Math.toRadians(60.0) * dt.toDouble(DurationUnit.SECONDS)
            } else {
                turret.targetAngle += yawInput * Math.toRadians(60.0) * dt.toDouble(DurationUnit.SECONDS)
            }
        }

        turret.targetDistance = targetDistance

        // Flywheel controls
        var flywheelPower = 0.0

        // Manual flywheel override (operator right stick)
        if (gm2.right_stick_y.absoluteValue > 0.1) {
            flywheelPower = -gm2.right_stick_y.toDouble() * dVoltage
        }

        // Apply flywheel power (only if not being controlled by SpindexerLogic shooting)
        if (spindexerLogic.currentState != SpindexerLogic.State.SHOOTING &&
            spindexerLogic.currentState != SpindexerLogic.State.MOVING_SHOOT) {
            io.shooter = flywheelPower
        }

        // Update turret PID
        turret.update(dt)
    }

    private fun processIntakeAndShooting(dt: Duration) {
        // Zone Selection (Operator D-pad)
        if (gm2.dpad_down) manualOverridePower = ShotPowers.shortShotPower
        if (gm2.dpad_left || gm2.dpad_right) manualOverridePower = ShotPowers.midShotPower
        if (gm2.dpad_up) manualOverridePower = ShotPowers.longShotPower

        // Auto Power Calculation
        val autoPower = when {
            targetDistance < ShotPowers.shortDistanceLimit -> ShotPowers.shortShotPower
            targetDistance < ShotPowers.midDistanceLimit -> ShotPowers.midShotPower
            else -> ShotPowers.longShotPower
        }

        val activePower = manualOverridePower ?: autoPower

        // Locking Logic
        if (spindexerLogic.shootingRequested) {
            wasShooting
            if (lockedShotPower == null) {
                lockedShotPower = activePower
            }
            spindexerLogic.shotPower = lockedShotPower!!
        } else {
            lockedShotPower = null
            spindexerLogic.shotPower = activePower
        }

        // Intake control (driver left trigger)
        val intaking = gm1.left_trigger > 0.1
        if (intaking && !wasIntaking) {
            spindexerLogic.startIntaking()
        } else if (!intaking && wasIntaking) {
            spindexerLogic.stopIntaking()
        }
        wasIntaking = intaking

        /*if (io.distance() < 0.15) {
            spindexerLogic.fsm.sendEvent(SpindexerLogic.Event.BALL_DETECTED)
        }*/

        // Spindexer Nudge Controls (Driver Bumpers)
        if (gm1.left_bumper && !wasLeftBumper) {
            spindexerLogic.nudge(true)
        }
        wasLeftBumper = gm1.left_bumper

        if (gm1.right_bumper && !wasRightBumper) {
            spindexerLogic.nudge(false)
        }
        wasRightBumper = gm1.right_bumper

        // Shooting controls
        // Driver right trigger - shoot
        val isShooting = gm1.right_trigger > 0.5
        spindexerLogic.shootingRequested = isShooting

        if (isShooting && !wasShooting) {
            spindexerLogic.sortedShoot()
        }
        if (!isShooting && wasShooting) {
            manualOverridePower = null
        }
        wasShooting = isShooting

        // Operator quick shot (right bumper)
        if (gm2.right_bumper && !gm2.left_bumper) {
            // Single press shoot - edge detection handled by checking state
            if (spindexerLogic.currentState == SpindexerLogic.State.IDLE ||
                spindexerLogic.currentState == SpindexerLogic.State.FULL) {
                spindexerLogic.shoot()
            }
        }

        // Update spindexer FSM
        spindexerLogic.update(dt)
    }

    private var lastTimestep = 0.milliseconds
    private fun updateTelemetry(state: sigmacorns.State) {
        // Driver telemetry (essential info)
        telemetry.addLine("=== DRIVER ===")
        telemetry.addData("Spindexer", spindexerLogic.currentState.name)

        // Count balls in spindexer
        val ballCount = spindexerLogic.spindexerState.count { it != null }
        telemetry.addData("Balls", "$ballCount/3")
        telemetry.addData("Speed", if (driveController.getSpeedMultiplier() == 1.0) "FULL" else "PRECISION")

        val looptime = state.timestamp-lastTimestep
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

        val zone = when (spindexerLogic.shotPower) {
            ShotPowers.shortShotPower -> "SHORT"
            ShotPowers.midShotPower -> "MID"
            ShotPowers.longShotPower -> "LONG"
            else -> "CUSTOM"
        }
        telemetry.addData("Shot Zone", zone)
        telemetry.addData("Shot Power", "%.0f%%", spindexerLogic.shotPower * 100)
        telemetry.addData("Power Locked", if (lockedShotPower != null) "YES" else "NO")
        telemetry.addData("Manual Override", if (manualOverridePower != null) "YES" else "NO")

        telemetry.addData("Flywheel", "%.0f%%", io.shooter * 100)

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

            // Fused pose telemetry
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

        telemetry.update()
    }
}