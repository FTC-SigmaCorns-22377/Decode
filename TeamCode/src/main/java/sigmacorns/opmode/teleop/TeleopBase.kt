package sigmacorns.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import sigmacorns.constants.flywheelMotor
import sigmacorns.control.subsystem.AimingSystem
import sigmacorns.control.subsystem.DriveController
import sigmacorns.control.subsystem.Flywheel
import sigmacorns.control.subsystem.ShotPowers
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.globalFieldState
import sigmacorns.io.PosePersistence
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.tune.FlywheelDeadbeatConfig
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
    // Subsystems
    private lateinit var spindexerLogic: SpindexerLogic
    private lateinit var aiming: AimingSystem
    private val driveController = DriveController()

    // State
    private var dVoltage = 1.0
    private var lockedShotPower: Double? = null
    private var manualOverridePower: Double? = null

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

        // Initialize subsystems with Flywheel controller
        val flywheel = Flywheel(flywheelMotor,
            FlywheelDeadbeatConfig.inertia,
            io,
            lag = FlywheelDeadbeatConfig.lagMs.milliseconds
        )
        spindexerLogic = SpindexerLogic(io, flywheel)

        if(!fromAuto && PosePersistence.loadPose(storageDir()) == null) {
            io.configurePinpoint()
            io.setPosition(Pose2d(0.0, 0.0, PI / 2.0))
        }

        // Try to load pose from auto, fallback to default if unavailable or too old
        val savedPose = PosePersistence.loadPose(storageDir())
        val initialPose = savedPose ?: Pose2d(0.0, 0.0, PI / 2.0)
        //val initialPose = if(blue) Pose2d(-0.53,0.53,Math.toRadians(152.0)) else Pose2d(0.53,0.53,Math.toRadians(28.0))
        //io.setPosition(initialPose)

        if (savedPose != null) {
            telemetry.addData("Initial Pose", "Loaded from auto")
            telemetry.addData("Position", "(%.2f, %.2f, %.2f rad)", savedPose.v.x, savedPose.v.y, savedPose.rot)
        } else {
            telemetry.addData("Initial Pose", "Using default")
        }
        telemetry.update()

        // Initialize shared aiming system
        aiming = AimingSystem(io, blue)
        aiming.init(io.position())

        try {
            spindexerLogic.autoSort = false
            waitForStart()

            var loopStartTime = System.nanoTime()
            ioLoop { state, dt ->

                // Update voltage compensation
                dVoltage = 12.0 / io.voltage()

                // Update vision + auto-aim
                val visionStartTime = System.nanoTime()
                aiming.updateVision()
                profileVisionTime = (System.nanoTime() - visionStartTime) / 1_000_000

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

                loopStartTime = System.nanoTime()
                false // continue loop
            }
        } finally {
            aiming.close()
        }
    }

    private fun processDrivetrain(dt: Duration) {
        driveController.update(gm1, io)
    }

    private fun processTurret(dt: Duration) {
        val turret = aiming.turret
        val autoAim = aiming.autoAim

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
            if (turret.fieldRelativeMode) {
                turret.fieldTargetAngle = turret.pos + turret.robotHeading
            }
        }
        wasFieldRelativeToggle = fieldRelativeToggle

        // Turret yaw control — check for manual control input
        val yawInput = -gm2.left_stick_x.toDouble()
        val isManualControl = yawInput.absoluteValue > 0.1 || gm2.left_stick_button

        if (autoAim.enabled && autoAim.hasTarget && !isManualControl) {
            // Auto-aim: delegate to aiming system
            aiming.applyAutoAimTarget()
        } else if (isManualControl) {
            // Manual turret yaw control (operator left stick X)
            if (turret.fieldRelativeMode) {
                turret.fieldTargetAngle += yawInput * Math.toRadians(60.0) * dt.toDouble(DurationUnit.SECONDS)
            } else {
                turret.targetAngle += yawInput * Math.toRadians(60.0) * dt.toDouble(DurationUnit.SECONDS)
            }
        }

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

        // Update turret PID (uses heading from aiming system)
        aiming.updateTurret(dt)
    }

    private fun processIntakeAndShooting(dt: Duration) {
        // Use AdaptiveTuner for flywheel velocity, fallback to old zones if not calibrated
        val recommendedVelocity = aiming.getRecommendedFlywheelVelocity()!!

        if (recommendedVelocity != null) {
            // Use adaptive tuner velocity
            spindexerLogic.targetVelocityOverride = recommendedVelocity
        } else {
            // Fallback to old shot power zones if adaptive tuner not calibrated
            val targetDistance = aiming.targetDistance

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
            spindexerLogic.targetShotPower = activePower
            spindexerLogic.targetVelocityOverride = null
        }

        if(gm2.aWasPressed()) spindexerLogic.autoSort = !spindexerLogic.autoSort

        // Intake control (driver left trigger)
        val intaking = gm1.left_trigger > 0.1
        if (intaking && !wasIntaking) {
            spindexerLogic.startIntaking()
        } else if (!intaking) {
            spindexerLogic.stopIntaking()
        }
        wasIntaking = intaking

        // Spindexer Nudge Controls (Driver Bumpers)
        if (gm1.left_bumper && !wasLeftBumper) {
            spindexerLogic.nudge(true)
        }
        wasLeftBumper = gm1.left_bumper

        if (gm1.right_bumper && !wasRightBumper) {
            spindexerLogic.nudge(false)
        }
        wasRightBumper = gm1.right_bumper

        // Shooting controls — Driver right trigger
        val isShooting = gm1.right_trigger > 0.5
        spindexerLogic.shootingRequested = isShooting

        if (isShooting && !wasShooting) {
            spindexerLogic.shoot()
        }
        if (!isShooting && wasShooting) {
            manualOverridePower = null
        }
        wasShooting = isShooting

        // Operator quick shot (right bumper)
        if (gm2.right_bumper && !gm2.left_bumper) {
            if (spindexerLogic.currentState == SpindexerLogic.State.IDLE ||
                spindexerLogic.currentState == SpindexerLogic.State.FULL) {
                spindexerLogic.shoot()
            }
        }

        // Update spindexer FSM
        spindexerLogic.update(dt, dVoltage)
    }

    private var lastTimestep = 0.milliseconds
    private fun updateTelemetry(state: sigmacorns.State) {
        val turret = aiming.turret
        val autoAim = aiming.autoAim
        val targetDistance = aiming.targetDistance

        // Driver telemetry (essential info)
        telemetry.addLine("=== DRIVER ===")
        telemetry.addData("Spindexer", spindexerLogic.currentState.name)
        telemetry.addData("Balls", "${spindexerLogic.spindexerState}")
        telemetry.addData("Speed", if (driveController.getSpeedMultiplier() == 1.0) "FULL" else "PRECISION")

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

        val usingAdaptiveTuner = spindexerLogic.targetVelocityOverride != null && aiming.adaptiveTuner.canInterpolate()
        if (usingAdaptiveTuner) {
            telemetry.addData("Mode", "ADAPTIVE")
            telemetry.addData("Target Vel", "%.0f rad/s", spindexerLogic.targetVelocityOverride ?: 0.0)
        } else {
            val zone = when (spindexerLogic.targetShotPower) {
                ShotPowers.shortShotPower -> "SHORT"
                ShotPowers.midShotPower -> "MID"
                ShotPowers.longShotPower -> "LONG"
                else -> "CUSTOM"
            }
            telemetry.addData("Mode", "ZONES")
            telemetry.addData("Shot Zone", zone)
            telemetry.addData("Shot Power", "%.0f%%", spindexerLogic.targetShotPower * 100)
        }
        telemetry.addData("Power Locked", if (lockedShotPower != null) "YES" else "NO")
        telemetry.addData("Manual Override", if (manualOverridePower != null) "YES" else "NO")

        telemetry.addData("Flywheel", "%.0f%%", io.shooter * 100)
        telemetry.addData("Tuner Points", aiming.adaptiveTuner.pointCount())

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
        telemetry.addData("if nay balls were found:", spindexerLogic.foundAnyBall)
        telemetry.addData("current ball being detected:", io.colorSensorGetBallColor())
        telemetry.update()
    }
}
