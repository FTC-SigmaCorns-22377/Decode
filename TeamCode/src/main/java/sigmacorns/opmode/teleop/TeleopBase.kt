package sigmacorns.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.aim.AutoAim
import sigmacorns.control.MotorRangeMapper
import sigmacorns.control.ShotPowers
import sigmacorns.control.SpindexerLogic
import sigmacorns.control.Turret
import sigmacorns.globalFieldState
import sigmacorns.io.HardwareIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumDynamics
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.time.Duration


@TeleOp(name = "TeleopBlue", group = "Competition")
class TeleopBlue: TeleopBase(true)

@TeleOp(name = "TeleopRed", group = "Competition")
class TeleopRed: TeleopBase(false)

open class TeleopBase(val blue: Boolean) : SigmaOpMode() {

    // Subsystems
    private lateinit var spindexerLogic: SpindexerLogic
    private lateinit var turret: Turret
    private lateinit var autoAim: AutoAim
    private val mecanumDynamics = MecanumDynamics(drivetrainParameters)

    val ticksPerRad = (1.0 + (46.0 / 11.0)) * 28.0 / (2*PI) * 76 / 19

    private val turretRange = MotorRangeMapper(
        limits = -PI/2.0..PI/2.0,           // turret can rotate +/- 190 degrees
        limitsTick = -PI/2.0*ticksPerRad..PI/2.0*ticksPerRad,           // turret can rotate +/- 190 degrees
        slowdownDist = 0.3           // slow down within 0.3 rad of limits
    )

    // State
    private var speedMultiplier = 1.0
    private var targetDistance = 3.0  // default 3m for flywheel speed
    private var dVoltage = 1.0        // voltage compensation factor
    private var lockedShotPower: Double? = null
    private var manualOverridePower: Double? = null

    // Tracking for edge detection on triggers
    private var wasIntaking = false
    private var wasShooting = false
    private var wasAutoAimToggle = false
    private var wasFieldRelativeToggle = false
    private var wasLeftBumper = false
    private var wasRightBumper = false

    private lateinit var gm1: Gamepad
    private lateinit var gm2: Gamepad

    override fun runOpMode() {
//        gm1 = PanelsGamepad.firstManager.asCombinedFTCGamepad(gamepad1)
//        gm2 = PanelsGamepad.secondManager.asCombinedFTCGamepad(gamepad2)
        gm1 = gamepad1
        gm2 = gamepad2
        // Initialize subsystems
        spindexerLogic = SpindexerLogic(io)
        turret = Turret(turretRange, io)

        // Initialize auto-aim with limelight from HardwareIO
        val hardwareIO = io as? HardwareIO
        autoAim = AutoAim(hardwareIO?.limelight, targetAprilTagIds = if(blue) setOf(20) else setOf(24))
        autoAim.configure()

        waitForStart()

        ioLoop { state, dt ->
            // Update voltage compensation
            dVoltage = 12.0 / io.voltage()

            // Update auto-aim system with odometry and turret angle for sensor fusion
            val robotPose = io.position()
            val turretAngle = turret.pos
            autoAim.update(robotPose, turretAngle)

            // Process all controls
            processDrivetrain(dt)
            processTurret(dt)
            processIntakeAndShooting(dt)
            updateTelemetry(state)

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

            false // continue loop
        }
    }

    private fun processDrivetrain(dt: Duration) {
        // Speed mode toggle
        if (gm1.dpad_up) {
            speedMultiplier = 1.0  // Full speed
        } else if (gm1.dpad_down) {
            speedMultiplier = 0.5  // Precision mode
        }

        // Mecanum drive calculation
        val robotPower = Pose2d(
            -gm1.left_stick_y.toDouble() * speedMultiplier,
            -gm1.left_stick_x.toDouble() * speedMultiplier,
            -gm1.right_stick_x.toDouble() * speedMultiplier
        )

        val maxSpeed = mecanumDynamics.maxSpeed()
        val robotVelocities = maxSpeed.componentMul(robotPower)
        val wheelVelocities = mecanumDynamics.mecanumInverseVelKinematics(robotVelocities)
        var wheelPowers = wheelVelocities * (1.0 / mecanumDynamics.p.motor.freeSpeed)

        val maxComponent = wheelPowers[wheelPowers.maxComponent()]
        if (maxComponent > 1.0) {
            wheelPowers *= (1.0 / maxComponent)
        }

        io.driveFL = wheelPowers[0]
        io.driveBL = wheelPowers[1]
        io.driveBR = wheelPowers[2]
        io.driveFR = wheelPowers[3]
    }

    private fun processTurret(dt: Duration) {
        // Update robot heading for field-relative aiming
        turret.robotHeading = io.position().rot
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

        // Turret yaw control
        if (autoAim.enabled && autoAim.hasTarget && !gm2.left_stick_button) {
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

            // Use auto-aim distance for flywheel
            targetDistance = autoAim.targetDistance.coerceIn(1.0, 10.0)
        } else {
            // Manual turret yaw control (operator left stick Y)
            val yawInput = -gm2.left_stick_x.toDouble()
            if (yawInput.absoluteValue > 0.1) {
                if (turret.fieldRelativeMode) {
                    // In field-relative mode, adjust field target angle
                    turret.fieldTargetAngle += yawInput * 0.03  // radians per loop
                } else {
                    turret.targetAngle += yawInput * 0.03  // radians per loop
                }
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
        val dist = if (autoAim.enabled && autoAim.hasTarget) autoAim.targetDistance else targetDistance
        val autoPower = when {
            dist < ShotPowers.shortDistanceLimit -> ShotPowers.shortShotPower
            dist < ShotPowers.midDistanceLimit -> ShotPowers.midShotPower
            else -> ShotPowers.longShotPower
        }

        val activePower = manualOverridePower ?: autoPower

        // Locking Logic
        if (spindexerLogic.shootingRequested) {
            wasShooting
            if (lockedShotPower == null) {
                lockedShotPower = activePower
            }
            spindexerLogic.targetShotPower = lockedShotPower!!
        } else {
            lockedShotPower = null
            spindexerLogic.targetShotPower = activePower
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
            spindexerLogic.nudge(-2 * PI / 3) // 120 deg Left
        }
        wasLeftBumper = gm1.left_bumper

        if (gm1.right_bumper && !wasRightBumper) {
            spindexerLogic.nudge(2 * PI / 3) // 120 deg Right
        }
        wasRightBumper = gm1.right_bumper

        // Shooting controls
        // Driver right trigger - shoot
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
            // Single press shoot - edge detection handled by checking state
            if (spindexerLogic.currentState == SpindexerLogic.State.IDLE ||
                spindexerLogic.currentState == SpindexerLogic.State.FULL) {
                spindexerLogic.shoot()
            }
        }

        // Update spindexer FSM
        spindexerLogic.update(io.spindexerPosition(), dt, dVoltage)
    }

    private fun updateTelemetry(state: sigmacorns.State) {
        // Driver telemetry (essential info)
        telemetry.addLine("=== DRIVER ===")
        telemetry.addData("Spindexer", spindexerLogic.currentState.name)

        // Count balls in spindexer
        val ballCount = spindexerLogic.spindexerState.count { it != null }
        telemetry.addData("Balls", "$ballCount/3")
        telemetry.addData("Speed", if (speedMultiplier == 1.0) "FULL" else "PRECISION")

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
        
        val zone = when (spindexerLogic.targetShotPower) {
            ShotPowers.shortShotPower -> "SHORT"
            ShotPowers.midShotPower -> "MID"
            ShotPowers.longShotPower -> "LONG"
            else -> "CUSTOM"
        }
        telemetry.addData("Shot Zone", zone)
        telemetry.addData("Shot Power", "%.0f%%", spindexerLogic.targetShotPower * 100)
        telemetry.addData("Power Locked", if (lockedShotPower != null) "YES" else "NO")
        telemetry.addData("Manual Override", if (manualOverridePower != null) "YES" else "NO")
        
        telemetry.addData("Flywheel", "%.0f%%", io.shooter * 100)

        telemetry.addLine("")

        // Auto-aim logging section
        telemetry.addLine("=== AUTO-AIM ===")
        telemetry.addData("Enabled", if (autoAim.enabled) "ON" else "OFF")
        telemetry.addData("Limelight Valid", autoAim.lastResultValid)
        telemetry.addData("Tags Detected", autoAim.detectedTagCount)
        telemetry.addData("Uncertainty", "%.0f%%", autoAim.uncertainty * 100)
        telemetry.addData("Target Status", when {
            autoAim.hasVisionTarget && autoAim.uncertainty < autoAim.maxAcceptableUncertainty -> "VISION"
            autoAim.hasVisionTarget -> "VISION (REJECTED)"
            autoAim.usingPrediction -> "PREDICTED"
            else -> "SEARCHING"
        })
        if (autoAim.hasTarget) {
            telemetry.addData("Tracked Tag ID", autoAim.trackedTagId)
            telemetry.addData("Raw TX", "%.2f deg", autoAim.rawTxDegrees)
            telemetry.addData("Adjusted TX", "%.2f deg (sign=%.0f)",
                Math.toDegrees(autoAim.getAdjustedTx()), autoAim.txSignMultiplier)
            telemetry.addData("Target Distance", "%.2f m", autoAim.targetDistance)
            telemetry.addData("Target Robot Angle", "%.1f deg", Math.toDegrees(autoAim.targetRobotAngle))
            telemetry.addData("Current Turret", "%.1f deg", Math.toDegrees(turret.pos))
            val timeSinceDetection = System.currentTimeMillis() - autoAim.lastDetectionTimeMs
            telemetry.addData("Last Detection", "${timeSinceDetection}ms ago")
            autoAim.targetFieldPosition?.let { pos ->
                telemetry.addData("Target Field Pos", "(%.2f, %.2f)", pos.x, pos.y)
            }
        }

        telemetry.addLine("")

        // Position info
        telemetry.addData(
            "Pose (m, m, rad)",
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
