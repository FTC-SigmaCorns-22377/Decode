package sigmacorns.opmode.teleop

import com.bylazar.gamepad.PanelsGamepad
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.AutoAim
import sigmacorns.control.MotorRangeMapper
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

@TeleOp(name = "TeleopV2", group = "Competition")
class TeleopV2 : SigmaOpMode() {

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

    // Tracking for edge detection on triggers
    private var wasIntaking = false
    private var wasShooting = false
    private var wasAutoAimToggle = false
    private var wasFieldRelativeToggle = false

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
        autoAim = AutoAim(hardwareIO?.limelight)
        autoAim.configure()

        val voltageSensor = hardwareMap.voltageSensor.iterator().next()

        waitForStart()

        ioLoop { state, dt ->
            // Update voltage compensation
            val voltage = voltageSensor.voltage
            dVoltage = 12.0 / voltage

            // Update auto-aim system with odometry and turret angle for sensor fusion
            val robotPose = io.position()
            val turretAngle = turret.pos
            autoAim.update(robotPose, turretAngle)

            // Process all controls
            processDrivetrain(dt)
            processTurret(dt)
            processIntakeAndShooting(dt)
            processPracticeGame()
            updateTelemetry(state)

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
            return
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
        if (autoAim.enabled && autoAim.hasTarget) {
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
            val yawInput = -gm2.left_stick_y.toDouble()
            if (yawInput.absoluteValue > 0.1) {
                if (turret.fieldRelativeMode) {
                    // In field-relative mode, adjust field target angle
                    turret.fieldTargetAngle += yawInput * 0.03  // radians per loop
                } else {
                    turret.targetAngle += yawInput * 0.03  // radians per loop
                }
            }
        }

        // Manual turret pitch control (operator left stick X -> servo)
        val pitchInput = gm2.left_stick_x.toDouble()
        if (pitchInput.absoluteValue > 0.1) {
            turret.targetPitch = (turret.targetPitch + pitchInput * 0.02).coerceIn(0.0, 1.0)
            io.turretAngle = turret.targetPitch
        }

        // Distance adjustment for flywheel speed (operator D-pad) - only in manual mode
        if (!autoAim.enabled || !autoAim.hasTarget) {
            if (gm2.dpad_up) {
                targetDistance = (targetDistance + 0.05).coerceAtMost(10.0)
            }
            if (gm2.dpad_down) {
                targetDistance = (targetDistance - 0.05).coerceAtLeast(1.0)
            }
        }
        turret.targetDistance = targetDistance

        // Flywheel controls
        var flywheelPower = 0.0

        // Driver flywheel preset (right bumper = far shot)
        if (gm1.right_bumper) {
            flywheelPower = 0.79 * dVoltage
        }

        // Manual flywheel override (operator right stick)
        if (gm2.right_stick_y.absoluteValue > 0.1) {
            flywheelPower = -gm2.right_stick_y.toDouble() * dVoltage
        }

        // Apply flywheel power (only if not being controlled by SpindexerLogic shooting)
        if (spindexerLogic.currentState != SpindexerLogic.State.SHOOTING &&
            spindexerLogic.currentState != SpindexerLogic.State.MOVING_SHOOT) {
            if (flywheelPower != 0.0) {
                io.shooter = flywheelPower
            }
        }

        // Update turret PID
        turret.update(dt)
    }

    private fun processIntakeAndShooting(dt: Duration) {
        // Intake control (driver left trigger)
        val intaking = gm1.left_trigger > 0.1
        if (intaking && !wasIntaking) {
            spindexerLogic.startIntaking()
        } else if (!intaking && wasIntaking) {
            spindexerLogic.stopIntaking()
        }
        wasIntaking = intaking

        if (io.distance() < 0.15) {
            spindexerLogic.fsm.sendEvent(SpindexerLogic.Event.BALL_DETECTED)
        }

        // Shooting controls
        // Driver right trigger - shoot
        val shooting = gm1.right_trigger > 0.5
        if (shooting && !wasShooting) {
            spindexerLogic.shoot()
        }
        wasShooting = shooting

        // Operator quick shot (right bumper)
        if (gm2.right_bumper && !gm2.left_bumper) {
            // Single press shoot - edge detection handled by checking state
            if (spindexerLogic.currentState == SpindexerLogic.State.IDLE ||
                spindexerLogic.currentState == SpindexerLogic.State.FULL) {
                spindexerLogic.shoot()
            }
        }

        // Operator rapid fire (left bumper held)
        if (gm2.left_bumper) {
            // Continuous shooting while held
            if (spindexerLogic.currentState == SpindexerLogic.State.IDLE ||
                spindexerLogic.currentState == SpindexerLogic.State.FULL) {
                spindexerLogic.shoot()
            }
        }

        // Update spindexer FSM
        spindexerLogic.update(io.spindexerPosition(), dt)
    }

    private fun processPracticeGame() {
        val rampArray = globalFieldState.ramp

        // Add green ball (A button)
        if (gm2.a && !gm2.start) {
            for (i in 0 until 9) {
                if (rampArray[i] == 0) {
                    rampArray[i] = 1
                    break
                }
            }
        }

        // Add purple ball (B button)
        if (gm2.b && !gm2.start) {
            for (i in 0 until 9) {
                if (rampArray[i] == 0) {
                    rampArray[i] = 2
                    break
                }
            }
        }

        // Remove last ball (X button)
        if (gm2.x && !gm2.start) {
            for (i in 8 downTo 0) {
                if (rampArray[i] != 0) {
                    rampArray[i] = 0
                    break
                }
            }
        }

        // Clear all (Y button)
        if (gm2.y && !gm2.start) {
            for (i in 0 until 9) {
                rampArray[i] = 0
            }
        }
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
