package sigmacorns.tuning

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.control.AutoAim
import sigmacorns.control.MotorRangeMapper
import sigmacorns.control.Turret
import sigmacorns.io.HardwareIO
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.tuning.model.FeedbackType
import sigmacorns.tuning.model.ShotTrial
import kotlin.math.PI
import kotlin.math.absoluteValue

/**
 * OpMode for gathering shot data to find optimal flywheel speeds for different distances.
 *
 * Features:
 * - Auto-aims turret to AprilTag
 * - Fixed hood angle (varies only flywheel speed)
 * - Web interface for feedback at http://192.168.43.1:8082
 * - Smart adaptive tuning algorithm
 * - Persists data to /sdcard/FIRST/shot_tuning_data.json
 */
@TeleOp(name = "Shot Tuning", group = "Tuning")
class ShotTuningOpMode : SigmaOpMode() {

    companion object {
        const val WEB_SERVER_PORT = 8082
        const val TURRET_ALIGNMENT_THRESHOLD = 0.05  // radians (~3 degrees)
        const val FLYWHEEL_SPEED_THRESHOLD = 20.0    // rad/s
        const val FIXED_HOOD_ANGLE = 0.5             // servo position (0-1)
        const val MAX_FLYWHEEL_SPEED = 628.0         // rad/s (~6000 RPM)
    }

    // Turret configuration (same as TeleopV2)
    private val ticksPerRad = (1.0 + (46.0 / 11.0)) * 28.0 / (2 * PI) * 76 / 19
    private val turretRange = MotorRangeMapper(
        limits = -PI / 2.0..PI / 2.0,
        limitsTick = -PI / 2.0 * ticksPerRad..PI / 2.0 * ticksPerRad,
        slowdownDist = 0.3
    )

    // Subsystems
    private lateinit var autoAim: AutoAim
    private lateinit var turret: Turret
    private lateinit var dataStore: ShotDataStore
    private lateinit var adaptiveTuner: AdaptiveTuner
    private var webServer: TuningWebServer? = null

    // State
    private var targetFlywheelSpeed = 0.0
    private var currentDistance = 0.0
    @Volatile private var pendingFeedback: Pair<FeedbackType, Int>? = null
    @Volatile private var shootRequested = false
    private var shootingPhase = ShootingPhase.IDLE
    private var shootingStartTime = 0L

    private enum class ShootingPhase {
        IDLE,           // Waiting for shot trigger
        SPINNING_UP,    // Flywheel ramping up
        SHOOTING,       // Ball being released
        COOLDOWN        // Brief pause after shot
    }

    override fun runOpMode() {
        // Initialize hardware
        val hardwareIO = io as? HardwareIO
        autoAim = AutoAim(hardwareIO?.limelight)
        autoAim.configure()
        autoAim.enabled = true  // Always enabled for tuning

        turret = Turret(turretRange, io)

        // Initialize data store and tuner
        dataStore = ShotDataStore()
        dataStore.load()
        adaptiveTuner = AdaptiveTuner(dataStore)

        // Initialize web server
        webServer = TuningWebServer(
            port = WEB_SERVER_PORT,
            stateProvider = { getCurrentState() },
            onFeedback = { type, intensity -> pendingFeedback = Pair(type, intensity) },
            onShoot = { shootRequested = true },
            dataStore = dataStore
        )

        telemetry.addLine("Shot Tuning OpMode")
        telemetry.addLine("Web UI: http://192.168.43.1:$WEB_SERVER_PORT")
        telemetry.addLine("")
        telemetry.addLine("Loaded ${dataStore.getAllTrials().size} trials")
        telemetry.addLine("Lookup table: ${dataStore.getLookupTable().size} entries")
        telemetry.update()

        waitForStart()

        // Start web server
        try {
            webServer?.start()
            telemetry.addLine("Web server started on port $WEB_SERVER_PORT")
        } catch (e: Exception) {
            telemetry.addLine("Web server failed: ${e.message}")
        }
        telemetry.update()

        try {
            ioLoop { state, dt ->
                // Update vision
                autoAim.update()

                // Get current distance
                if (autoAim.hasTarget) {
                    currentDistance = autoAim.targetDistance
                }

                // Update turret aim
                if (autoAim.hasTarget) {
                    turret.targetAngle = turret.pos + autoAim.getTurretYawAdjustment()
                }
                turret.targetPitch = FIXED_HOOD_ANGLE
                io.turretAngle = FIXED_HOOD_ANGLE
                turret.update(dt)

                // Get recommended speed based on current distance
                if (autoAim.hasTarget) {
                    targetFlywheelSpeed = adaptiveTuner.getRecommendedSpeed(currentDistance)
                }

                // Handle shooting phases
                handleShooting()

                // Process pending feedback
                processFeedback()

                // Handle gamepad controls (fallback)
                handleGamepadInput()

                // Update telemetry
                updateTelemetry()

                false // continue loop
            }
        } finally {
            // Cleanup
            webServer?.stop()
            autoAim.stop()
            dataStore.save()
        }
    }

    private fun getCurrentState(): TuningWebServer.TuningState {
        return TuningWebServer.TuningState(
            hasTarget = autoAim.hasTarget,
            distance = currentDistance,
            currentFlywheelSpeed = io.flywheelVelocity(),
            targetFlywheelSpeed = targetFlywheelSpeed,
            turretAligned = isTurretAligned(),
            readyToShoot = isReadyToShoot(),
            tuningInfo = adaptiveTuner.getTuningStateInfo(currentDistance)
        )
    }

    private fun isTurretAligned(): Boolean {
        return autoAim.hasTarget && autoAim.targetTx.absoluteValue < TURRET_ALIGNMENT_THRESHOLD
    }

    private fun isFlywheelAtSpeed(): Boolean {
        return (io.flywheelVelocity() - targetFlywheelSpeed).absoluteValue < FLYWHEEL_SPEED_THRESHOLD
    }

    private fun isReadyToShoot(): Boolean {
        return autoAim.hasTarget &&
                isTurretAligned() &&
                isFlywheelAtSpeed() &&
                shootingPhase == ShootingPhase.IDLE
    }

    private fun handleShooting() {
        val now = System.currentTimeMillis()

        when (shootingPhase) {
            ShootingPhase.IDLE -> {
                // Spin flywheel to target speed (normalized to power)
                val power = (targetFlywheelSpeed / MAX_FLYWHEEL_SPEED).coerceIn(0.0, 1.0)
                io.shooter = power

                // Check for shoot request
                if (shootRequested && isReadyToShoot()) {
                    shootRequested = false
                    shootingPhase = ShootingPhase.SPINNING_UP
                    shootingStartTime = now
                }
            }

            ShootingPhase.SPINNING_UP -> {
                val power = (targetFlywheelSpeed / MAX_FLYWHEEL_SPEED).coerceIn(0.0, 1.0)
                io.shooter = power

                // Wait for flywheel to reach speed (max 1 second timeout)
                if (isFlywheelAtSpeed() || (now - shootingStartTime > 1000)) {
                    shootingPhase = ShootingPhase.SHOOTING
                    shootingStartTime = now
                }
            }

            ShootingPhase.SHOOTING -> {
                // Keep flywheel at speed
                val power = (targetFlywheelSpeed / MAX_FLYWHEEL_SPEED).coerceIn(0.0, 1.0)
                io.shooter = power

                // Activate transfer servo to release ball
                io.transfer = 0.7

                // Hold for 300ms
                if (now - shootingStartTime > 300) {
                    shootingPhase = ShootingPhase.COOLDOWN
                    shootingStartTime = now
                }
            }

            ShootingPhase.COOLDOWN -> {
                // Reset transfer servo
                io.transfer = 0.0

                // Keep flywheel spinning
                val power = (targetFlywheelSpeed / MAX_FLYWHEEL_SPEED).coerceIn(0.0, 1.0)
                io.shooter = power

                // Wait 200ms before allowing next shot
                if (now - shootingStartTime > 200) {
                    shootingPhase = ShootingPhase.IDLE
                }
            }
        }
    }

    private fun processFeedback() {
        val feedback = pendingFeedback ?: return
        pendingFeedback = null

        val (type, intensity) = feedback

        // Record trial
        val trial = ShotTrial(
            timestamp = System.currentTimeMillis(),
            distance = currentDistance,
            flywheelSpeed = targetFlywheelSpeed,
            feedbackType = type,
            feedbackIntensity = intensity
        )
        dataStore.addTrial(trial)

        // Update adaptive tuner
        adaptiveTuner.processFeedback(currentDistance, type, intensity)

        // Save data
        dataStore.save()
    }

    private fun handleGamepadInput() {
        // A = Shoot
        if (gamepad1.a && isReadyToShoot() && !shootRequested) {
            shootRequested = true
        }

        // B = Good
        if (gamepad1.b) {
            pendingFeedback = Pair(FeedbackType.GOOD, 0)
            // Debounce
            while (gamepad1.b && opModeIsActive()) idle()
        }

        // X = Undershoot (hold left bumper for big)
        if (gamepad1.x) {
            val intensity = if (gamepad1.left_bumper) 2 else 1
            pendingFeedback = Pair(FeedbackType.UNDERSHOOT, intensity)
            while (gamepad1.x && opModeIsActive()) idle()
        }

        // Y = Overshoot (hold left bumper for big)
        if (gamepad1.y) {
            val intensity = if (gamepad1.left_bumper) 2 else 1
            pendingFeedback = Pair(FeedbackType.OVERSHOOT, intensity)
            while (gamepad1.y && opModeIsActive()) idle()
        }

        // Right bumper = enable/disable auto-aim
        if (gamepad1.right_bumper) {
            autoAim.enabled = !autoAim.enabled
            while (gamepad1.right_bumper && opModeIsActive()) idle()
        }

        // D-pad = manual speed adjustment
        if (gamepad1.dpad_up) {
            targetFlywheelSpeed = (targetFlywheelSpeed + 10).coerceAtMost(MAX_FLYWHEEL_SPEED)
            while (gamepad1.dpad_up && opModeIsActive()) idle()
        }
        if (gamepad1.dpad_down) {
            targetFlywheelSpeed = (targetFlywheelSpeed - 10).coerceAtLeast(100.0)
            while (gamepad1.dpad_down && opModeIsActive()) idle()
        }
    }

    private fun updateTelemetry() {
        telemetry.addLine("=== Shot Tuning ===")
        telemetry.addData("Web UI", "http://192.168.43.1:$WEB_SERVER_PORT")
        telemetry.addLine("")

        telemetry.addData("Target", if (autoAim.hasTarget) "LOCKED" else "SEARCHING")
        telemetry.addData("Distance", "%.2f m".format(currentDistance))
        telemetry.addData("Turret TX", "%.1f deg".format(Math.toDegrees(autoAim.targetTx)))
        telemetry.addData("Turret Aligned", isTurretAligned())
        telemetry.addLine("")

        telemetry.addData("Target Speed", "%.0f rad/s".format(targetFlywheelSpeed))
        telemetry.addData("Actual Speed", "%.0f rad/s".format(io.flywheelVelocity()))
        telemetry.addData("Flywheel At Speed", isFlywheelAtSpeed())
        telemetry.addData("Phase", shootingPhase.name)
        telemetry.addData("Ready", isReadyToShoot())
        telemetry.addLine("")

        telemetry.addData("Tuning Info", adaptiveTuner.getTuningStateInfo(currentDistance))
        telemetry.addLine("")

        val bucket = dataStore.distanceToBucket(currentDistance)
        val trialsAtDist = dataStore.getTrialsForDistance(currentDistance).size
        val lookupEntry = dataStore.getLookupTable()[bucket]
        telemetry.addData("Bucket", "%.2f m".format(bucket))
        telemetry.addData("Trials at Distance", trialsAtDist)
        if (lookupEntry != null) {
            telemetry.addData("Confirmed Speed", "%.0f rad/s".format(lookupEntry.optimalSpeed))
        }
        telemetry.addLine("")

        telemetry.addData("Total Trials", dataStore.getAllTrials().size)
        telemetry.addData("Lookup Entries", dataStore.getLookupTable().size)
        telemetry.addLine("")

        telemetry.addLine("Gamepad: A=Shoot, B=Good")
        telemetry.addLine("X=Under, Y=Over (LB=big)")
        telemetry.addLine("D-pad=Adjust speed")
        telemetry.update()
    }
}
