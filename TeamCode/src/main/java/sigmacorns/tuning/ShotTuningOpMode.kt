package sigmacorns.tuning

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.constants.flywheelMotor
import sigmacorns.control.subsystem.AimingSystem
import sigmacorns.control.subsystem.Flywheel
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.tune.FlywheelDeadbeatConfig
import sigmacorns.sim.Balls
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.time.Duration.Companion.milliseconds

/**
 * OpMode for tuning flywheel speed vs distance using a linear interpolation table.
 *
 * - Uses [AimingSystem] for vision + turret (same as TeleopBase)
 * - Uses [SpindexerLogic] with [Flywheel] deadbeat controller for shooting
 * - No pitch control — only power vs distance
 * - Web UI at http://192.168.43.1:8082 for full control
 */
@TeleOp(name = "Shot Tuning", group = "Tuning")
class ShotTuningOpMode : SigmaOpMode() {

    companion object {
        const val WEB_SERVER_PORT = 8082
        const val FLYWHEEL_SPEED_THRESHOLD = 20.0    // rad/s
        const val MAX_VELOCITY_HISTORY = 500          // ~10 seconds at 50Hz
    }

    // Subsystems
    private lateinit var aiming: AimingSystem
    private lateinit var spindexerLogic: SpindexerLogic
    private lateinit var flywheel: Flywheel
    private lateinit var dataStore: ShotDataStore
    private lateinit var tuner: AdaptiveTuner
    private var webServer: TuningWebServer? = null

    // State
    private var targetSpeed = 0.0
    private var dVoltage = 1.0
    @Volatile private var shootRequested = false
    @Volatile private var pendingInertia: Double? = null

    // Velocity history ring buffer for web graph
    private val velocityHistory = mutableListOf<TuningWebServer.VelocitySnapshot>()
    private var startTimeMs = 0L

    override fun runOpMode() {
        startTimeMs = System.currentTimeMillis()

        io.setPosition(Pose2d(0.0, 0.0, PI / 2.0))

        // Initialize shared aiming system (hardcoded red for tuning — doesn't affect turret aim)
        aiming = AimingSystem(io, blue = false)
        aiming.init(io.position(),true)

        // Initialize data store and tuner
        dataStore = ShotDataStore()
        dataStore.load()
        tuner = AdaptiveTuner(dataStore)

        // Initialize flywheel deadbeat controller
        flywheel = Flywheel(
            motor = flywheelMotor,
            inertia = FlywheelDeadbeatConfig.inertia,
            io = io,
            lag = FlywheelDeadbeatConfig.lagMs.milliseconds
        )

        // Initialize spindexer logic with flywheel controller
        spindexerLogic = SpindexerLogic(io, flywheel)
        spindexerLogic.spindexerState[0] = Balls.Green
        spindexerLogic.spindexerState[1] = Balls.Green
        spindexerLogic.spindexerState[2] = Balls.Green

        // Initialize web server
        webServer = TuningWebServer(
            port = WEB_SERVER_PORT,
            stateProvider = { getCurrentState() },
            velocityHistoryProvider = { getVelocityHistory() },
            onShoot = { shootRequested = true },
            onInertiaChange = { pendingInertia = it },
            dataStore = dataStore,
            tuner = tuner
        )

        telemetry.addLine("Shot Tuning OpMode")
        telemetry.addLine("Web UI: http://192.168.43.1:$WEB_SERVER_PORT")
        telemetry.addLine("Points loaded: ${tuner.pointCount()}")
        telemetry.addLine("Can interpolate: ${tuner.canInterpolate()}")
        telemetry.update()

        waitForStart()

        try {
            webServer?.start()
        } catch (e: Exception) {
            telemetry.addLine("Web server failed: ${e.message}")
            telemetry.update()
        }

        try {
            ioLoop { state, dt ->
                dVoltage = 12.0 / io.voltage()

                // Handle inertia changes from web UI
                pendingInertia?.let { newInertia ->
                    pendingInertia = null
                    FlywheelDeadbeatConfig.inertia = newInertia
                    flywheel = Flywheel(
                        motor = flywheelMotor,
                        inertia = newInertia,
                        io = io,
                        lag = FlywheelDeadbeatConfig.lagMs.milliseconds
                    )
                    spindexerLogic.flywheel = flywheel
                }

                // Update vision + auto-aim turret (full pipeline)
                aiming.update(dt)

                // Get target speed from interpolation table
                val distance = aiming.targetDistance
                if (aiming.autoAim.hasTarget && tuner.canInterpolate()) {
                    targetSpeed = tuner.getRecommendedSpeed(distance) ?: 0.0
                }

                // Set target velocity on spindexer logic (drives the flywheel)
                spindexerLogic.shotVelocity = if (targetSpeed > 0) targetSpeed else null

                // Handle shoot requests
                if (shootRequested) {
                    shootRequested = false
                    if (isReadyToShoot()) {
                        spindexerLogic.spindexerState[1] = Balls.Green
                        spindexerLogic.shoot()
                    }
                }

                // Update spindexer logic (handles FSM + flywheel control)
                spindexerLogic.update(dt)

                // Record velocity history
                recordVelocity()

                // Handle gamepad controls
                handleGamepadInput()

                // Update telemetry
                updateTelemetry()

                false
            }
        } finally {
            webServer?.stop()
            aiming.close()
            dataStore.save()
        }
    }

    private fun getCurrentState(): TuningWebServer.TuningState {
        return TuningWebServer.TuningState(
            hasTarget = aiming.autoAim.hasTarget,
            distance = aiming.targetDistance,
            currentFlywheelSpeed = io.flywheelVelocity(),
            targetFlywheelSpeed = targetSpeed,
            turretAligned = isTurretAligned(),
            readyToShoot = isReadyToShoot(),
            inertia = FlywheelDeadbeatConfig.inertia,
            spindexerState = spindexerLogic.currentState.name
        )
    }

    @Synchronized
    private fun getVelocityHistory(): List<TuningWebServer.VelocitySnapshot> {
        return velocityHistory.toList()
    }

    @Synchronized
    private fun recordVelocity() {
        val t = (System.currentTimeMillis() - startTimeMs) / 1000.0
        velocityHistory.add(TuningWebServer.VelocitySnapshot(
            time = t,
            target = targetSpeed,
            actual = io.flywheelVelocity()
        ))
        while (velocityHistory.size > MAX_VELOCITY_HISTORY) {
            velocityHistory.removeAt(0)
        }
    }

    private fun isTurretAligned(): Boolean {
        return aiming.autoAim.hasTarget &&
                aiming.autoAim.targetTx.absoluteValue < 0.05
    }

    private fun isFlywheelAtSpeed(): Boolean {
        return targetSpeed > 0 &&
                (io.flywheelVelocity() - targetSpeed).absoluteValue < FLYWHEEL_SPEED_THRESHOLD
    }

    private fun isReadyToShoot(): Boolean {
        return aiming.autoAim.hasTarget &&
                isTurretAligned() &&
                spindexerLogic.currentState == SpindexerLogic.State.IDLE
    }

    private fun handleGamepadInput() {
        // A = Shoot
        if (gamepad1.a && isReadyToShoot()) {
            spindexerLogic.spindexerState[1] = Balls.Green
            spindexerLogic.shoot()
        }

        // D-pad = manual speed adjustment
        if (gamepad1.dpad_up) {
            targetSpeed = (targetSpeed + 10).coerceAtMost(AdaptiveTuner.MAX_FLYWHEEL_SPEED)
            while (gamepad1.dpad_up && opModeIsActive()) idle()
        }
        if (gamepad1.dpad_down) {
            targetSpeed = (targetSpeed - 10).coerceAtLeast(0.0)
            while (gamepad1.dpad_down && opModeIsActive()) idle()
        }

        // Right bumper = toggle auto-aim
        if (gamepad1.right_bumper) {
            aiming.autoAim.enabled = !aiming.autoAim.enabled
            while (gamepad1.right_bumper && opModeIsActive()) idle()
        }
    }

    private fun updateTelemetry() {
        telemetry.addLine("=== Shot Tuning ===")
        telemetry.addData("Web UI", "http://192.168.43.1:$WEB_SERVER_PORT")
        telemetry.addLine("")

        telemetry.addData("Target", if (aiming.autoAim.hasTarget) "LOCKED" else "SEARCHING")
        telemetry.addData("Distance", "%.2f m".format(aiming.targetDistance))
        telemetry.addData("Turret Aligned", isTurretAligned())
        telemetry.addLine("")

        telemetry.addData("Target Speed", "%.0f rad/s".format(targetSpeed))
        telemetry.addData("Actual Speed", "%.0f rad/s".format(io.flywheelVelocity()))
        telemetry.addData("At Speed", isFlywheelAtSpeed())
        telemetry.addData("Spindexer", spindexerLogic.currentState.name)
        telemetry.addData("Ready", isReadyToShoot())
        telemetry.addLine("")

        telemetry.addData("Points", tuner.pointCount())
        telemetry.addData("Can Interpolate", tuner.canInterpolate())
        telemetry.addData("Inertia", "%.3f".format(FlywheelDeadbeatConfig.inertia))
        telemetry.addLine("")

        telemetry.addLine("A=Shoot, D-pad=Adjust, RB=Toggle aim")
        telemetry.update()
    }
}
