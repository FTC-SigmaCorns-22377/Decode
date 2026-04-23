package sigmacorns.opmode.tune

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import sigmacorns.Robot
import sigmacorns.constants.turretPos
import sigmacorns.control.aim.tune.PhysicsEstimateGenerator
import sigmacorns.control.aim.tune.ShotDataStore
import sigmacorns.control.aim.tune.SpeedPoint
import sigmacorns.logic.AimConfig
import sigmacorns.math.Pose2d
import sigmacorns.math.ShooterDataPoint
import sigmacorns.math.ShooterFitter
import sigmacorns.math.ShooterSurfaceCoeffs
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.PI

/**
 * Shooter tuning OpMode: drive, shoot, and collect empirical data points.
 *
 * Two modes (toggle with Y):
 *   MANUAL  — operator sets flywheel RPM (D-pad L/R) and hood angle (D-pad U/D),
 *             drives and shoots, then presses A to log the data point.
 *   AUTO    — fitted polynomial surfaces set hood + flywheel from distance automatically;
 *             operator can still log points to refine the fit.
 *
 * Data persists across opmode restarts on the Control Hub SD card.
 *
 * Controls (single gamepad):
 *   Left stick       — translate
 *   Right stick X    — rotate
 *   Left trigger     — intake
 *   B                — reverse intake
 *   Right trigger    — shoot (flywheel + transfer)
 *   Left bumper      — spin up flywheel only
 *   D-pad L/R        — flywheel target ±25 rad/s
 *   D-pad U/D        — hood angle ±1° (manual mode only)
 *   A                — log data point
 *   X                — delete last data point
 *   Y                — toggle manual / auto mode
 */
@TeleOp(name = "Shooter Tuner", group = "Tune")
class ShooterTuner : SigmaOpMode() {

    companion object {
        private const val DATA_FILE = "/sdcard/FIRST/shooter_tuner_data.json"
        private const val SHOT_DURATION_MS = 1500L
        private const val RAD_S_PER_RPM = 2.0 * PI / 60.0
    }

    override fun runOpMode() {
        // --- Robot setup ---
        val robot = Robot(io, blue = false, useNativeAim = true)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = !SIM)
        robot.startApriltag()
        robot.aimFlywheel = false
        robot.shooter.autoAdjust = false

        // --- Data store ---
        val dataStore = ShotDataStore(DATA_FILE)
        dataStore.load()

        // Inject physics-estimated seed points on first use
        if (dataStore.getPhysicsEstimateCount() == 0) {
            val physicsPoints = PhysicsEstimateGenerator.loadFromResources()
            for (p in physicsPoints) {
                dataStore.addPoint(p)
            }
            dataStore.save()
        }

        // --- Fitter ---
        val fitter = ShooterFitter(
            goalHeight = AimConfig.goalHeight,
            launchHeight = turretPos.z
        )

        val coeffs = AtomicReference<ShooterSurfaceCoeffs?>(null)
        val fitting = AtomicBoolean(false)
        val fitScope = CoroutineScope(Dispatchers.Default + Job())

        var usePhysicsEstimates = true

        fun refit() {
            val points = dataStore.getPoints(includePhysicsEstimates = usePhysicsEstimates)
            if (points.size < 3 || fitting.get()) return
            fitting.set(true)
            fitScope.launch {
                try {
                    val dataPoints = points.map { p ->
                        ShooterDataPoint(
                            rpm = p.speed / RAD_S_PER_RPM,
                            hoodAngle = p.hoodAngle,
                            distance = p.distance
                        )
                    }
                    coeffs.set(fitter.fit(dataPoints))
                } catch (e: Exception) {
                    System.err.println("ShooterFitter.fit() failed: ${e.message}")
                } finally {
                    fitting.set(false)
                }
            }
        }

        // Initial fit if we have data
        refit()

        // --- State ---
        var autoMode = false
        var flywheelTarget = 400.0   // rad/s
        var hoodAngleDeg = ShooterConfig.defaultAngleDeg

        var shotActive = false
        var shotStartTime = io.time()

        // Last logged point for telemetry display
        var lastLoggedRpm = 0.0
        var lastLoggedHood = 0.0
        var lastLoggedDist = 0.0

        var lastShotRads = 0.0
        var lastShotHood = 0.0
        var lastShotDist = 0.0

        // Debounce flags
        var lastA = false
        var lastX = false
        var lastY = false
        var lastDpadLeft = false
        var lastDpadRight = false
        var lastDpadUp = false
        var lastDpadDown = false
        var lastRightBumper = false

        telemetry.addLine("Shooter Tuner initialized")
        telemetry.addData("Data points loaded", dataStore.getPoints().size)
        telemetry.addLine("Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // ============================================================
            // Driving
            // ============================================================
            robot.drive.fieldCentricHeading = robot.aim.autoAim.fusedPose.rot - 0.5 * PI
            robot.drive.update(gamepad1, io)

            // ============================================================
            // Mode toggle (Y)
            // ============================================================
            if (gamepad1.y && !lastY) {
                autoMode = !autoMode
            }
            lastY = gamepad1.y

            // ============================================================
            // Physics estimates toggle (Right Bumper)
            // ============================================================
            if (gamepad1.right_bumper && !lastRightBumper) {
                usePhysicsEstimates = !usePhysicsEstimates
                refit()
            }
            lastRightBumper = gamepad1.right_bumper

            // ============================================================
            // Manual adjustments (only in manual mode)
            // ============================================================
            if (!autoMode) {
                // Flywheel: D-pad left/right
                if (gamepad1.dpad_left && !lastDpadLeft) {
                    flywheelTarget = (flywheelTarget - 25.0).coerceAtLeast(0.0)
                }
                lastDpadLeft = gamepad1.dpad_left

                if (gamepad1.dpad_right && !lastDpadRight) {
                    flywheelTarget = (flywheelTarget + 25.0).coerceAtMost(628.0)
                }
                lastDpadRight = gamepad1.dpad_right

                // Hood: D-pad up/down
                if (gamepad1.dpad_up && !lastDpadUp) {
                    hoodAngleDeg = (hoodAngleDeg + 1.0).coerceAtMost(ShooterConfig.maxAngleDeg)
                }
                lastDpadUp = gamepad1.dpad_up

                if (gamepad1.dpad_down && !lastDpadDown) {
                    hoodAngleDeg = (hoodAngleDeg - 1.0).coerceAtLeast(ShooterConfig.minAngleDeg)
                }
                lastDpadDown = gamepad1.dpad_down
            }

            // ============================================================
            // Auto mode: set hood + flywheel from fitted surfaces
            // ============================================================
            if (autoMode) {
                val c = coeffs.get()
                if (c != null) {
                    val result = fitter.distanceToShooterParams(
                        robot.aim.targetDistance, c
                    )
                    if (result != null) {
                        flywheelTarget = result.rpm * RAD_S_PER_RPM
                        hoodAngleDeg = result.hoodAngle
                    }
                }
            }

            // ============================================================
            // Apply shooter values
            // ============================================================
            robot.shooter.manualHoodAngle = Math.toRadians(hoodAngleDeg)

            // ============================================================
            // Intake / shoot / reverse (single prioritised chain)
            // ============================================================
            when {
                // Shoot: highest priority
                gamepad1.right_trigger > 0.1 -> {
                    robot.shooter.flywheelTarget = flywheelTarget
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                    shotStartTime = io.time()
                    lastShotRads = flywheelTarget
                    lastShotDist = robot.aim.targetDistance
                    lastShotHood = hoodAngleDeg
                    shotActive = true
                }
                // Reverse intake
                gamepad1.b -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.intakeTransfer.state = IntakeTransfer.State.REVERSING
                    shotActive = false
                }
                // Spin up flywheel only
                gamepad1.left_bumper -> {
                    robot.shooter.flywheelTarget = flywheelTarget
                    if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING) {
                        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                    }
                    shotActive = false
                }
                // Intake
                gamepad1.left_trigger > 0.1 -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.intakeCoordinator.startIntake()
                    shotActive = false
                }
                // Idle
                else -> {
                    robot.shooter.flywheelTarget = 0.0
                    if (shotActive && (io.time() - shotStartTime).inWholeMilliseconds >= SHOT_DURATION_MS) {
                        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                        shotActive = false
                    }
                    if (!shotActive) {
                        if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING ||
                            robot.intakeTransfer.state == IntakeTransfer.State.INTAKING ||
                            robot.intakeTransfer.state == IntakeTransfer.State.REVERSING
                        ) {
                            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                        }
                    }
                }
            }

            // ============================================================
            // Log data point (A)
            // ============================================================
            if (gamepad1.a && !lastA) {
                val dist = robot.aim.targetDistance
                val speedRadS = robot.shooter.flywheelTarget
                val hood = Math.toDegrees(robot.shooter.computedHoodAngle)

                dataStore.addPoint(SpeedPoint(dist, speedRadS, hood))
                dataStore.save()

                lastLoggedRpm = lastShotRads
                lastLoggedHood = lastShotHood
                lastLoggedDist = lastShotDist

                refit()
            }
            lastA = gamepad1.a

            // ============================================================
            // Delete last data point (X)
            // ============================================================
            if (gamepad1.x && !lastX) {
                val points = dataStore.getPoints()
                if (points.isNotEmpty()) {
                    dataStore.removePoint(points.size - 1)
                    dataStore.save()
                    refit()
                }
            }
            lastX = gamepad1.x

            // ============================================================
            // Robot update
            // ============================================================
            robot.update()
            io.update()

            // ============================================================
            // Telemetry
            // ============================================================
            val fusedPose = robot.aim.autoAim.fusedPose
            val flywheelVel = io.flywheelVelocity()
            val flywheelRPM = flywheelVel / RAD_S_PER_RPM

            telemetry.addLine("=== POSITION ===")
            telemetry.addData("Fused Pose", "x=%.3f y=%.3f th=%.1f",
                fusedPose.v.x, fusedPose.v.y, Math.toDegrees(fusedPose.rot))
            telemetry.addData("Goal Distance", "%.2f m", robot.aim.targetDistance)

            telemetry.addLine("")
            telemetry.addLine("=== SHOOTER ===")
            telemetry.addData("Flywheel", "%.0f RPM (%.0f rad/s)", flywheelRPM, flywheelVel)
            telemetry.addData("FW Target", "%.0f rad/s", flywheelTarget)
            telemetry.addData("Hood Angle", "%.1f deg", hoodAngleDeg)
            telemetry.addData("Hood Servo", "%.3f", robot.shooter.hoodServoPosition)
            telemetry.addData("Mode", if (autoMode) "AUTO (fitted)" else "MANUAL")

            telemetry.addLine("")
            telemetry.addLine("=== TUNER ===")
            val physicsCount = dataStore.getPhysicsEstimateCount()
            val empiricalCount = dataStore.getEmpiricalCount()
            val activePoints = dataStore.getPoints(includePhysicsEstimates = usePhysicsEstimates).size
            telemetry.addData("Physics Estimates", if (usePhysicsEstimates) "ON ($physicsCount pts)" else "OFF")
            telemetry.addData("Data Points", "$empiricalCount empirical + $physicsCount physics = ${empiricalCount + physicsCount} total")
            telemetry.addData("Active (fitting)", activePoints)
            if (lastLoggedDist > 0.0) {
                telemetry.addData("Last Logged",
                    "RPM=%.0f Hood=%.1f Dist=%.2fm", lastLoggedRpm, lastLoggedHood, lastLoggedDist)
            }
            telemetry.addData("Fit Status", when {
                fitting.get() -> "FITTING..."
                coeffs.get() != null -> "OK"
                activePoints < 3 -> "NEED ${3 - activePoints} MORE POINTS"
                else -> "NOT FIT"
            })

            telemetry.addLine("")
            telemetry.addLine("=== STATUS ===")
            telemetry.addData("Balls", robot.beamBreak.ballCount)
            telemetry.addData("Loop", "%d ms", dt.inWholeMilliseconds)

            telemetry.addLine("")
            telemetry.addLine("A=log  X=undo  Y=mode  RB=physics  dpad=adjust  RT=shoot")
            telemetry.update()

            false
        }

        fitScope.cancel()
        robot.close()
    }
}
