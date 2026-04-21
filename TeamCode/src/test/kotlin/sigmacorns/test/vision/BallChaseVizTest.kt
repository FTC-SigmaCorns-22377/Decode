package sigmacorns.test.vision

import org.joml.Vector2d
import org.joml.Vector3d
import org.junit.jupiter.api.Test
import sigmacorns.io.JoltSimIO
import sigmacorns.logic.BallChaseAutoFSM
import sigmacorns.logic.BallTrackingSystem
import sigmacorns.math.Pose2d
import sigmacorns.sim.viz.SimVizServer
import sigmacorns.subsystem.Drivetrain
import sigmacorns.vision.sim.SimCameraConfig
import sigmacorns.vision.sim.SimulatedCamera
import sigmacorns.vision.tracker.Gating
import sigmacorns.vision.tracker.Projection
import sigmacorns.vision.tracker.TrackerConfig
import sigmacorns.vision.viz.VizStateBuilder
import kotlin.math.sqrt

/**
 * Interactive ball-tracker demo on the Jolt sim + web viewer.
 *
 *   1. Boot JoltSimIO, spawn field balls.
 *   2. Load TrackerConfig from config/ball_tracker.json (with sim-friendly
 *      pitch override — see `simExtrinsics` below).
 *   3. Attach a SimulatedCamera to the Jolt IO; build BallTrackingSystem +
 *      ChaseCoordinator on top of it.
 *   4. Wire the SimVizServer `trackerStateProvider` + `onScenarioChange` so
 *      the browser UI can flip noise profiles live.
 *   5. Main loop at 50 Hz (wall-clock): step sim, tracker.update, chase.update,
 *      broadcast.
 *
 * Controls:
 *   WASD  — drive robot (taken from SimVizServer gamepad1 mapping)
 *   Q/E   — yaw
 *   SPACE — toggle auto-chase (pressed once; held is ignored)
 *
 * The test runs until the process is killed — like the other *VisualizerTest
 * classes, it is interactive, not assertion-bearing.
 */
class BallChaseVizTest {

    private enum class Scenario(
        val sigmaPxSim: Double,
        val pDrop: Double,
        val pFalsePositive: Double,
    ) {
        LIVE(0.8, 0.0, 0.0),
        PIXEL_ONLY(1.5, 0.0, 0.0),
        PIXEL_HEAVY(3.0, 0.0, 0.0),
        DROPOUT(1.5, 0.3, 0.0),
        FALSE_POSITIVES(1.5, 0.0, 0.2),
        FULL_NOISE(1.5, 0.2, 0.1);

        companion object {
            fun fromNameOrNull(name: String?): Scenario? =
                values().firstOrNull { it.name.equals(name, ignoreCase = true) }
        }
    }

    // Override the on-robot extrinsics for sim only. The real robot mount is
    // unverified and the sim test uses a generic downward pitch so the ball
    // layout projects nicely into the image.
    private fun simExtrinsics() = sigmacorns.vision.tracker.CameraExtrinsicsR(
        camPosR = Vector3d(0.0, 0.0, 0.30),
        pitchDownRad = 0.15,
        yawRad = 0.0,
        rollRad = 0.0,
    )

    @Test
    fun runBallChaseSim() {
        val sim = JoltSimIO(realTime = false)
        sim.spawnFieldBalls()
        sim.setPosition(Pose2d(0.0, 0.0, 0.0))

        val loadedConfig = TrackerConfig.loadDefault()
        val cfg = loadedConfig.copy(
            cameraExtrinsics = simExtrinsics(),
            intakeMaskYMinFrac = 0.95,  // near-zero mask in sim
            rampPolygon = emptyList(),
            // Faster lock-on during a search sweep: one confirmed hit is enough
            // in sim since we trust Jolt's ball positions. Hardware stays at 3.
            minHitsForConfirmed = 1,
        )

        val tracking = BallTrackingSystem(config = cfg, io = sim)
        val drivetrain = Drivetrain()

        // Shooting zone: position near the red goal facing it. Field frame.
        val goal = sigmacorns.constants.FieldLandmarks.redGoalPosition
        val shootingZone = Pose2d(
            goal.x - 1.0, goal.y - 0.6,
            kotlin.math.atan2(goal.y - (goal.y - 0.6), goal.x - (goal.x - 1.0)),
        )

        val fsm = BallChaseAutoFSM(
            tracking = tracking,
            drivetrain = drivetrain,
            io = sim,
            shootingZone = shootingZone,
            maxSpeed = 1.0,
            arrivalRadiusM = 0.18,
            chasePullInRadiusM = 0.35,
            flywheelShootSpeed = 1.0,
            heldBallCountFn = { sim.heldBalls.size },
        )

        val server = SimVizServer(sim)
        val sigmaAtomic = java.util.concurrent.atomic.AtomicReference(Scenario.LIVE)

        fun applyScenario(s: Scenario) {
            sigmaAtomic.set(s)
            sim.simulatedCamera = SimulatedCamera(
                SimCameraConfig(
                    intrinsics = cfg.intrinsics,
                    imageWidthPx = cfg.imageWidthPx,
                    imageHeightPx = cfg.imageHeightPx,
                    extrinsics = cfg.cameraExtrinsics,
                    sigmaPxSim = s.sigmaPxSim,
                    pDrop = s.pDrop,
                    pFalsePositive = s.pFalsePositive,
                    rngSeed = System.nanoTime(),
                )
            )
        }
        applyScenario(Scenario.LIVE)

        server.onScenarioChange = { name ->
            Scenario.fromNameOrNull(name)?.let {
                applyScenario(it)
                println("[BallChaseViz] scenario -> $it")
            }
        }
        server.onChaseToggle = { on ->
            fsm.enabled = on
            println("[BallChaseViz] autoFSM -> $on")
        }
        server.onReset = {
            // Order matters: disable the FSM first so it stops commanding
            // the drivetrain / subsystems during the respawn. Then zero
            // the world state, re-place the robot, and clear the tracker.
            fsm.enabled = false
            sim.driveFL = 0.0; sim.driveBL = 0.0; sim.driveFR = 0.0; sim.driveBR = 0.0
            sim.intake = 0.0
            sim.flywheel = 0.0
            sim.blocker = sigmacorns.subsystem.IntakeTransfer.BLOCKER_ENGAGED
            sim.clearAllBalls()
            sim.spawnFieldBalls()
            sim.setPosition(Pose2d(0.0, 0.0, 0.0))
            tracking.reset()
            println("[BallChaseViz] reset complete.")
        }
        server.trackerStateProvider = {
            val t = sim.time().inWholeMilliseconds / 1000.0
            val pose = sim.position()
            val rawDetectionsPx = tracking.lastDetectionsPx
            val ballStates = sim.getBallStates()
            val ballsTruth = ballStates.map { Vector3d(it.x.toDouble(), it.y.toDouble(), it.z.toDouble()) }
            val ballsColors: List<String?> = ballStates.map { it.color.name.lowercase() }

            // Re-run the gating path against live config to populate survivors
            // for the viz. Cheap — same math as Tracker.tick's prefix.
            val T_FC = sigmacorns.vision.tracker.Frames.buildTFC(pose, cfg.TRC)
            val survivingPos = mutableListOf<Vector2d>()
            val survivingCov = mutableListOf<DoubleArray>()
            for (d in rawDetectionsPx) {
                if (Gating.insideIntakeMask(d.v, cfg.imageHeightPx, cfg.intakeMaskYMinFrac)) continue
                val result = Projection.projectToGroundWithCovariance(
                    d.u, d.v, cfg.intrinsics, T_FC,
                    cfg.ballRadiusM, cfg.maxRangeM, cfg.sigmaPx,
                ) ?: continue
                val p = result.point
                if (!Gating.insideField(p, cfg.fieldWidthM, cfg.fieldHeightM, cfg.fieldMarginM)) continue
                if (Gating.insideRamp(p, cfg.rampPolygon, cfg.rampExpandM)) continue
                if (Gating.insideRobotFootprint(p, pose, cfg.robotFootprintR)) continue
                survivingPos.add(Vector2d(p))
                survivingCov.add(result.cov)
            }

            val target = tracking.target
            val rms = target?.let { tr ->
                val truthPos = ballsTruth.minByOrNull { bt ->
                    val dx = bt.x - tr.position().x; val dy = bt.y - tr.position().y
                    dx * dx + dy * dy
                } ?: return@let null
                val dx = truthPos.x - tr.position().x
                val dy = truthPos.y - tr.position().y
                sqrt(dx * dx + dy * dy)
            }

            VizStateBuilder.build(
                scenario = sigmaAtomic.get().name,
                rmsErrorM = rms,
                tracker = tracking.tracker,
                config = cfg,
                robotPose = pose,
                ballsTruthField = ballsTruth,
                ballsTruthColors = ballsColors,
                rawDetectionsPx = rawDetectionsPx,
                survivingFieldDetections = survivingPos,
                survivingFieldDetectionCovs = survivingCov,
                targetId = target?.id,
            )
        }

        server.start()
        println("[BallChaseViz] viewer at http://localhost:8080 — connect a browser to begin.")
        server.awaitClient()
        println("[BallChaseViz] client connected. Starting loop.")

        // Construct pseudo-gamepads for the drivetrain; SimVizServer fills them
        // from the browser each tick.
        val gp1 = com.qualcomm.robotcore.hardware.Gamepad()
        val gp2 = com.qualcomm.robotcore.hardware.Gamepad()

        val dt = 0.02  // 50 Hz
        var lastChaseToggleSent = false
        val wallClockStartNs = System.nanoTime()
        val simClockStart = sim.time().inWholeMilliseconds / 1000.0
        try {
            while (!Thread.currentThread().isInterrupted) {
                server.syncGamepads(gp1, gp2)

                // Toggle auto-FSM with gp1.a (rising edge). Same toggle is
                // also driven by the browser button via onChaseToggle.
                if (gp1.a && !lastChaseToggleSent) {
                    fsm.enabled = !fsm.enabled
                    println("[BallChaseViz] autoFSM = ${fsm.enabled}")
                }
                lastChaseToggleSent = gp1.a

                val tNow = sim.time().inWholeMilliseconds / 1000.0

                // Step tracker first (consumes the latest detections).
                tracking.update(tNow)

                if (fsm.enabled) {
                    fsm.update()
                } else {
                    // Manual driver control.
                    drivetrain.update(gp1, sim)
                }

                // Step physics.
                sim.update()

                server.broadcastState()

                // Wall-clock pacing.
                val targetElapsedMs = ((tNow - simClockStart + dt) * 1000).toLong()
                val elapsedMs = (System.nanoTime() - wallClockStartNs) / 1_000_000
                val sleepMs = targetElapsedMs - elapsedMs
                if (sleepMs > 0) Thread.sleep(sleepMs)
            }
        } finally {
            server.stop()
            sim.close()
        }
    }
}
