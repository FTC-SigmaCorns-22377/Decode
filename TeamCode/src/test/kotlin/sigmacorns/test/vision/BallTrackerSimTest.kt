package sigmacorns.test.vision

import org.joml.Vector2d
import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import sigmacorns.math.Pose2d
import sigmacorns.vision.sim.SimCameraConfig
import sigmacorns.vision.sim.SimulatedCamera
import sigmacorns.vision.tracker.CameraExtrinsicsR
import sigmacorns.vision.tracker.Gating
import sigmacorns.vision.tracker.Intrinsics
import sigmacorns.vision.tracker.KalmanTrack
import sigmacorns.vision.tracker.PixelDetection
import sigmacorns.vision.tracker.PoseBuffer
import sigmacorns.vision.tracker.Tracker
import sigmacorns.vision.tracker.TrackerConfig
import java.io.File
import java.util.Random
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * End-to-end tracker sim. Seven scenarios exercise the full
 * detection → project → gate → associate → KF pipeline against known ground
 * truth. All scenarios run headless by default; the `BALL_TRACKER_VIZ_WEB`
 * and `BALL_TRACKER_VIZ_LWJGL` env flags are wired in VizPublisher (follow-up
 * commit) — they are intentionally no-ops here.
 *
 * Each scenario emits a per-tick CSV under `build/reports/ball_tracker/` so
 * regressions show up as RMS curves, not just booleans.
 */
@DisplayName("BallTrackerSim")
class BallTrackerSimTest {

    enum class Noise { ZERO, PIXEL_ONLY, PIXEL_AND_POSE, FULL }

    // --- Shared scaffolding -------------------------------------------------

    private val baseIntrinsics = Intrinsics(fx = 900.0, fy = 900.0, cx = 640.0, cy = 360.0)
    private val imageW = 1280
    private val imageH = 720

    // Pitch 0.15 rad keeps a ball 0.7-2 m away near image center, which is the
    // regime the tracker's covariance is well-behaved in. At closer range
    // (<0.45 m) the ball drops below the image edge — the chase scenario relies
    // on coasting toward the last-known estimate, not fresh detections.
    private val baseExtrinsics = CameraExtrinsicsR(
        camPosR = Vector3d(0.0, 0.0, 0.30),
        pitchDownRad = 0.15,
        yawRad = 0.0,
        rollRad = 0.0,
    )

    private fun baseTrackerConfig(
        rampPolygon: List<Vector2d> = emptyList(),
    ) = TrackerConfig(
        // Synthetic tracker-only field: generously sized so every scenario's
        // placement (balls up to ~2.5 m from origin) stays inside the gate.
        // The real 3.66 x 3.66 field is used in the chase-viz harness.
        fieldWidthM = 6.0,
        fieldHeightM = 6.0,
        fieldMarginM = 0.02,
        rampPolygon = rampPolygon,
        rampExpandM = 0.10,
        ballRadiusM = 0.035,
        intrinsics = baseIntrinsics,
        imageWidthPx = imageW,
        imageHeightPx = imageH,
        cameraExtrinsics = baseExtrinsics,
        robotFootprintR = emptyList(),
        intakeMaskYMinFrac = 0.95,   // near-zero mask for sim
        sigmaPx = 1.5,
        maxRangeM = 8.0,
        sigmaAMps2 = 3.0,
        initVelVar = 1.0,
        chi2Gate = 9.21,
        maxCoastFrames = 15,
        minHitsForConfirmed = 3,
    )

    private data class ScenarioResult(
        val name: String,
        val durationSec: Double,
        val dt: Double,
        val rmsErrorLast3s: Double,
        val framesWithTrack: Int,
        val totalFrames: Int,
        val anyTrackInsideRamp: Boolean,
        val reacquireGapBridged: Boolean,
        val finalRobotToBallDistance: Double,
    ) {
        val trackPersistence: Double get() =
            if (totalFrames == 0) 0.0 else framesWithTrack.toDouble() / totalFrames
    }

    private fun openCsv(name: String): File {
        val outDir = File(
            System.getProperty("projectDir") ?: System.getProperty("user.dir"),
            "build/reports/ball_tracker",
        )
        outDir.mkdirs()
        val f = File(outDir, "$name.csv")
        f.writeText("t,truthX,truthY,estX,estY,errM,covTrace,targetId,tracksN\n")
        return f
    }

    /**
     * Kinematic scenario runner. Pose over time is an explicit function
     * (no JoltSim dependency) — the tracker doesn't need physical dynamics
     * to be validated, it needs real forward projection + accurate pose
     * interpolation. JoltSim-backed scenarios can be added in a follow-up.
     */
    private fun runScenario(
        name: String,
        noise: Noise,
        ballTruth: (Double) -> Vector3d?,
        robotPose: (Double) -> Pose2d,
        durationSec: Double,
        dt: Double = 1.0 / 50.0,
        rampPolygon: List<Vector2d> = emptyList(),
        targetPersistenceCheck: Boolean = false,
        reacquireGapWindow: Pair<Double, Double>? = null,
    ): ScenarioResult {
        val cfg = baseTrackerConfig(rampPolygon = rampPolygon)
        val tracker = Tracker(cfg)
        val poseBuffer = PoseBuffer()

        val sigmaPxSim = when (noise) {
            Noise.ZERO -> 0.0
            Noise.PIXEL_ONLY, Noise.PIXEL_AND_POSE -> 1.5
            Noise.FULL -> 1.5
        }
        val pDrop = if (noise == Noise.FULL) 0.15 else 0.0
        val pFp = if (noise == Noise.FULL) 0.05 else 0.0
        val posePosNoise = if (noise == Noise.PIXEL_AND_POSE || noise == Noise.FULL) 0.01 else 0.0
        val poseHeadingNoise = if (noise == Noise.PIXEL_AND_POSE || noise == Noise.FULL) 0.005 else 0.0

        val simCam = SimulatedCamera(
            SimCameraConfig(
                intrinsics = baseIntrinsics,
                imageWidthPx = imageW,
                imageHeightPx = imageH,
                extrinsics = baseExtrinsics,
                sigmaPxSim = sigmaPxSim,
                pDrop = pDrop,
                pFalsePositive = pFp,
                rngSeed = 7L,
            )
        )
        val rng = Random(11L)
        val csv = openCsv(name)

        var framesWithTrack = 0
        var totalFrames = 0
        var anyTrackInsideRamp = false
        var reacquireGapBridged = true   // default true; flipped false if the track dies

        // For RMS over the last 3 seconds.
        val errBuf = mutableListOf<Pair<Double, Double>>()  // (t, err)

        var preGapTrackId: Int? = null
        var finalRobotToBall = Double.NaN

        var t = 0.0
        while (t <= durationSec + 1e-9) {
            val truth = ballTruth(t)
            val poseTrue = robotPose(t)

            poseBuffer.add(t, poseTrue)

            // Noisy pose that the tracker sees (pose-noise model).
            val noisyPose = if (posePosNoise > 0.0 || poseHeadingNoise > 0.0) {
                Pose2d(
                    poseTrue.v.x + rng.nextGaussian() * posePosNoise,
                    poseTrue.v.y + rng.nextGaussian() * posePosNoise,
                    poseTrue.rot + rng.nextGaussian() * poseHeadingNoise,
                )
            } else poseTrue

            // Generate detections from truth positions through the TRUE camera transform
            // (pose noise is what the tracker sees, not what the camera sees).
            val detections: List<PixelDetection> = if (truth == null) emptyList()
            else simCam.frame(listOf(truth), t, poseTrue)

            val tracks = tracker.tick(detections, noisyPose, t)
            val target = tracker.selectTarget(noisyPose, t)

            totalFrames++
            val hasConfirmed = target != null
            if (hasConfirmed) framesWithTrack++

            // Ramp-leakage check (any track whose position sits inside the ramp).
            if (rampPolygon.isNotEmpty()) {
                for (tr in tracks) {
                    if (Gating.insideRamp(tr.position(), rampPolygon, 0.0)) {
                        anyTrackInsideRamp = true
                    }
                }
            }

            // In-and-out-of-frame check: record the target track id before the gap,
            // assert the same track is still alive after the gap.
            if (reacquireGapWindow != null) {
                val (gapStart, gapEnd) = reacquireGapWindow
                val inGap = t in gapStart..gapEnd
                if (!inGap) {
                    val id = target?.id
                    if (t < gapStart && id != null) preGapTrackId = id
                    if (t > gapEnd && preGapTrackId != null) {
                        val stillAlive = tracks.any { it.id == preGapTrackId }
                        if (!stillAlive) reacquireGapBridged = false
                    }
                }
            }

            // RMS should only grade the CONFIRMED target track, never a spurious
            // unconfirmed track from a false-positive detection.
            val bestTrack: KalmanTrack? = target
            val est = bestTrack?.positionAt(t)
            val err = if (truth != null && est != null)
                sqrt((truth.x - est.x) * (truth.x - est.x) + (truth.y - est.y) * (truth.y - est.y))
            else Double.NaN

            if (!err.isNaN()) errBuf.add(t to err)

            if (truth != null) {
                val dx = truth.x - poseTrue.v.x
                val dy = truth.y - poseTrue.v.y
                finalRobotToBall = sqrt(dx * dx + dy * dy)
            }

            val covTrace = bestTrack?.positionCovTrace() ?: Double.NaN
            csv.appendText(
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%s,%d\n".format(
                    t,
                    truth?.x ?: Double.NaN,
                    truth?.y ?: Double.NaN,
                    est?.x ?: Double.NaN,
                    est?.y ?: Double.NaN,
                    err,
                    covTrace,
                    target?.id?.toString() ?: "",
                    tracks.size,
                )
            )

            t += dt
        }

        val last3sStart = durationSec - 3.0
        val last3s = errBuf.filter { it.first >= last3sStart && !it.second.isNaN() }
        val rms = if (last3s.isEmpty()) Double.NaN
        else sqrt(last3s.sumOf { it.second * it.second } / last3s.size)

        val result = ScenarioResult(
            name = name,
            durationSec = durationSec,
            dt = dt,
            rmsErrorLast3s = rms,
            framesWithTrack = framesWithTrack,
            totalFrames = totalFrames,
            anyTrackInsideRamp = anyTrackInsideRamp,
            reacquireGapBridged = reacquireGapBridged,
            finalRobotToBallDistance = finalRobotToBall,
        )
        // One-line pass/fail friendly summary to stdout.
        println(
            "[SIM] %-32s rms=%.4fm  persist=%.1f%% (%d/%d)  rampLeak=%s  gapOK=%s".format(
                name, rms,
                100.0 * result.trackPersistence, framesWithTrack, totalFrames,
                anyTrackInsideRamp, reacquireGapBridged,
            )
        )
        return result
    }

    // --- Scenarios ----------------------------------------------------------

    @Test
    @DisplayName("zero-noise stationary ball — RMS < 1e-3 m")
    fun zeroNoiseStationaryBall() {
        val r = runScenario(
            name = "zeroNoiseStationaryBall",
            noise = Noise.ZERO,
            ballTruth = { Vector3d(1.5, 1.8, 0.035) },
            robotPose = { Pose2d(0.8, 1.8, 0.0) },
            durationSec = 5.0,
        )
        assertTrue(r.rmsErrorLast3s < 1e-3, "RMS too high: ${r.rmsErrorLast3s}")
    }

    @Test
    @DisplayName("pixel-only noise — RMS < 2e-2 m")
    fun pixelNoiseOnly() {
        val r = runScenario(
            name = "pixelNoiseOnly",
            noise = Noise.PIXEL_ONLY,
            ballTruth = { Vector3d(1.5, 1.8, 0.035) },
            robotPose = { Pose2d(0.8, 1.8, 0.0) },
            durationSec = 5.0,
        )
        assertTrue(r.rmsErrorLast3s < 2e-2, "RMS too high: ${r.rmsErrorLast3s}")
    }

    @Test
    @DisplayName("pixel+pose noise — RMS < 5e-2 m")
    fun pixelAndPoseNoise() {
        val r = runScenario(
            name = "pixelAndPoseNoise",
            noise = Noise.PIXEL_AND_POSE,
            ballTruth = { Vector3d(2.0, 1.8, 0.035) },
            robotPose = { t ->
                // Slow circular motion so pose noise matters.
                val cx = 0.8
                val cy = 1.8
                val w = 0.2
                Pose2d(cx + 0.1 * cos(w * t), cy + 0.1 * sin(w * t), 0.0)
            },
            durationSec = 6.0,
        )
        assertTrue(r.rmsErrorLast3s < 5e-2, "RMS too high: ${r.rmsErrorLast3s}")
    }

    @Test
    @DisplayName("full noise (dropout + FP) — RMS < 10e-2 m AND target track persists >= 95%")
    fun fullNoiseWithDropoutAndFP() {
        val r = runScenario(
            name = "fullNoiseWithDropoutAndFP",
            noise = Noise.FULL,
            ballTruth = { Vector3d(1.5, 1.8, 0.035) },
            robotPose = { Pose2d(0.8, 1.8, 0.0) },
            durationSec = 8.0,
        )
        assertTrue(r.rmsErrorLast3s < 10e-2, "RMS too high: ${r.rmsErrorLast3s}")
        assertTrue(r.trackPersistence >= 0.95, "target track persistence too low: ${r.trackPersistence}")
    }

    @Test
    @DisplayName("ball on ramp — zero tracks inside ramp polygon")
    fun ballOnRampRejected() {
        val ramp = listOf(
            Vector2d(1.7, 1.5),
            Vector2d(2.3, 1.5),
            Vector2d(2.3, 2.1),
            Vector2d(1.7, 2.1),
        )
        val r = runScenario(
            name = "ballOnRampRejected",
            noise = Noise.PIXEL_ONLY,
            ballTruth = { Vector3d(2.0, 1.8, 0.035) },    // inside the ramp
            robotPose = { Pose2d(0.8, 1.8, 0.0) },
            durationSec = 4.0,
            rampPolygon = ramp,
        )
        assertTrue(!r.anyTrackInsideRamp, "a track leaked inside the ramp polygon")
    }

    @Test
    @DisplayName("ball in and out of frame — track survives a 10-frame gap and re-locks")
    fun ballInAndOutOfFrame() {
        // Gap: ball absent between 2.0 s and 2.2 s (10 frames at 50 Hz).
        val gapStart = 2.0
        val gapEnd = 2.2
        val r = runScenario(
            name = "ballInAndOutOfFrame",
            noise = Noise.PIXEL_ONLY,
            ballTruth = { t -> if (t in gapStart..gapEnd) null else Vector3d(2.0, 1.8, 0.035) },
            robotPose = { Pose2d(0.8, 1.8, 0.0) },
            durationSec = 5.0,
            reacquireGapWindow = gapStart to gapEnd,
        )
        assertTrue(r.reacquireGapBridged, "track died across the ball-absence gap")
    }

    @Test
    @DisplayName("headless chase — robot reaches ball within 15 cm in 5 s sim time, PIXEL_AND_POSE noise")
    fun headlessChaseScenario() {
        // Minimal chase controller: a live robot-pose integrator that moves
        // toward the selected target each tick at 1 m/s. The scenario runner
        // doesn't expose a velocity model, so we run the chase loop inline.
        val cfg = baseTrackerConfig()
        val tracker = Tracker(cfg)
        val sim = SimulatedCamera(
            SimCameraConfig(
                intrinsics = baseIntrinsics,
                imageWidthPx = imageW, imageHeightPx = imageH,
                extrinsics = baseExtrinsics,
                sigmaPxSim = 1.5, rngSeed = 3L,
            )
        )
        val rng = Random(4L)
        val posePosNoise = 0.01
        val poseHeadingNoise = 0.005

        val truth = Vector3d(2.5, 1.8, 0.035)
        var pose = Pose2d(0.5, 1.8, 0.0)     // robot starts 2 m in front of ball

        val dt = 1.0 / 50.0
        val maxT = 5.0
        val maxSpeed = 1.0

        val csv = openCsv("headlessChaseScenario")
        var t = 0.0
        var lastKnownTarget: Vector2d? = null
        while (t <= maxT) {
            val noisyPose = Pose2d(
                pose.v.x + rng.nextGaussian() * posePosNoise,
                pose.v.y + rng.nextGaussian() * posePosNoise,
                pose.rot + rng.nextGaussian() * poseHeadingNoise,
            )
            val detections = sim.frame(listOf(truth), t, pose)
            tracker.tick(detections, noisyPose, t)
            val target = tracker.selectTarget(noisyPose, t)

            // Only trust the target estimate when its position covariance is tight.
            // Near-horizon projections can blow up transiently; accepting those
            // updates lets the controller steer toward a bad last-known position
            // after the track dies.
            if (target != null && target.positionCovTrace() < 0.05) {
                val p = target.positionAt(t)
                lastKnownTarget = Vector2d(p.x, p.y)
            }

            val steer = lastKnownTarget
            if (steer != null) {
                val dx = steer.x - pose.v.x
                val dy = steer.y - pose.v.y
                val d = sqrt(dx * dx + dy * dy)
                // Once within 5 cm per the estimate, hold position so the ball
                // falling out of the image (it drops off the bottom at close range)
                // doesn't let a coasting track drift the robot sideways.
                if (d > 0.05) {
                    val speed = (maxSpeed * (d / 0.30)).coerceIn(0.0, maxSpeed)
                    val vx = speed * dx / d
                    val vy = speed * dy / d
                    pose = Pose2d(pose.v.x + vx * dt, pose.v.y + vy * dt, pose.rot)
                }
            }

            val dxTrue = truth.x - pose.v.x
            val dyTrue = truth.y - pose.v.y
            val robotToBall = sqrt(dxTrue * dxTrue + dyTrue * dyTrue)
            csv.appendText(
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%s,%d\n".format(
                    t, truth.x, truth.y, pose.v.x, pose.v.y, robotToBall,
                    target?.positionCovTrace() ?: Double.NaN,
                    target?.id?.toString() ?: "",
                    tracker.tracks.size,
                )
            )
            t += dt
        }
        val dxFinal = truth.x - pose.v.x
        val dyFinal = truth.y - pose.v.y
        val dFinal = sqrt(dxFinal * dxFinal + dyFinal * dyFinal)
        println("[SIM] %-32s final=%.4fm".format("headlessChaseScenario", dFinal))
        assertTrue(dFinal < 0.15, "robot did not reach ball within 15 cm: final=$dFinal m")
    }
}
