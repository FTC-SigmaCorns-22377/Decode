package sigmacorns.test.vision

import org.joml.Matrix4d
import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.CameraExtrinsicsR
import sigmacorns.vision.tracker.Frames
import sigmacorns.vision.tracker.Intrinsics
import sigmacorns.vision.tracker.PixelDetection
import sigmacorns.vision.tracker.Projection
import sigmacorns.vision.tracker.Tracker
import sigmacorns.vision.tracker.TrackerConfig

@DisplayName("Tracker")
class TrackerTest {

    private fun makeConfig(): TrackerConfig {
        // Looking forward and down. Camera 0.3 m above the floor, pitched down 25 deg.
        return TrackerConfig(
            fieldWidthM = 3.6576,
            fieldHeightM = 3.6576,
            fieldMarginM = 0.02,
            rampPolygon = emptyList(),
            rampExpandM = 0.0,
            ballRadiusM = 0.035,
            intrinsics = Intrinsics(fx = 900.0, fy = 900.0, cx = 640.0, cy = 360.0),
            imageWidthPx = 1280,
            imageHeightPx = 720,
            cameraExtrinsics = CameraExtrinsicsR(
                camPosR = Vector3d(0.0, 0.0, 0.30),
                pitchDownRad = 0.25,
                yawRad = 0.0,
                rollRad = 0.0,
            ),
            robotFootprintR = emptyList(),
            intakeMaskYMinFrac = 0.95,   // near-zero mask, don't accidentally kill test pixels
            sigmaPx = 1.0,
            maxRangeM = 20.0,
            sigmaAMps2 = 3.0,
            initVelVar = 1.0,
            chi2Gate = 9.21,
            maxCoastFrames = 5,
            minHitsForConfirmed = 2,
        )
    }

    /** Project a field-frame ground point (z = ballRadius) through the camera to a pixel. */
    private fun pixelFor(cfg: TrackerConfig, robotPose: Pose2d, fx: Double, fy: Double): DoubleArray {
        val T_FC: Matrix4d = Frames.buildTFC(robotPose, cfg.TRC)
        val uv = Projection.forwardProject(Vector3d(fx, fy, cfg.ballRadiusM), cfg.intrinsics, T_FC)
        return uv ?: error("forwardProject returned null for ($fx, $fy)")
    }

    @Test
    @DisplayName("spawns a track on a clean detection at the ball height")
    fun spawnsOnCleanDetection() {
        val cfg = makeConfig()
        val tr = Tracker(cfg)
        val pose = Pose2d(1.0, 1.0, 0.0)
        val ball = Pose2d(2.5, 1.0, 0.0)     // 1.5 m in front of robot
        val (u, v) = pixelFor(cfg, pose, ball.v.x, ball.v.y)
        val t0 = 0.1
        val tracks = tr.tick(listOf(PixelDetection(u, v, t0)), pose, t0)
        assertEquals(1, tracks.size)
        val p = tracks[0].position()
        assertEquals(ball.v.x, p.x, 0.01)
        assertEquals(ball.v.y, p.y, 0.01)
    }

    @Test
    @DisplayName("associates successive detections into one track")
    fun associatesSuccessiveDetections() {
        val cfg = makeConfig()
        val tr = Tracker(cfg)
        val pose = Pose2d(0.5, 1.0, 0.0)
        val truth = Pose2d(2.0, 1.0, 0.0)
        var t = 0.0
        repeat(10) { i ->
            t = (i + 1) * 0.05
            val (u, v) = pixelFor(cfg, pose, truth.v.x, truth.v.y)
            tr.tick(listOf(PixelDetection(u, v, t)), pose, t)
        }
        assertEquals(1, tr.tracks.size)
        assertTrue(tr.tracks[0].hits >= cfg.minHitsForConfirmed)
    }

    @Test
    @DisplayName("two separated balls produce two tracks")
    fun twoBallsTwoTracks() {
        val cfg = makeConfig()
        val tr = Tracker(cfg)
        val pose = Pose2d(0.5, 1.0, 0.0)
        val a = Pose2d(2.0, 0.6, 0.0)
        val b = Pose2d(2.0, 1.4, 0.0)
        var t = 0.0
        repeat(5) { i ->
            t = (i + 1) * 0.05
            val (ua, va) = pixelFor(cfg, pose, a.v.x, a.v.y)
            val (ub, vb) = pixelFor(cfg, pose, b.v.x, b.v.y)
            tr.tick(listOf(PixelDetection(ua, va, t), PixelDetection(ub, vb, t)), pose, t)
        }
        assertEquals(2, tr.tracks.size)
    }

    @Test
    @DisplayName("selectTarget returns the nearest confirmed track")
    fun selectTargetNearestConfirmed() {
        val cfg = makeConfig()
        val tr = Tracker(cfg)
        val pose = Pose2d(0.5, 1.0, 0.0)
        val near = Pose2d(1.5, 1.0, 0.0)
        val far  = Pose2d(3.0, 1.0, 0.0)
        var t = 0.0
        repeat(5) { i ->
            t = (i + 1) * 0.05
            val (un, vn) = pixelFor(cfg, pose, near.v.x, near.v.y)
            val (uf, vf) = pixelFor(cfg, pose, far.v.x, far.v.y)
            tr.tick(
                listOf(PixelDetection(un, vn, t), PixelDetection(uf, vf, t)),
                pose, t,
            )
        }
        val target = tr.selectTarget(pose, t)
        assertNotNull(target)
        val tp = target!!.position()
        // Closer ball is `near`.
        assertEquals(near.v.x, tp.x, 0.02)
    }

    @Test
    @DisplayName("selectTarget is null before any confirmed track exists")
    fun selectTargetNullWithoutConfirmed() {
        val cfg = makeConfig()
        val tr = Tracker(cfg)
        val pose = Pose2d(0.5, 1.0, 0.0)
        val ball = Pose2d(2.0, 1.0, 0.0)
        val (u, v) = pixelFor(cfg, pose, ball.v.x, ball.v.y)
        tr.tick(listOf(PixelDetection(u, v, 0.05)), pose, 0.05)
        assertNull(tr.selectTarget(pose, 0.05))
    }

    @Test
    @DisplayName("track deleted after MAX_COAST_FRAMES misses")
    fun trackDeletedAfterCoast() {
        val cfg = makeConfig()
        val tr = Tracker(cfg)
        val pose = Pose2d(0.5, 1.0, 0.0)
        val ball = Pose2d(2.0, 1.0, 0.0)
        val (u, v) = pixelFor(cfg, pose, ball.v.x, ball.v.y)
        tr.tick(listOf(PixelDetection(u, v, 0.05)), pose, 0.05)
        assertEquals(1, tr.tracks.size)
        // Now coast — no detections for > maxCoastFrames ticks.
        var t = 0.05
        repeat(cfg.maxCoastFrames + 1) {
            t += 0.05
            tr.tick(emptyList(), pose, t)
        }
        assertEquals(0, tr.tracks.size, "track should be deleted after coasting")
    }

    @Test
    @DisplayName("detection in intake-mask region is dropped before projection")
    fun intakeMaskDropsPixel() {
        val cfg = makeConfig().copy(intakeMaskYMinFrac = 0.3)  // aggressive mask
        val tr = Tracker(cfg)
        val pose = Pose2d(0.5, 1.0, 0.0)
        val ball = Pose2d(2.0, 1.0, 0.0)
        val (u, v) = pixelFor(cfg, pose, ball.v.x, ball.v.y)
        assertTrue(v > cfg.imageHeightPx * 0.3, "test precondition: pixel must land below the mask cutoff")
        val tracks = tr.tick(listOf(PixelDetection(u, v, 0.05)), pose, 0.05)
        assertTrue(tracks.isEmpty(), "intake mask should drop the detection before projection")
    }

    @Test
    @DisplayName("reset clears tracks and id counter")
    fun resetClears() {
        val cfg = makeConfig()
        val tr = Tracker(cfg)
        val pose = Pose2d(0.5, 1.0, 0.0)
        val ball = Pose2d(2.0, 1.0, 0.0)
        val (u, v) = pixelFor(cfg, pose, ball.v.x, ball.v.y)
        tr.tick(listOf(PixelDetection(u, v, 0.05)), pose, 0.05)
        assertEquals(1, tr.tracks.size)
        tr.reset()
        assertEquals(0, tr.tracks.size)
    }
}
