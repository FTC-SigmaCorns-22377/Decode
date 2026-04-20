package sigmacorns.test.vision

import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import sigmacorns.math.Pose2d
import sigmacorns.vision.sim.SimCameraConfig
import sigmacorns.vision.sim.SimulatedCamera
import sigmacorns.vision.tracker.CameraExtrinsicsR
import sigmacorns.vision.tracker.Intrinsics

@DisplayName("SimulatedCamera")
class SimulatedCameraTest {

    private fun cfg(
        sigmaPxSim: Double = 0.0,
        pDrop: Double = 0.0,
        pFalsePositive: Double = 0.0,
        seed: Long = 42L,
    ) = SimCameraConfig(
        intrinsics = Intrinsics(fx = 900.0, fy = 900.0, cx = 640.0, cy = 360.0),
        imageWidthPx = 1280,
        imageHeightPx = 720,
        extrinsics = CameraExtrinsicsR(
            camPosR = Vector3d(0.0, 0.0, 0.30),
            pitchDownRad = 0.25,
            yawRad = 0.0,
            rollRad = 0.0,
        ),
        sigmaPxSim = sigmaPxSim,
        pDrop = pDrop,
        pFalsePositive = pFalsePositive,
        rngSeed = seed,
    )

    @Test
    @DisplayName("projects a ball in front of the robot to a pixel inside the image")
    fun projectsVisibleBall() {
        val cam = SimulatedCamera(cfg())
        val detections = cam.frame(
            ballsField = listOf(Vector3d(2.0, 1.0, 0.035)),
            t = 0.1,
            robotPose = Pose2d(0.5, 1.0, 0.0),
        )
        assertEquals(1, detections.size)
        val d = detections[0]
        assertTrue(d.u in 0.0..1280.0 && d.v in 0.0..720.0, "pixel out of bounds: ($d.u, $d.v)")
        assertEquals(0.1, d.t, 0.0)
    }

    @Test
    @DisplayName("skips a ball behind the camera")
    fun skipsBallBehindCamera() {
        val cam = SimulatedCamera(cfg())
        val detections = cam.frame(
            ballsField = listOf(Vector3d(-1.0, 1.0, 0.035)),   // behind
            t = 0.1,
            robotPose = Pose2d(0.5, 1.0, 0.0),
        )
        assertTrue(detections.isEmpty())
    }

    @Test
    @DisplayName("skips a ball out of the image frame")
    fun skipsOutOfFrame() {
        val cam = SimulatedCamera(cfg())
        // Pitched-down camera — ball far off to the left at close range should exit the image.
        val detections = cam.frame(
            ballsField = listOf(Vector3d(1.0, 5.0, 0.035)),   // 4 m to the side
            t = 0.1,
            robotPose = Pose2d(0.5, 1.0, 0.0),
        )
        assertTrue(detections.isEmpty(), "expected out-of-frame rejection, got: $detections")
    }

    @Test
    @DisplayName("zero-noise detections are reproducible")
    fun zeroNoiseReproducible() {
        val camA = SimulatedCamera(cfg(seed = 1L))
        val camB = SimulatedCamera(cfg(seed = 1L))
        val balls = listOf(Vector3d(2.0, 1.0, 0.035))
        val a = camA.frame(balls, 0.0, Pose2d(0.5, 1.0, 0.0))
        val b = camB.frame(balls, 0.0, Pose2d(0.5, 1.0, 0.0))
        assertEquals(a.size, b.size)
        assertEquals(a[0].u, b[0].u, 0.0)
        assertEquals(a[0].v, b[0].v, 0.0)
    }

    @Test
    @DisplayName("pDrop=1.0 drops every detection")
    fun pDropOneDropsAll() {
        val cam = SimulatedCamera(cfg(pDrop = 1.0))
        val detections = cam.frame(
            ballsField = listOf(Vector3d(2.0, 1.0, 0.035)),
            t = 0.1,
            robotPose = Pose2d(0.5, 1.0, 0.0),
        )
        assertTrue(detections.isEmpty())
    }

    @Test
    @DisplayName("pFalsePositive=1.0 emits one FP per frame in addition to real detections")
    fun pFalsePositiveOneEmitsFp() {
        val cam = SimulatedCamera(cfg(pFalsePositive = 1.0))
        val detections = cam.frame(
            ballsField = listOf(Vector3d(2.0, 1.0, 0.035)),
            t = 0.1,
            robotPose = Pose2d(0.5, 1.0, 0.0),
        )
        assertEquals(2, detections.size)  // 1 real + 1 FP
    }

    @Test
    @DisplayName("Gaussian noise spreads detections around the zero-noise centroid")
    fun noiseSpreadsAroundCentroid() {
        val truth = Vector3d(2.0, 1.0, 0.035)
        val pose = Pose2d(0.5, 1.0, 0.0)
        val camClean = SimulatedCamera(cfg())
        val clean = camClean.frame(listOf(truth), 0.0, pose)[0]

        val camNoisy = SimulatedCamera(cfg(sigmaPxSim = 2.0, seed = 123L))
        val sumU = mutableListOf<Double>()
        val sumV = mutableListOf<Double>()
        repeat(500) {
            val d = camNoisy.frame(listOf(truth), 0.0, pose)[0]
            sumU.add(d.u - clean.u)
            sumV.add(d.v - clean.v)
        }
        val meanU = sumU.average()
        val meanV = sumV.average()
        val stdU  = kotlin.math.sqrt(sumU.map { (it - meanU) * (it - meanU) }.average())
        val stdV  = kotlin.math.sqrt(sumV.map { (it - meanV) * (it - meanV) }.average())
        assertTrue(kotlin.math.abs(meanU) < 0.3, "meanU off-center: $meanU")
        assertTrue(kotlin.math.abs(meanV) < 0.3, "meanV off-center: $meanV")
        assertEquals(2.0, stdU, 0.3, "stdU should match sigmaPxSim")
        assertEquals(2.0, stdV, 0.3, "stdV should match sigmaPxSim")
    }
}
