package sigmacorns.test.vision

import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.Frames
import kotlin.math.PI

@DisplayName("Frames")
class FramesTest {

    private val tol = 1e-12

    @Test
    @DisplayName("buildTFR identity for zero pose")
    fun tfrIdentityForZeroPose() {
        val T = Frames.buildTFR(Pose2d(0.0, 0.0, 0.0))
        for (i in 0..3) for (j in 0..3) {
            val expected = if (i == j) 1.0 else 0.0
            assertEquals(expected, T.get(i, j), tol, "T[$i][$j]")
        }
    }

    @Test
    @DisplayName("buildTFR translation only places robot origin in field")
    fun tfrPureTranslation() {
        val T = Frames.buildTFR(Pose2d(1.5, -0.4, 0.0))
        assertEquals(1.5, T.m30(), tol)
        assertEquals(-0.4, T.m31(), tol)
        assertEquals(0.0, T.m32(), tol)
        // Rotation block is identity
        assertEquals(1.0, T.m00(), tol); assertEquals(1.0, T.m11(), tol); assertEquals(1.0, T.m22(), tol)
    }

    @Test
    @DisplayName("buildTFR with 90-deg heading rotates +X_robot to +Y_field")
    fun tfrRotation90() {
        val T = Frames.buildTFR(Pose2d(0.0, 0.0, PI / 2.0))
        // Apply T to a robot-frame point along +X (forward) — should land along +Y in field.
        val r = org.joml.Vector3d(1.0, 0.0, 0.0)
        val f = org.joml.Vector3d()
        T.transformPosition(r, f)
        assertEquals(0.0, f.x, 1e-12)
        assertEquals(1.0, f.y, 1e-12)
        assertEquals(0.0, f.z, 1e-12)
    }

    @Test
    @DisplayName("buildTRC identity extrinsics map camera +Z to robot +X (forward)")
    fun trcZeroExtrinsics() {
        val T = Frames.buildTRC(Vector3d(0.0, 0.0, 0.0), 0.0, 0.0, 0.0)
        // Apply rotation only: camera +Z (optical axis) should land on robot +X.
        val cZ = Vector3d(0.0, 0.0, 1.0)
        val r = Vector3d()
        T.transformDirection(cZ, r)
        assertEquals(1.0, r.x, 1e-12)
        assertEquals(0.0, r.y, 1e-12)
        assertEquals(0.0, r.z, 1e-12)
    }

    @Test
    @DisplayName("buildTRC identity extrinsics map camera +X (image right) to robot -Y (right)")
    fun trcZeroExtrinsicsImageRight() {
        val T = Frames.buildTRC(Vector3d(0.0, 0.0, 0.0), 0.0, 0.0, 0.0)
        val cX = Vector3d(1.0, 0.0, 0.0)
        val r = Vector3d()
        T.transformDirection(cX, r)
        assertEquals( 0.0, r.x, 1e-12)
        assertEquals(-1.0, r.y, 1e-12)
        assertEquals( 0.0, r.z, 1e-12)
    }

    @Test
    @DisplayName("buildTRC identity extrinsics map camera +Y (image down) to robot -Z (down)")
    fun trcZeroExtrinsicsImageDown() {
        val T = Frames.buildTRC(Vector3d(0.0, 0.0, 0.0), 0.0, 0.0, 0.0)
        val cY = Vector3d(0.0, 1.0, 0.0)
        val r = Vector3d()
        T.transformDirection(cY, r)
        assertEquals( 0.0, r.x, 1e-12)
        assertEquals( 0.0, r.y, 1e-12)
        assertEquals(-1.0, r.z, 1e-12)
    }

    @Test
    @DisplayName("buildTRC translates camera origin into robot frame")
    fun trcTranslation() {
        val T = Frames.buildTRC(Vector3d(0.20, -0.05, 0.30), 0.0, 0.0, 0.0)
        // The 4th column in JOML is (m30, m31, m32) — the translation.
        assertEquals( 0.20, T.m30(), tol)
        assertEquals(-0.05, T.m31(), tol)
        assertEquals( 0.30, T.m32(), tol)
    }

    @Test
    @DisplayName("buildTRC with positive pitchDown tilts optical axis toward floor")
    fun trcPitchDownTiltsOpticalAxisDown() {
        val T = Frames.buildTRC(Vector3d(0.0, 0.0, 0.0), pitchDown = 0.30, yaw = 0.0, roll = 0.0)
        val cZ = Vector3d(0.0, 0.0, 1.0)
        val r = Vector3d()
        T.transformDirection(cZ, r)
        // Forward component still positive, vertical component now negative (down).
        assert(r.x > 0.0) { "optical axis should still point mostly forward, got x=${r.x}" }
        assert(r.z < 0.0) { "pitchDown > 0 must give optical axis with negative robot Z, got z=${r.z}" }
        // Sanity: |z| ~ sin(0.30); |x| ~ cos(0.30)
        assertEquals(kotlin.math.cos(0.30), r.x, 1e-9)
        assertEquals(-kotlin.math.sin(0.30), r.z, 1e-9)
    }

    @Test
    @DisplayName("buildTRC with positive yaw swings optical axis toward robot left (+Y)")
    fun trcYawSwingsLeft() {
        val T = Frames.buildTRC(Vector3d(0.0, 0.0, 0.0), pitchDown = 0.0, yaw = 0.20, roll = 0.0)
        val cZ = Vector3d(0.0, 0.0, 1.0)
        val r = Vector3d()
        T.transformDirection(cZ, r)
        assertEquals(kotlin.math.cos(0.20), r.x, 1e-9)
        assertEquals(kotlin.math.sin(0.20), r.y, 1e-9)
        assertEquals(0.0, r.z, 1e-9)
    }

    @Test
    @DisplayName("buildTFC composes T_FR * T_RC")
    fun tfcComposition() {
        val T_RC = Frames.buildTRC(Vector3d(0.10, 0.0, 0.20), 0.0, 0.0, 0.0)
        val pose = Pose2d(2.0, 1.0, PI / 2.0)
        val T_FC = Frames.buildTFC(pose, T_RC)
        // The camera origin in robot frame is (0.10, 0, 0.20). After heading=PI/2,
        // robot +X becomes field +Y, so robot (0.10, 0, 0.20) -> field offset (0, +0.10, 0.20).
        // Plus the robot origin at field (2.0, 1.0, 0.0):
        // expected camera origin in field = (2.0, 1.10, 0.20).
        assertEquals(2.0,  T_FC.m30(), 1e-12)
        assertEquals(1.10, T_FC.m31(), 1e-12)
        assertEquals(0.20, T_FC.m32(), 1e-12)
    }
}
