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
import sigmacorns.vision.tracker.Frames
import sigmacorns.vision.tracker.Intrinsics
import sigmacorns.vision.tracker.Projection
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.math.sqrt

@DisplayName("Projection")
class ProjectionTest {

    private val K = Intrinsics(fx = 800.0, fy = 800.0, cx = 640.0, cy = 360.0)
    // Realistic mild distortion. k2=-0.20 (the OpenCV doc example) makes the
    // distortion polynomial non-monotonic at off-axis pixels and breaks the
    // iterative undistort — not a model bug, just an unphysical test point.
    private val Kdist = Intrinsics(
        fx = 800.0, fy = 800.0, cx = 640.0, cy = 360.0,
        k1 = 0.05, k2 = -0.02, p1 = 5e-4, p2 = -5e-4, k3 = 0.0,
    )

    /** Assert a forward-projected pixel is inside a 1280x720 sensor. */
    private fun assertInSensor(px: DoubleArray, where: Any) {
        assertTrue(px[0] in 0.0..1280.0 && px[1] in 0.0..720.0,
            "pixel $where = (${px[0]}, ${px[1]}) outside sensor — test data is bogus")
    }

    /** Build a T_FC in which the camera sits at (xc, yc, h) looking straight DOWN. */
    private fun downwardCameraTFC(xc: Double, yc: Double, h: Double): Matrix4d {
        // Hand-build R_FC: camera +X = field +X, camera +Y = -Y field, camera +Z = -Z field.
        // (Image-right along field +X; image-down along field -Y; optical axis pointing down.)
        return Matrix4d().apply {
            m00( 1.0); m01( 0.0); m02( 0.0); m03(0.0)   // camera +X -> field +X
            m10( 0.0); m11(-1.0); m12( 0.0); m13(0.0)   // camera +Y -> field -Y
            m20( 0.0); m21( 0.0); m22(-1.0); m23(0.0)   // camera +Z -> field -Z
            m30(xc);   m31(yc);   m32(h);    m33(1.0)
        }
    }

    @Test
    @DisplayName("downward camera: principal-point pixel projects directly below the camera")
    fun principalPointProjectsBelowDownwardCamera() {
        val T_FC = downwardCameraTFC(2.0, 3.0, 1.5)
        val p = Projection.projectToGround(K.cx, K.cy, K, T_FC, h = 0.0, maxRangeM = 100.0)
        assertNotNull(p, "ray should hit the ground")
        assertEquals(2.0, p!!.x, 1e-12)
        assertEquals(3.0, p.y, 1e-12)
    }

    @Test
    @DisplayName("downward camera: off-axis pixel projects to expected ground offset")
    fun offAxisPixelDownwardCamera() {
        val T_FC = downwardCameraTFC(0.0, 0.0, 2.0)
        // Pixel offset 80 px right of cx -> normalized x = 80/800 = 0.1
        // Camera +X = field +X, so the ray hits ground at x = 2 m * 0.1 = 0.2 m.
        // Pixel offset 40 px below cy -> normalized y = 40/800 = 0.05.
        // Camera +Y = field -Y, so the ray hits ground at y = 2 m * (-0.05) = -0.10 m.
        val p = Projection.projectToGround(K.cx + 80.0, K.cy + 40.0, K, T_FC, 0.0, 100.0)
        assertNotNull(p)
        assertEquals( 0.20, p!!.x, 1e-12)
        assertEquals(-0.10, p.y,  1e-12)
    }

    @Test
    @DisplayName("ray pointing away from plane returns null")
    fun raySloppingUpReturnsNull() {
        // Camera looks straight UP (rotate downward camera by 180 around field +X).
        val T_FC = Matrix4d().apply {
            m00( 1.0); m01( 0.0); m02( 0.0); m03(0.0)
            m10( 0.0); m11( 1.0); m12( 0.0); m13(0.0)
            m20( 0.0); m21( 0.0); m22( 1.0); m23(0.0)
            m30( 0.0); m31( 0.0); m32( 1.0); m33(1.0)   // camera at z=1 looking +Z
        }
        // Optical axis along +Z field, so ray will never hit z=0.
        val p = Projection.projectToGround(K.cx, K.cy, K, T_FC, h = 0.0, maxRangeM = 100.0)
        assertNull(p, "ray pointing up cannot intersect ground below")
    }

    @Test
    @DisplayName("range cap rejects very long shots")
    fun rangeCapRejects() {
        val T_FC = downwardCameraTFC(0.0, 0.0, 0.5)
        // Tiny pitch from straight-down — but with downward optics any pixel is in range.
        // Use an extreme pixel to make the slant range > 0.6 m.
        val u = K.cx + 600.0   // normalized x = 0.75
        val v = K.cy
        // s = (0 - 0.5) / dF.z; dF after rotation = (0.75, 0, -1) -> s = 0.5; slant = 0.5 * sqrt(0.75^2+1) = 0.625
        val ok  = Projection.projectToGround(u, v, K, T_FC, 0.0, maxRangeM = 1.0)
        val bad = Projection.projectToGround(u, v, K, T_FC, 0.0, maxRangeM = 0.5)
        assertNotNull(ok)
        assertNull(bad)
    }

    @Test
    @DisplayName("forward-then-back round trip is exact (no distortion) to 1e-9 m")
    fun roundTripNoDistortion() {
        val T_RC = Frames.buildTRC(
            camPosR = Vector3d(0.10, 0.05, 0.30),
            pitchDown = 0.25, yaw = 0.0, roll = 0.0,
        )
        val pose = Pose2d(0.0, 0.0, 0.0)
        val T_FC = Frames.buildTFC(pose, T_RC)
        // Balls in the camera's FOV given the geometry above
        // (camera at (0.10, 0.05, 0.30), pitched 0.25 rad down, looking +X).
        val targets = listOf(
            Vector3d(1.30, 0.05, 0.0),  // near principal-point ground intercept
            Vector3d(1.00, 0.30, 0.0),  // closer, left-of-center
            Vector3d(2.00, -0.20, 0.0), // farther, right-of-center
        )
        for (ball in targets) {
            val px = Projection.forwardProject(ball, K, T_FC)
            assertNotNull(px, "ball $ball should be visible")
            assertInSensor(px!!, ball)
            val back = Projection.projectToGround(px[0], px[1], K, T_FC, h = 0.0, maxRangeM = 50.0)
            assertNotNull(back, "back-projection should hit ground")
            assertEquals(ball.x, back!!.x, 1e-9, "x for ball $ball")
            assertEquals(ball.y, back.y,  1e-9, "y for ball $ball")
        }
    }

    @Test
    @DisplayName("forward-then-back round trip is within 1e-6 m WITH realistic distortion")
    fun roundTripWithDistortion() {
        val T_RC = Frames.buildTRC(
            camPosR = Vector3d(0.10, 0.05, 0.30),
            pitchDown = 0.25, yaw = 0.0, roll = 0.0,
        )
        val pose = Pose2d(0.0, 0.0, 0.0)
        val T_FC = Frames.buildTFC(pose, T_RC)
        val targets = listOf(
            Vector3d(1.30, 0.05, 0.0),
            Vector3d(1.00, 0.30, 0.0),
            Vector3d(2.00, -0.20, 0.0),
            Vector3d(1.50, 0.40, 0.0),
        )
        for (ball in targets) {
            val px = Projection.forwardProject(ball, Kdist, T_FC)
            assertNotNull(px, "ball $ball should be visible")
            assertInSensor(px!!, ball)
            val back = Projection.projectToGround(px[0], px[1], Kdist, T_FC, h = 0.0, maxRangeM = 50.0)
            assertNotNull(back, "back-projection should hit ground")
            assertEquals(ball.x, back!!.x, 1e-6, "x for ball $ball")
            assertEquals(ball.y, back.y,  1e-6, "y for ball $ball")
        }
    }

    @Test
    @DisplayName("undistort(distort(x, y)) round trip is < 1e-7")
    fun distortionRoundTrip() {
        val xn = 0.3
        val yn = -0.2
        val (xd, yd) = Projection.distortNormalized(xn, yn, Kdist)
        val u = Kdist.fx * xd + Kdist.cx
        val v = Kdist.fy * yd + Kdist.cy
        val (xu, yu) = Projection.undistortPixel(u, v, Kdist)
        assertEquals(xn, xu, 1e-7)
        assertEquals(yn, yu, 1e-7)
    }

    @Test
    @DisplayName("covariance major axis aligns with viewing direction (range axis)")
    fun covarianceMajorAxisAlignsWithViewingDirection() {
        // Camera at robot origin pitched down 25 deg, robot at field origin facing +X.
        val T_RC = Frames.buildTRC(Vector3d(0.0, 0.0, 0.5), pitchDown = 0.25, yaw = 0.0, roll = 0.0)
        val pose = Pose2d(0.0, 0.0, 0.0)
        val T_FC = Frames.buildTFC(pose, T_RC)
        // Ball straight ahead on the ground: forward-project it, then call covariance API.
        val ball = Vector3d(2.0, 0.0, 0.0)
        val px = Projection.forwardProject(ball, K, T_FC)
        assertNotNull(px, "ball should be visible")
        val pr = Projection.projectToGroundWithCovariance(
            px!![0], px[1], K, T_FC, h = 0.0, maxRangeM = 50.0, sigmaPx = 1.0,
        )
        assertNotNull(pr)
        val cov = pr!!.cov
        // 2x2 covariance: c00 = var_x, c11 = var_y. For a forward-looking camera,
        // pixel noise stretches mainly along the range (X) direction.
        val varX = cov[0]
        val varY = cov[3]
        assertTrue(varX > varY,
            "expected var_x (range) > var_y (cross-range) for forward-looking camera; got varX=$varX varY=$varY")
        // Symmetry check on off-diagonal.
        assertEquals(cov[1], cov[2], 1e-15, "covariance should be symmetric")
        // For axis-aligned geometry (ball on +X axis, no yaw, no roll), the off-diagonal
        // should be ~zero.
        assertTrue(kotlin.math.abs(cov[1]) < 1e-12,
            "off-diagonal should be ~0 for axis-aligned geometry, got ${cov[1]}")
    }
}
