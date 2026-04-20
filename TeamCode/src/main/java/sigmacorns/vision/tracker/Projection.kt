package sigmacorns.vision.tracker

import org.joml.Matrix4d
import org.joml.Vector2d
import org.joml.Vector3d
import kotlin.math.abs

/**
 * Pinhole + radial/tangential distortion intrinsics. Layout matches OpenCV's
 * (k1, k2, p1, p2, k3) distortion vector. Pure-Kotlin port — no OpenCV dep.
 */
data class Intrinsics(
    val fx: Double,
    val fy: Double,
    val cx: Double,
    val cy: Double,
    val k1: Double = 0.0,
    val k2: Double = 0.0,
    val p1: Double = 0.0,
    val p2: Double = 0.0,
    val k3: Double = 0.0,
)

/** Result of projection-with-covariance: 2D field point + 2x2 covariance (row-major). */
data class ProjectionResult(
    val point: Vector2d,
    val cov: DoubleArray,
) {
    init {
        require(cov.size == 4) { "cov must be a 2x2 matrix in row-major order (size=4)" }
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is ProjectionResult) return false
        return point == other.point && cov.contentEquals(other.cov)
    }

    override fun hashCode(): Int = 31 * point.hashCode() + cov.contentHashCode()
}

/**
 * Pixel <-> field-plane projection helpers. Filters in field coordinates only —
 * pixel is just an entry/exit point.
 */
object Projection {

    /**
     * OpenCV-style iterative undistortion (20 Newton iterations). Returns
     * normalized image coordinates (xn, yn) such that the radial+tangential
     * distortion polynomial maps them back to (u, v). 20 iterations was chosen
     * empirically — 5 (the OpenCV default for speed) is not enough for k1≈0.1
     * distortion at off-axis pixels to round-trip below 1e-6 m at typical
     * field-projection ranges.
     */
    fun undistortPixel(u: Double, v: Double, K: Intrinsics): DoubleArray {
        val x = (u - K.cx) / K.fx
        val y = (v - K.cy) / K.fy
        var xu = x
        var yu = y
        repeat(20) {
            val r2 = xu * xu + yu * yu
            val radial = 1.0 / (1.0 + K.k1 * r2 + K.k2 * r2 * r2 + K.k3 * r2 * r2 * r2)
            val dx = 2.0 * K.p1 * xu * yu + K.p2 * (r2 + 2.0 * xu * xu)
            val dy = K.p1 * (r2 + 2.0 * yu * yu) + 2.0 * K.p2 * xu * yu
            xu = (x - dx) * radial
            yu = (y - dy) * radial
        }
        return doubleArrayOf(xu, yu)
    }

    /**
     * Forward distortion: maps undistorted normalized (xn, yn) to distorted
     * normalized (xd, yd). Used by SimulatedCamera and as the inverse of
     * undistortPixel for round-trip checks.
     */
    fun distortNormalized(xn: Double, yn: Double, K: Intrinsics): DoubleArray {
        val r2 = xn * xn + yn * yn
        val radial = 1.0 + K.k1 * r2 + K.k2 * r2 * r2 + K.k3 * r2 * r2 * r2
        val xd = xn * radial + 2.0 * K.p1 * xn * yn + K.p2 * (r2 + 2.0 * xn * xn)
        val yd = yn * radial + K.p1 * (r2 + 2.0 * yn * yn) + 2.0 * K.p2 * xn * yn
        return doubleArrayOf(xd, yd)
    }

    /**
     * Forward projection: a 3D field point through the camera to a pixel.
     * Returns null if the point is at or behind the image plane in camera frame.
     */
    fun forwardProject(pF: Vector3d, K: Intrinsics, T_FC: Matrix4d): DoubleArray? {
        val T_CF = Matrix4d(T_FC).invert()
        val pC = Vector3d()
        T_CF.transformPosition(pF, pC)
        if (pC.z <= 0.0) return null
        val xn = pC.x / pC.z
        val yn = pC.y / pC.z
        val (xd, yd) = distortNormalized(xn, yn, K)
        return doubleArrayOf(K.fx * xd + K.cx, K.fy * yd + K.cy)
    }

    /**
     * Project a pixel onto a horizontal plane at field-frame height h.
     *
     * Steps:
     *   1. Undistort to (xn, yn).
     *   2. Build camera-frame ray d_C = (xn, yn, 1).
     *   3. Rotate to field frame: d_F = R_FC · d_C.
     *   4. Intersect with z=h: s = (h - o_F.z) / d_F.z.
     *   5. p_F = o_F + s · d_F.
     *
     * Returns null if s <= 0 (ray points away from the plane), the ray is
     * parallel to the plane, or the slant range exceeds maxRangeM.
     */
    fun projectToGround(
        u: Double,
        v: Double,
        K: Intrinsics,
        T_FC: Matrix4d,
        h: Double,
        maxRangeM: Double,
    ): Vector2d? {
        val (xn, yn) = undistortPixel(u, v, K)
        val dC = Vector3d(xn, yn, 1.0)
        val dF = Vector3d()
        T_FC.transformDirection(dC, dF)
        if (abs(dF.z) < 1e-12) return null
        val oF = Vector3d(T_FC.m30(), T_FC.m31(), T_FC.m32())
        val s = (h - oF.z) / dF.z
        if (s <= 0.0) return null
        val slantRange = s * dF.length()
        if (slantRange > maxRangeM) return null
        return Vector2d(oF.x + s * dF.x, oF.y + s * dF.y)
    }

    /**
     * Project a pixel + propagate per-pixel covariance via central finite
     * differences (delta = 0.5 px per axis). Returns null if any of the five
     * sample points fail to project.
     *
     * Covariance is row-major 2x2: [c00, c01, c10, c11] with c01 == c10.
     */
    fun projectToGroundWithCovariance(
        u: Double,
        v: Double,
        K: Intrinsics,
        T_FC: Matrix4d,
        h: Double,
        maxRangeM: Double,
        sigmaPx: Double,
    ): ProjectionResult? {
        val center = projectToGround(u, v, K, T_FC, h, maxRangeM) ?: return null
        val delta = 0.5
        val pPlusU  = projectToGround(u + delta, v, K, T_FC, h, maxRangeM) ?: return null
        val pMinusU = projectToGround(u - delta, v, K, T_FC, h, maxRangeM) ?: return null
        val pPlusV  = projectToGround(u, v + delta, K, T_FC, h, maxRangeM) ?: return null
        val pMinusV = projectToGround(u, v - delta, K, T_FC, h, maxRangeM) ?: return null

        val dpduX = (pPlusU.x  - pMinusU.x) / (2.0 * delta)
        val dpduY = (pPlusU.y  - pMinusU.y) / (2.0 * delta)
        val dpdvX = (pPlusV.x  - pMinusV.x) / (2.0 * delta)
        val dpdvY = (pPlusV.y  - pMinusV.y) / (2.0 * delta)

        val s2 = sigmaPx * sigmaPx
        val c00 = s2 * (dpduX * dpduX + dpdvX * dpdvX)
        val c01 = s2 * (dpduX * dpduY + dpdvX * dpdvY)
        val c11 = s2 * (dpduY * dpduY + dpdvY * dpdvY)
        return ProjectionResult(center, doubleArrayOf(c00, c01, c01, c11))
    }
}
