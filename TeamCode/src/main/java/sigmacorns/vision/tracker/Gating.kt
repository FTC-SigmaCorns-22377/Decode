package sigmacorns.vision.tracker

import org.joml.Vector2d
import sigmacorns.math.Pose2d
import kotlin.math.cos
import kotlin.math.sin

/**
 * Gating filters applied by the tracker in this order:
 *   1. insideIntakeMask(u, v, imageHeight, yMinFrac)
 *      — image space, reject detections in the bottom portion of the frame
 *        (our own intake wheels and blobs held just inside the intake).
 *      Applied BEFORE projection, on raw pixels.
 *   2. insideField(p, fieldW, fieldH, margin)
 *      — field-rect bounds with an inward margin.
 *   3. outsideRamp(p, rampPolygon, expandM)
 *      — point-in-polygon test against the ramp outline, expanded on the
 *        camera-facing side by expandM (simple isotropic buffer here, since
 *        the ramp polygon itself encodes the shape's orientation).
 *   4. outsideRobotFootprint(p, robotPose, footprintR)
 *      — transform the chassis polygon into field frame by the robot pose,
 *        reject interior detections. Guards against the camera catching our
 *        own frame during aggressive motion.
 */
object Gating {

    /** Returns true if the pixel row is inside the intake mask region. */
    fun insideIntakeMask(v: Double, imageHeight: Int, yMinFrac: Double): Boolean {
        val cutoff = imageHeight * yMinFrac
        return v >= cutoff
    }

    /**
     * Field rectangle bounds with an inward margin, centered at the origin.
     *
     * The FTC DECODE field uses origin-centered coordinates (matches
     * `FieldLandmarks.fieldHalfExtend` and `sigmacorns.math.Pose2d`), so
     * valid `p` satisfies `|p.x| <= fieldWidthM/2 - margin` and likewise for y.
     */
    fun insideField(p: Vector2d, fieldWidthM: Double, fieldHeightM: Double, marginM: Double): Boolean {
        val halfW = fieldWidthM / 2.0 - marginM
        val halfH = fieldHeightM / 2.0 - marginM
        return p.x >= -halfW && p.x <= halfW &&
               p.y >= -halfH && p.y <= halfH
    }

    /**
     * Returns true if `p` lies inside the ramp polygon, expanded isotropically
     * by `expandM`. Empty polygons always return false (no ramp yet configured).
     * The polygon is closed implicitly (last vertex -> first).
     */
    fun insideRamp(p: Vector2d, rampPolygon: List<Vector2d>, expandM: Double): Boolean {
        if (rampPolygon.size < 3) return false
        // Straight point-in-polygon first; if inside, it's definitely inside.
        if (pointInPolygon(p, rampPolygon)) return true
        if (expandM <= 0.0) return false
        // Expansion: if distance from p to the nearest polygon edge is below
        // expandM, treat as inside. Isotropic buffer — simple and conservative.
        return distToPolygonBoundary(p, rampPolygon) < expandM
    }

    /**
     * Transform the chassis footprint into field frame via `robotPose` and
     * return true if `p` is inside. If the polygon has fewer than 3 vertices,
     * returns false (no footprint gating).
     */
    fun insideRobotFootprint(
        p: Vector2d,
        robotPose: Pose2d,
        footprintR: List<Vector2d>,
    ): Boolean {
        if (footprintR.size < 3) return false
        val transformed = transformFootprint(footprintR, robotPose)
        return pointInPolygon(p, transformed)
    }

    /** Apply image-space + field-coord gates in order. Returns the filtered detection or null. */
    fun filterDetection(
        pixel: Vector2d,
        imageHeight: Int,
        intakeMaskYMinFrac: Double,
        fieldPoint: Vector2d?,
        fieldWidthM: Double,
        fieldHeightM: Double,
        fieldMarginM: Double,
        rampPolygon: List<Vector2d>,
        rampExpandM: Double,
        robotPose: Pose2d,
        footprintR: List<Vector2d>,
    ): Vector2d? {
        if (insideIntakeMask(pixel.y, imageHeight, intakeMaskYMinFrac)) return null
        if (fieldPoint == null) return null
        if (!insideField(fieldPoint, fieldWidthM, fieldHeightM, fieldMarginM)) return null
        if (insideRamp(fieldPoint, rampPolygon, rampExpandM)) return null
        if (insideRobotFootprint(fieldPoint, robotPose, footprintR)) return null
        return fieldPoint
    }

    internal fun transformFootprint(footprintR: List<Vector2d>, pose: Pose2d): List<Vector2d> {
        val cs = cos(pose.rot)
        val sn = sin(pose.rot)
        return footprintR.map { v ->
            Vector2d(
                pose.v.x + cs * v.x - sn * v.y,
                pose.v.y + sn * v.x + cs * v.y,
            )
        }
    }

    /** Standard crossing-number point-in-polygon. Edges include their lower endpoint. */
    internal fun pointInPolygon(p: Vector2d, poly: List<Vector2d>): Boolean {
        var inside = false
        val n = poly.size
        var j = n - 1
        for (i in 0 until n) {
            val pi = poly[i]
            val pj = poly[j]
            if ((pi.y > p.y) != (pj.y > p.y)) {
                val xCross = (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x
                if (p.x < xCross) inside = !inside
            }
            j = i
        }
        return inside
    }

    /** Minimum distance from `p` to any closed edge of `poly`. */
    internal fun distToPolygonBoundary(p: Vector2d, poly: List<Vector2d>): Double {
        var best = Double.POSITIVE_INFINITY
        val n = poly.size
        for (i in 0 until n) {
            val a = poly[i]
            val b = poly[(i + 1) % n]
            val d = pointToSegmentDist(p, a, b)
            if (d < best) best = d
        }
        return best
    }

    private fun pointToSegmentDist(p: Vector2d, a: Vector2d, b: Vector2d): Double {
        val abx = b.x - a.x
        val aby = b.y - a.y
        val apx = p.x - a.x
        val apy = p.y - a.y
        val ab2 = abx * abx + aby * aby
        if (ab2 < 1e-18) {
            val dx = p.x - a.x
            val dy = p.y - a.y
            return kotlin.math.sqrt(dx * dx + dy * dy)
        }
        val tRaw = (apx * abx + apy * aby) / ab2
        val t = tRaw.coerceIn(0.0, 1.0)
        val cx = a.x + t * abx
        val cy = a.y + t * aby
        val dx = p.x - cx
        val dy = p.y - cy
        return kotlin.math.sqrt(dx * dx + dy * dy)
    }
}
