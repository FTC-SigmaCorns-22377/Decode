package sigmacorns.math

import org.joml.Vector2d
import kotlin.math.cos
import kotlin.math.sin

class PolygonOverlapDetection {

    companion object {
        // Robot corners from center, heading, and inflated dimensions
        fun robotCorners(
            cx: Double,
            cy: Double,
            heading: Double,
            width: Double,
            depth: Double,
            margin: Double
        ): List<Vector2d> {
            val hw = (width) / 2 - margin
            val hd = (depth) / 2 - margin
            val cos = cos(heading)
            val sin = sin(heading)
            // local offsets → rotated into field frame
            return listOf(
                Vector2d(cx + hw * cos - hd * sin, cy + hw * sin + hd * cos),
                Vector2d(cx - hw * cos - hd * sin, cy - hw * sin + hd * cos),
                Vector2d(cx - hw * cos + hd * sin, cy - hw * sin - hd * cos),
                Vector2d(cx + hw * cos + hd * sin, cy + hw * sin - hd * cos),
            )
        }

        // SAT overlap test for two convex polygons
        fun doPolygonsOverlap(a: List<Vector2d>, b: List<Vector2d>): Boolean {
            for (poly in listOf(a, b)) {
                for (i in poly.indices) {
                    val edge = Vector2d(poly[(i + 1) % poly.size]).sub(poly[i])
                    val axis = Vector2d(-edge.y, edge.x) // normal

                    val aProj = a.map { axis.dot(it) }
                    val bProj = b.map { axis.dot(it) }

                    if (aProj.max() < bProj.min() || bProj.max() < aProj.min())
                        return false
                }
            }
            return true
        }
    }
}