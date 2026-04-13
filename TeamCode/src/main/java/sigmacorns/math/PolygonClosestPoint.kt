package sigmacorns.math

import org.joml.Vector2d

fun closestPointOnConvexPolygon(polygon: List<Vector2d>, point: Vector2d): Vector2d {
    // If the point is already inside the convex polygon, return it directly.
    if (isInsideConvexPolygon(polygon, point)) return Vector2d(point)

    var closestPoint = polygon[0]
    var minDistSq = Double.MAX_VALUE

    for (i in polygon.indices) {
        val a = polygon[i]
        val b = polygon[(i + 1) % polygon.size]

        val candidate = closestPointOnSegment(a, b, point)
        val distSq = (candidate.x - point.x).let { dx ->
            (candidate.y - point.y).let { dy -> dx * dx + dy * dy }
        }

        if (distSq < minDistSq) {
            minDistSq = distSq
            closestPoint = candidate
        }
    }

    return closestPoint
}

/** Check if [point] lies inside a convex [polygon] using cross-product sign consistency. */
fun isInsideConvexPolygon(polygon: List<Vector2d>, point: Vector2d): Boolean {
    if (polygon.size < 3) return false
    var sign = 0
    for (i in polygon.indices) {
        val a = polygon[i]
        val b = polygon[(i + 1) % polygon.size]
        val cross = (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x)
        if (cross != 0.0) {
            val s = if (cross > 0) 1 else -1
            if (sign == 0) sign = s
            else if (s != sign) return false
        }
    }
    return true
}

fun closestPointOnSegment(a: Vector2d, b: Vector2d, p: Vector2d): Vector2d {
    val abx = b.x - a.x
    val aby = b.y - a.y
    val apx = p.x - a.x
    val apy = p.y - a.y

    val t = ((apx * abx + apy * aby) / (abx * abx + aby * aby)).coerceIn(0.0, 1.0)

    return Vector2d(a.x + t * abx, a.y + t * aby)
}