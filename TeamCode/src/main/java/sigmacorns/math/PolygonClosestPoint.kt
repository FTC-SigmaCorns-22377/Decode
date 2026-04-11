package sigmacorns.math

import org.joml.Vector2d

fun closestPointOnConvexPolygon(polygon: List<Vector2d>, point: Vector2d): Vector2d {
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

fun closestPointOnSegment(a: Vector2d, b: Vector2d, p: Vector2d): Vector2d {
    val abx = b.x - a.x
    val aby = b.y - a.y
    val apx = p.x - a.x
    val apy = p.y - a.y

    val t = ((apx * abx + apy * aby) / (abx * abx + aby * aby)).coerceIn(0.0, 1.0)

    return Vector2d(a.x + t * abx, a.y + t * aby)
}