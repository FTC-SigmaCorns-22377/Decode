package sigmacorns.math

import org.joml.*

data class Pose2d(
    var v: Vector2d,
    var rot: Double
) {
    constructor(x: Double, y: Double, theta: Double): this(Vector2d(x,y), theta)
    constructor(): this(Vector2d(), 0.0)

    operator fun plus(rhs: Pose2d) = Pose2d(v + rhs.v, rot + rhs.rot)
    operator fun plus(rhs: Vector2d) = Pose2d(v + rhs, rot)

    operator fun minus(rhs: Pose2d) = Pose2d(v - rhs.v, rot - rhs.rot)
    operator fun minus(rhs: Vector2d) = Pose2d(v - rhs, rot)

    operator fun times(rhs: Number) = Pose2d(v * rhs.toDouble(), rot * rhs.toDouble())
    operator fun div(rhs: Number) = Pose2d(v / rhs.toDouble(), rot / rhs.toDouble())

    fun componentMul(rhs: Pose2d)  = Pose2d(
        v.x * rhs.v.x,
        v.y * rhs.v.y,
        rot * rhs.rot
    )
}