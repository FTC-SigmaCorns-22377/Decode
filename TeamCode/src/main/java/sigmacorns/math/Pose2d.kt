package sigmacorns.math

import org.joml.*

data class Pose2d(
    var v: Vector2d,
    var rot: Double
) {
    constructor(x: Double, y: Double, theta: Double): this(Vector2d(x,y), theta)

    operator fun plus(rhs: Pose2d) = Pose2d(v + rhs.v, rot + rhs.rot)
    operator fun plus(rhs: Vector2d) = Pose2d(v + rhs, rot)
}