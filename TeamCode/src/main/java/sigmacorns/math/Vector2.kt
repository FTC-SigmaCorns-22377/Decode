package sigmacorns.math

import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

class Vector2(
    var x: Double,
    var y: Double,
) {
    constructor(x: Number, y: Number): this(x.toDouble(), y.toDouble() )

    companion object {
        val ZERO = Vector2(0.0, 0.0)
    }

    operator fun plus(rhs: Vector2): Vector2 {
        return Vector2(
            x + rhs.x,
            y + rhs.y,
        )
    }

    operator fun minus(rhs: Vector2): Vector2 {
        return Vector2(
            x - rhs.x,
            y - rhs.y,
        )
    }

    operator fun times(rhs: Number): Vector2 {
        val s = rhs.toDouble()
        return Vector2(
            x*s,
            y*s
        )
    }

    operator fun div(rhs: Number): Vector2 {
        val s = rhs.toDouble()
        return Vector2(
            x/s,
            y/s
        )
    }

    infix fun dot(rhs: Vector2): Double {
        return x*rhs.x + y*rhs.y
    }

    fun magnitude(): Double = sqrt(this dot this)

    fun normalized(): Vector2 = this/magnitude()

    fun withZ(z: Number) = Vector3(x,y,z)
}

operator fun Number.times(rhs: Vector2): Vector2 {
    val s = toDouble()
    return Vector2(
        rhs.x*s,
        rhs.y*s,
    )
}
