package sigmacorns.math

import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

class Vector3(
    var x: Double,
    var y: Double,
    var z: Double
) {
    constructor(x: Number, y: Number, z: Number): this(x.toDouble(), y.toDouble(), z.toDouble())

    val xy: Vector2
        get() = Vector2(x,y)
    val yz: Vector2
        get() = Vector2(y,z)
    val xz: Vector2
        get() = Vector2(x,z)

    companion object {
        val ZERO = Vector3(0.0, 0.0, 0.0)

        fun spherical(phi: Number, theta: Number, magnitude: Number = 1.0) {
            val phi = phi.toDouble()
            val theta = theta.toDouble()
            val magnitude = magnitude.toDouble()

            Vector3(
                sin(theta)*cos(phi)*magnitude,
                sin(theta)*sin(phi)*magnitude,
                cos(theta)*magnitude

            )
        }
    }

    operator fun plus(rhs: Vector3): Vector3 {
        return Vector3(
            x + rhs.x,
            y + rhs.y,
            z + rhs.z
        )
    }

    operator fun minus(rhs: Vector3): Vector3 {
        return Vector3(
            x - rhs.x,
            y - rhs.y,
            z - rhs.z
        )
    }

    operator fun times(rhs: Number): Vector3 {
        val s = rhs.toDouble()
        return Vector3(
            x*s,
            y*s,
            z*s
        )
    }

    operator fun div(rhs: Number): Vector3 {
        val s = rhs.toDouble()
        return Vector3(
            x/s,
            y/s,
            z/s
        )
    }

    infix fun dot(rhs: Vector3): Double {
        return x*rhs.x + y*rhs.y + z*rhs.z
    }

    fun magnitude(): Double = sqrt(this dot this)

    fun normalized(): Vector3 = this/magnitude()

    fun phi() = y.sign*acos(x/sqrt(x*x + y*y))

    fun theta() = acos(z/magnitude())
}

operator fun Number.times(rhs: Vector3): Vector3 {
    val s = toDouble()
    return Vector3(
        rhs.x*s,
        rhs.y*s,
        rhs.z*s
    )
}
