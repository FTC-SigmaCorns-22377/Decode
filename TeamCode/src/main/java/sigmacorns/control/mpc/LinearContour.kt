package sigmacorns.control.mpc

import org.joml.Vector2d
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Linear contour parameters for a single knot point.
 * Each knot defines a target point on the path and the path direction.
 */
data class LinearContour(
    /** Target position on the path */
    val lineP: Vector2d,
    /** Path direction (unit vector) */
    val lineD: Vector2d,
    /** Target heading angle (radians) */
    val targetTheta: Double,
    /** Target angular velocity (rad/s) */
    val targetOmega: Double,
    /** Target x velocity (m/s) for velocity tracking */
    val targetVx: Double = 0.0,
    /** Target y velocity (m/s) for velocity tracking */
    val targetVy: Double = 0.0,
) {
    fun toArray(): DoubleArray = doubleArrayOf(
        lineP.x, lineP.y,
        lineD.x, lineD.y,
        targetTheta, targetOmega,
        targetVx, targetVy
    )

    companion object {
        const val SIZE = 8

        fun fromSample(sample: TrajoptSample, nextSample: TrajoptSample?): LinearContour {
            // Compute path direction from velocity or position difference
            val dir = if (sample.vx != 0.0 || sample.vy != 0.0) {
                val speed = hypot(sample.vx, sample.vy)
                if (speed > 1e-6) {
                    Vector2d(sample.vx / speed, sample.vy / speed)
                } else {
                    Vector2d(1.0, 0.0)
                }
            } else if (nextSample != null) {
                val dx = nextSample.x - sample.x
                val dy = nextSample.y - sample.y
                val d = hypot(dx, dy)
                if (d > 1e-6) {
                    Vector2d(dx / d, dy / d)
                } else {
                    Vector2d(1.0, 0.0)
                }
            } else {
                Vector2d(cos(sample.heading), sin(sample.heading))
            }

            return LinearContour(
                lineP = Vector2d(sample.x, sample.y),
                lineD = dir,
                targetTheta = sample.heading,
                targetOmega = sample.omega,
                targetVx = sample.vx,
                targetVy = sample.vy,
            )
        }
    }
}

enum class ContourSelectionMode {
    POSITION,
    TIME,
}
