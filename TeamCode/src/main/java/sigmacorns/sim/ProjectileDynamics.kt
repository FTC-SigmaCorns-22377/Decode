package sigmacorns.sim

import org.joml.Vector3d

/**
 * Explicit ODE integration for projectile motion under gravity.
 */
class ProjectileDynamics(val gravityMagnitude: Double) {

    private fun dx(state: DoubleArray): DoubleArray {
        val vx = state[3]
        val vy = state[4]
        val vz = state[5]

        return doubleArrayOf(
            vx,
            vy,
            vz,
            0.0,
            0.0,
            -gravityMagnitude,
        )
    }

    fun integrate(tf: Double, dt: Double, x: ProjectileState): ProjectileState {
        if (!x.active) {
            return x
        }

        val x0 = x.toDoubleArray()
        val xf = rk4Integrate(tf, dt, x0) { state -> dx(state) }

        val position = Vector3d(xf[0], xf[1], xf[2])
        val velocity = Vector3d(xf[3], xf[4], xf[5])
        val stillAirborne = position.z > 0.0

        return if (stillAirborne) {
            ProjectileState(position, velocity, true)
        } else {
            position.z = 0.0
            velocity.z = 0.0
            ProjectileState(position, velocity, false)
        }
    }
}
