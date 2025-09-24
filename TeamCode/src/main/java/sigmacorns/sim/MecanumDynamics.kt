package sigmacorns.sim

import org.joml.*
import sigmacorns.math.Pose2d

/**
 * Model of the dynamics of a Mecanum drivetrain
 *
 * @param r wheel radius (m)
 * @param l manhattan distance to wheel (m)
 * @param motorTopSpeed top speed of the motor (rad/s)
 * @param motorStallTorque stall torque of the motor (Nm)
 * @param weight weight of the robot (kg)
 * @param rotInertia rotational inertia of the robot about the wheelbase center (kg m^2)
 */
class MecanumDynamics(
    val r: Double,
    val l: Double,
    val motorTopSpeed: Double,
    val motorStallTorque: Double,
    val weight: Double,
    val rotInertia: Double,
) {
    fun mecanumForwardKinematics(wheelVels: Vector4d): Pose2d {
        val A = Matrix4d(
            1.0, 1.0, 1.0, 1.0,
            -1.0, 1.0, -1.0, 1.0,
            -1.0 / l, -1.0 / l, 1.0 / l, 1.0 / l,
            0.0, 0.0, 0.0, 0.0
        ).scale(r/4.0).transpose()

        val v = A * wheelVels


        return Pose2d(v.x,v.y,v.z)
    }

    fun mecanumInverseKinematics(robotVel: Pose2d): Vector4d {
        val Ainv = Matrix4d(
            1.0, -1.0, -l, 0.0,
            1.0, 1.0, -l, 0.0,
            1.0, -1.0, l, 0.0,
            1.0, 1.0, l, 0.0,
        )
            .transpose()
            .scale(1.0 / r)

        return Ainv*Vector4d(robotVel.v.x,robotVel.v.y,robotVel.rot, 0.0)
    }

    class MecanumState(
        val vel: Pose2d,
        val pos: Pose2d
    )

    private fun dx(u: DoubleArray, x: DoubleArray): DoubleArray {
        // robot relative velocity
        val vel = Pose2d(x[0],x[1],x[2]).also {
            it.v = Matrix2d().rotate(-it.rot)*it.v
        }

        val wheelVels = mecanumInverseKinematics(vel)


        // magnitude of the force produced by each wheel
        val forces = (Vector4d(motorTopSpeed) - wheelVels)*motorStallTorque/r

        // robot relative forces/torques
        val robotTwist = mecanumForwardKinematics(forces)

        // field relative acceleration
        val acc = robotTwist.copy().also {
            it.v = Matrix2d().rotate(it.rot)*it.v

            it.v /= weight
            it.rot /= rotInertia
        }

        return doubleArrayOf(
            acc.v.x,
            acc.v.y,
            acc.rot,
            x[0], // dx = vx
            x[1], // dy = vy
            x[2] // dtheta = omega
        )
    }

    fun integrate(tf: Double, dt: Double, u: DoubleArray, x: MecanumState): MecanumState {
        val x0 = doubleArrayOf(x.vel.v.x, x.vel.v.y, x.vel.rot, x.pos.v.x, x.pos.v.y, x.pos.rot)

        val xf = rk4Integrate(tf,dt,x0) { this.dx(u,it) }

        println("dx ${this.dx(u,x0).contentToString()}")
        println("xf ${xf.contentToString()}")

        return MecanumState(
            Pose2d(xf[0],xf[1],xf[2]),
            Pose2d(xf[3],xf[4],xf[5]),
        )
    }
}
