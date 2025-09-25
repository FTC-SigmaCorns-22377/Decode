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
    private val forward = Matrix4d(
        1.0, 1.0, 1.0, 1.0,
        -1.0, 1.0, -1.0, 1.0,
        -1.0 / l, -1.0 / l, 1.0 / l, 1.0 / l,
        0.0, 0.0, 0.0, 0.0
    ).scale(r/4.0).transpose()

    private val inverse = Matrix4d(
        1.0, -1.0, -l, 0.0,
        1.0, 1.0, -l, 0.0,
        1.0, -1.0, l, 0.0,
        1.0, 1.0, l, 0.0,
    )
        .transpose()
        .scale(1.0 / r)

    /**
     * Mecanum Forward Kinematics
     *
     * @param wheelVels vector of wheel velocities in rad/s, ordered as `[FL,BL,BR,FR]`
     *
     * @return the robot-relative velocity resulting from the provided `wheelVels` as `Pose2d(m/s, m/s, rad/s)`
     */
    fun mecanumForwardKinematics(wheelVels: Vector4d): Pose2d {
        val v = forward * wheelVels

        return Pose2d(v.x,v.y,v.z)
    }

    /**
     * Mecanum Inverse Kinematics
     *
     * @param robotVel the desired robot-relative velocity
     *
     * @return a vector of wheel velocities in rad/s ordered `[FL,BL,BR,FR]` that will result in the desired `robotVel`.
     */
    fun mecanumInverseKinematics(robotVel: Pose2d): Vector4d {
        return inverse*Vector4d(robotVel.v.x,robotVel.v.y,robotVel.rot, 0.0)
    }

    /**
     * The state of a mecanum drivetrain
     *
     * @param vel the velocity of the drivetrain. units: `Pose2d(m/s, m/s, rad/s)`
     * @param pos the position of the drivetrain. units: `Pose2d(m, m, rad)`
     */
    class MecanumState(
        val vel: Pose2d,
        val pos: Pose2d
    )

    /**
     * Gives the derivative of the state of the drivetrain
     *
     * @param u the array of motor powers ∈ [-1,1] ordered `[FL,BL,BR,FR]`
     * @param x a serialized version of [MecanumState], with the ordering `[vel.v.x, vel.v.y, vel.rot, pos.v.x, pos.v.y, pos.rot]`.
     * Units are (m/s, m/s, rad/s, m, m, rad)
     *
     * @return the derivative of [x] (still a serialized [MecanumState] in the same order).
     * Units are (m/s^2, m/s^2, rad/s^2, m/s, m/s, rad/s)
     */
    private fun dx(u: DoubleArray, x: DoubleArray): DoubleArray {
        // robot relative velocity
        val vel = Pose2d(x[0],x[1],x[2]).also {
            it.v = Matrix2d().rotate(-it.rot)*it.v
        }

        val wheelVels = mecanumInverseKinematics(vel)

        val motorPowers = Vector4d(u[0],u[1],u[2],u[3])

        // magnitude of the force produced by each wheel
        val forces = (Vector4d(motorTopSpeed).mul(motorPowers) - wheelVels)*motorStallTorque/r

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

    /**
     * Integrates [dx] to simulate the movement of a mecanum drivetrain
     *
     * @param tf how long to integrate (s)
     * @param dt how long each timestep should be (s). Lower = more accurate, but also increased computational cost
     * @param u the array of motor powers ∈ [-1,1] ordered (FL,BL,BR,FR)
     * @param x the state of the drivetrain at the start of integration
     *
     * @return the integrated [MecanumState]
     */
    fun integrate(tf: Double, dt: Double, u: DoubleArray, x: MecanumState): MecanumState {
        val x0 = doubleArrayOf(x.vel.v.x, x.vel.v.y, x.vel.rot, x.pos.v.x, x.pos.v.y, x.pos.rot)

        val xf = rk4Integrate(tf,dt,x0) { this.dx(u,it) }

        return MecanumState(
            Pose2d(xf[0],xf[1],xf[2]),
            Pose2d(xf[3],xf[4],xf[5]),
        )
    }
}
