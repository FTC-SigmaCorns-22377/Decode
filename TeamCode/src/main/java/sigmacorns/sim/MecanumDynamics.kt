package sigmacorns.sim

import org.joml.*
import sigmacorns.math.Pose2d
import kotlin.math.sqrt

/**
 * Model of the dynamics of a Mecanum drivetrain
 *
 * @param p the parameters of the model
 * @see MecanumParameters
 */
class MecanumDynamics(val p: MecanumParameters) {
    val l = (p.lx+p.ly)
    val l2 = Vector2d(p.lx,p.ly).length()

    private val forwardVel = Matrix4d(
        1.0, 1.0, 1.0, 1.0,
        -1.0, 1.0, -1.0, 1.0,
        -1.0 / l, -1.0 / l, 1.0 / l, 1.0 / l,
        0.0, 0.0, 0.0, 0.0
    ).scale(p.wheelRadius/4.0).transpose()

    private val inverseVel = Matrix4d(
        1.0, -1.0, -l, 0.0,
        1.0, 1.0, -l, 0.0,
        1.0, -1.0, l, 0.0,
        1.0, 1.0, l, 0.0,
    )
        .transpose()
        .scale(1.0 / p.wheelRadius)

    private val forwardAcc = Matrix4d(
        1.0, 1.0, 1.0, 1.0,
        -1.0, 1.0, -1.0, 1.0,
        -l2, -l2, l2, l2,
        0.0, 0.0, 0.0, 0.0
    )
        .scale((1.0/p.wheelRadius)/sqrt(2.0))
        .scale(1.0/p.weight, 1.0/p.weight, 1.0/p.rotInertia)
        .transpose()

    private val inverseAcc = Matrix4d(
        1.0, -1.0, -1.0/l2, 0.0,
        1.0, 1.0, -1.0/l2, 0.0,
        1.0, -1.0, 1.0/l2, 0.0,
        1.0, 1.0, 1.0/l2, 0.0,
    )
        .transpose()
        .scale(p.wheelRadius*sqrt(2.0)/4.0)
        .scale(1.0/p.weight, 1.0/p.weight, 1.0/p.rotInertia)

    /**
     * Mecanum Forward Kinematics
     *
     * @param wheelVels vector of wheel velocities in rad/s, ordered as `[FL,BL,BR,FR]`
     *
     * @return the robot-relative velocity resulting from the provided `wheelVels` as `Pose2d(m/s, m/s, rad/s)`
     */
    fun mecanumForwardVelKinematics(wheelVels: Vector4d): Pose2d {
        val v = forwardVel * wheelVels

        return Pose2d(v.x,v.y,v.z)
    }

    /**
     * Mecanum Inverse Kinematics
     *
     * @param robotVel the desired robot-relative velocity
     *
     * @return a vector of wheel velocities in rad/s ordered `[FL,BL,BR,FR]` that will result in the desired `robotVel`.
     */
    fun mecanumInverseVelKinematics(robotVel: Pose2d): Vector4d {
        return inverseVel*Vector4d(robotVel.v.x,robotVel.v.y,robotVel.rot, 0.0)
    }

    fun mecanumForwardAccKinematics(wheelTorques: Vector4d): Pose2d =
        (forwardAcc * wheelTorques).let {
            Pose2d(it.x,it.y,it.z)
        }

    fun mecanumInverseAccKinematics(acc: Pose2d): Vector4d =
        inverseAcc * Vector4d(acc.v.x, acc.v.y, acc.rot, 0.0)

    fun mecanumInversePowerKinematics(robotPower: Pose2d): Vector4d {
        val wheelVelocities = mecanumInverseVelKinematics(maxSpeed().componentMul(robotPower))
        return wheelVelocities * (1.0 / p.motor.freeSpeed)
    }

    /**
     * Gives the maximum speed for the drivetrain
     */
    fun maxSpeed() = Pose2d(
            p.motor.freeSpeed * p.wheelRadius,
            p.motor.freeSpeed * p.wheelRadius,
            p.motor.freeSpeed * p.wheelRadius / l)


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
        val pos = Pose2d(x[3], x[4], x[5])
        val vel = Pose2d(x[0],x[1],x[2]).also {
            it.v = Matrix2d().rotate(-pos.rot)*it.v
        }

        val wheelVels = mecanumInverseVelKinematics(vel)

        val motorPowers = Vector4d(u[0],u[1],u[2],u[3])

        // torques produced by each wheel motor
        val torques = Vector4d(
            p.motor.torque(motorPowers.x, wheelVels.x),
            p.motor.torque(motorPowers.y, wheelVels.y),
            p.motor.torque(motorPowers.z, wheelVels.z),
            p.motor.torque(motorPowers.w, wheelVels.w),
        )

        // robot relative accelerations
        val acc = mecanumForwardAccKinematics(torques)

        // make acc field relative
        acc.v = Matrix2d().rotate(pos.rot)*acc.v

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
