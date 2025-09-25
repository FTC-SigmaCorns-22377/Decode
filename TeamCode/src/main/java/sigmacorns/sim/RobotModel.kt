package sigmacorns.sim

import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d

const val MECANUM_DT: Double = 0.001

/**
 * A virtual model of the robot dynamics used for simulation
 */
class RobotModel {

    /**
     * Current state of the mecanum drivetrain.
     * @see MecanumDynamics.MecanumState
     */
    var drivetrainState = MecanumDynamics.MecanumState(
        Pose2d(),
        Pose2d()
    )

    // specs of a goBilda motor without a gearbox (Modern Robotics 12v DC motor)
    // from https://motors.vex.com/other-motors/modern-robotics-12vdc

    /**
     * Modern Robotics 12v DC motor top speed in rad/s
     */
    private val bareMotorTopSpeed = 617.84

    /**
     * Modern Robotics 12v DC motor stall torque in N*m
     */
    private val bareMotorStallTorque = 0.187


    /**
     * Gear ratio used with a Modern Robotics 12v DC motor.
     * See goBilda website for values
     */
    val gearRatio = 13.7

    // dynamics of the drivetrain
    val drivetrain = MecanumDynamics(
        0.048,
        0.4,
        bareMotorTopSpeed/gearRatio,
        bareMotorStallTorque*gearRatio,
        10.0,
        0.0870966
    )

    /**
     * Advances the simulation
     * @param t the time to advance the simulation by (s)
     */
    fun advanceSim(t: Double, io: SimIO) {
        val wheelMotorUs = doubleArrayOf(
            io.driveFL,
            io.driveBL,
            io.driveBR,
            io.driveFR
        )

        drivetrainState = drivetrain.integrate(t,MECANUM_DT, wheelMotorUs, drivetrainState)
    }
}