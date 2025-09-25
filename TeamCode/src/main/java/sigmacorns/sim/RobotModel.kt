package sigmacorns.sim

import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d

const val MECANUM_DT: Double = 0.001

class RobotModel {
    var drivetrainState = MecanumDynamics.MecanumState(
        Pose2d(),
        Pose2d()
    )

    val bareMotorTopSpeed = 617.84
    val bareMotorStallTorque = 0.187
    val gearRatio = 13.7
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