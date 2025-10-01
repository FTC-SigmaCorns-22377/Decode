package sigmacorns.sim

import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.flywheelParameters
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d

const val MECANUM_DT: Double = 0.0005
const val FLYWHEEL_DT: Double = 0.0002

/**
 * A virtual model of the robot dynamics used for simulation
 */
class RobotModel {

    var flywheelState = FlywheelState()

    /**
     * Current state of the mecanum drivetrain.
     * @see MecanumState
     */
    var drivetrainState = MecanumState(
        Pose2d(),
        Pose2d()
    )

    // dynamics of the drivetrain
    val drivetrain = MecanumDynamics(drivetrainParameters)
    val flywheel = FlywheelDynamics(flywheelParameters)

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

        val flywheelInputs = doubleArrayOf(
            io.flyWheel0,
            io.flyWheel1
        )
        flywheelState = flywheel.integrate(t, FLYWHEEL_DT, flywheelInputs, flywheelState)
    }
}
