package sigmacorns.io

import sigmacorns.math.Pose2d
import sigmacorns.sim.SimRobot

const val UPDATE_TIME = 0.05

class SimIO : SigmaIO {
    override var driveFL: Double = 0.0
    override var driveBL: Double = 0.0
    override var driveFR: Double = 0.0
    override var driveBR: Double = 0.0
    override var flyWheel: Double = 0.0
    override var guidingArm: Double = 0.0
    override var intakeMotor: Double = 0.0

    private val robot = SimRobot()

    override fun position(): Pose2d = robot.drivetrainState.pos

    override fun velocity(): Pose2d = robot.drivetrainState.vel

    override fun flywheelVelocity(): Double {
        TODO("Not yet implemented")
    }

    override fun update() {
        robot.advanceSim(UPDATE_TIME,this)
    }
}