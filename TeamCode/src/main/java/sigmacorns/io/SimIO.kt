package sigmacorns.io

import sigmacorns.math.Pose2d
import sigmacorns.sim.RobotModel

const val UPDATE_TIME = 0.05

class SimIO : SigmaIO {
    private val robot = RobotModel()

    private var t = 0.0

    override var driveFL: Double = 0.0
    override var driveBL: Double = 0.0
    override var driveFR: Double = 0.0
    override var driveBR: Double = 0.0
    override var flyWheel0: Double = 0.0
    override var flyWheel1: Double = 0.0
    override var intake: Double = 0.0

    override fun position(): Pose2d = robot.drivetrainState.pos

    override fun velocity(): Pose2d = robot.drivetrainState.vel

    override fun flywheelVelocity(): Double {
        return 0.0
    }

    override fun update() {
        robot.advanceSim(UPDATE_TIME,this)
        t += UPDATE_TIME
    }

    override fun time() = t
}