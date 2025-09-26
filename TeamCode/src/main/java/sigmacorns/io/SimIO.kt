package sigmacorns.io

import sigmacorns.math.Pose2d
import sigmacorns.sim.RobotModel
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

val SIM_UPDATE_TIME = 5.milliseconds

class SimIO : SigmaIO {
    val robot = RobotModel()

    private var t = 0.seconds

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
        robot.advanceSim(SIM_UPDATE_TIME.toDouble(DurationUnit.SECONDS),this)
        t += SIM_UPDATE_TIME
    }

    override fun time() = t
}