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
    override var flywheel: Double = 0.0
    override var intake: Double = 0.0
    override var turret: Double = 0.0
    override var turretLeft: Double = 0.5
    override var turretRight: Double = 0.5
    override var hood: Double = 0.5
    override var blocker: Double = 0.0

    override fun beamBreak1(): Boolean = false
    override fun beamBreak2(): Boolean = false
    override fun beamBreak3(): Boolean = false

    override fun position(): Pose2d = robot.drivetrainState.pos

    override fun velocity(): Pose2d = robot.drivetrainState.vel

    override fun flywheelVelocity(): Double {
        return robot.flywheelState.omega
    }

    override fun intake1RPM(): Double {
        return 0.0
    }

    override fun turretPosition(): Double {
        return 0.0 // Todo: Implement turret simulation
    }

    override fun setTurretPosition(Offset: Double) {
        TODO("Not yet implemented")
    }

    override fun update() {
        robot.advanceSim(SIM_UPDATE_TIME.toDouble(DurationUnit.SECONDS),this)
        t += SIM_UPDATE_TIME
    }

    override fun setPosition(p: Pose2d) {
        robot.drivetrainState.pos = p
    }

    override fun time() = t

    override fun configurePinpoint() {

    }

    override fun voltage(): Double {
        return 12.0
    }
}
