package sigmacorns.io

import org.joml.Vector3d
import sigmacorns.math.Pose2d
//import sigmacorns.sim.ProjectileSnapshot
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
    override var shooter: Double = 0.0
    override var intake: Double = 0.0

    override fun position(): Pose2d = robot.drivetrainState.pos

    override fun velocity(): Pose2d = robot.drivetrainState.vel

    override fun flywheelVelocity(): Double {
        return robot.flywheelState.omega
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

//    fun launchBall() {
//        robot.launchBall()
//    }
//
//    fun projectiles(): List<ProjectileSnapshot> = robot.projectileSnapshots()
//
//    fun logProjectiles(rr: RerunLogging, prefix: String = "balls") {
//        projectiles().forEach { projectile ->
//            if (projectile.path.isNotEmpty()) {
//                val path = if (projectile.path.size == 1) {
//                    val point = projectile.path.first()
//                    listOf(point, Vector3d(point))
//                } else {
//                    projectile.path
//                }
//                rr.logLineStrip("$prefix/ball-${projectile.id}", path)
//            }
//        }
//    }
}
