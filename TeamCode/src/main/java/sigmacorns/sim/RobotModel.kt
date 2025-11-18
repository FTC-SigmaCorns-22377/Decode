package sigmacorns.sim

import androidx.lifecycle.viewmodel.CreationExtras
import org.joml.Vector3d
import sigmacorns.constants.BALL_EXIT_SPEED_PER_RADIAN
import sigmacorns.constants.BALL_GRAVITY_MAGNITUDE
import sigmacorns.constants.BALL_LAUNCH_ANGLE_RADIANS
import sigmacorns.constants.BALL_LAUNCH_HEIGHT_METERS
import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.flywheelParameters
import sigmacorns.constants.spindexerParameters
import sigmacorns.sim.SpindexerState
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

private const val PROJECTILE_DT: Double = 0.0005

const val MECANUM_DT: Double = 0.0005
const val FLYWHEEL_DT: Double = 0.0002
const val SPINDEXER_DT: Double = 0.002

private data class Projectile(
    val id: Int,
    var state: ProjectileState,
    val path: MutableList<Vector3d>,
)

/**
 * A virtual model of the robot dynamics used for simulation
 */
class RobotModel {

    var flywheelState = FlywheelState()
    var spindexerState = SpindexerState(
        0.0,
        listOf(Balls.Empty,Balls.Empty,Balls.Empty),
        0.0
    )

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
    val spindexer = SpindexerDynamics(spindexerParameters)

    private val projectiles = mutableListOf<Projectile>()
    private var nextProjectileId = 0

    private val projectileDynamics = ProjectileDynamics(BALL_GRAVITY_MAGNITUDE)

    var ballLaunchAngleRadians: Double = BALL_LAUNCH_ANGLE_RADIANS
    var ballExitSpeedPerRad: Double = BALL_EXIT_SPEED_PER_RADIAN
    var ballLaunchHeightMeters: Double = BALL_LAUNCH_HEIGHT_METERS

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

        val flywheelInputs = doubleArrayOf(io.shooter)
        flywheelState = flywheel.integrate(t, FLYWHEEL_DT, flywheelInputs, flywheelState)

        val spindexerInputs = doubleArrayOf(io.spindexer)
        spindexerState = spindexer.integrate(t,SPINDEXER_DT,spindexerInputs,spindexerState)
        integrateProjectiles(t)
    }

    fun launchBall() {
        val muzzleSpeed = flywheelState.omega * ballExitSpeedPerRad
        if (muzzleSpeed <= 0.0) {
            return
        }

        val heading = drivetrainState.pos.rot
        val horizontalSpeed = muzzleSpeed * cos(ballLaunchAngleRadians)

        val vx = drivetrainState.vel.v.x + horizontalSpeed * cos(heading)
        val vy = drivetrainState.vel.v.y + horizontalSpeed * sin(heading)
        val vz = muzzleSpeed * sin(ballLaunchAngleRadians)

        val state = ProjectileState(
            position = Vector3d(drivetrainState.pos.v.x, drivetrainState.pos.v.y, ballLaunchHeightMeters),
            velocity = Vector3d(vx, vy, vz),
            active = true,
        )

        val projectile = Projectile(
            id = nextProjectileId++,
            state = state,
            path = mutableListOf(Vector3d(state.position)),
        )

        projectiles.add(projectile)
    }

    private fun integrateProjectiles(totalDt: Double) {
        if (projectiles.isEmpty()) {
            return
        }

        var remaining = totalDt
        while (remaining > 1e-9) {
            val step = min(PROJECTILE_DT, remaining)
            projectiles.forEach { projectile ->
                if (!projectile.state.active) {
                    return@forEach
                }
                val updated = projectileDynamics.integrate(step, PROJECTILE_DT, projectile.state)
                projectile.state = updated
                projectile.path.add(Vector3d(projectile.state.position))
            }

            remaining -= step
        }
    }
}
