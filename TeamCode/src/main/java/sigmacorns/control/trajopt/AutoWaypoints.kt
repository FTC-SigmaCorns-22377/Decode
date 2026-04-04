package sigmacorns.control.trajopt

import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.launch
import org.joml.Vector2d
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.ShotZone
import sigmacorns.math.Pose2d
import sigmacorns.math.closestPointOnConvexPolygon
import sigmacorns.sim.MecanumState
import sigmacorns.subsystem.IntakeTransfer
import kotlin.math.PI
import kotlin.time.Duration.Companion.seconds

enum class AutoAction {
    INTAKE,
    IDLE,
    SHOOT
}

enum class IntakePoints {
    SPIKE_FAR,
    SPIKE_MED,
    SPIKE_CLOSE,
    GATE,
    HUMAN,
    PRELOAD,
    LAUNCH_CLOSE,
    LAUNCH_FAR
}

class AutoWaypoint(val p: Pose2d, val v: Vector2d, val action: AutoAction = AutoAction.IDLE)

class AutoAuto(
    val blue: Boolean,
    val pos: Pose2d,
    val intakeSpeed: Double,
    val intakeStartX: Double,
    val intakeEndX: (FieldLandmarks.Spike) -> Double,
    sequence: List<IntakePoints>
) {
    fun spikeIntake(spike: FieldLandmarks.Spike) = listOf(
        AutoWaypoint(
            Pose2d(if(blue) -intakeStartX else intakeStartX, spike.v(blue).y, if(blue) PI else 0.0),
            Vector2d(if(blue) -intakeSpeed else intakeSpeed, 0.0),
            AutoAction.INTAKE
        ),
        AutoWaypoint(
            Pose2d(if(blue) -intakeEndX(spike) else intakeEndX(spike), spike.v(blue).y, if(blue) PI else 0.0),
            Vector2d(),
            AutoAction.IDLE
        )
    )

    fun shootZone(zone: ShotZone): AutoWaypoint {
        val p = closestPointOnConvexPolygon(zone.pts, lastPos.v)
        return AutoWaypoint(Pose2d(p,lastPos.rot), Vector2d(), AutoAction.SHOOT)
    }

    private var lastPos = pos

    val waypoints: List<AutoWaypoint> = sequence.flatMap {
        val res = when(it) {
            IntakePoints.SPIKE_FAR -> spikeIntake(FieldLandmarks.Spike.FAR)
            IntakePoints.SPIKE_MED -> spikeIntake(FieldLandmarks.Spike.MED)
            IntakePoints.SPIKE_CLOSE -> spikeIntake(FieldLandmarks.Spike.CLOSE)
            IntakePoints.GATE -> TODO()
            IntakePoints.HUMAN -> TODO()
            IntakePoints.PRELOAD -> listOf(AutoWaypoint(pos, Vector2d(), AutoAction.SHOOT))
            IntakePoints.LAUNCH_CLOSE -> listOf(shootZone(ShotZone.CLOSE))
            IntakePoints.LAUNCH_FAR -> listOf(shootZone(ShotZone.FAR))
        }

        res.lastOrNull()?.let {
            lastPos = it.p
        }

        res
    }

    suspend fun run(robot: Robot, waypoint: AutoWaypoint) {
        val path: suspend ()->Unit = suspend {
            robot.ltv.runWaypointToCompletion(
                MecanumState(robot.io.velocity(),robot.io.position()),
                MecanumState(Pose2d(waypoint.v,0.0),waypoint.p),
                5.seconds,
                robot.io
            );
        }

        when (waypoint.action) {
            AutoAction.INTAKE -> {
                robot.intakeCoordinator.startIntake()
                path()
            }
            AutoAction.IDLE -> {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                path()
            }
            AutoAction.SHOOT -> {
                robot.aimTurret = true
                robot.aimFlywheel = true

                path()

                val hold = robot.scope.launch {
                    try {
                        robot.ltv.holdPos(robot.io,waypoint.p)
                    } catch (e: CancellationException) { }
                }

                while (robot.beamBreak.ballCount > 0) {
                    robot.aim.shotRequested = true
                }

                hold.cancel()
            }
        }
    }
}