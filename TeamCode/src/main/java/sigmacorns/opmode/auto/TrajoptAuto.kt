package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.joml.Vector2d
import sigmacorns.constants.drivetrainCenter
import sigmacorns.control.subsystem.AimingSystem
import sigmacorns.control.Robot
import sigmacorns.control.subsystem.ShotPowers
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.control.mpc.MPCRunner
import sigmacorns.control.mpc.TrajoptEventMarker
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.control.mpc.TrajoptTrajectory
import sigmacorns.io.DrakeSimIO
import sigmacorns.io.PosePersistence
import sigmacorns.io.rotate
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.Balls

data class TrajoptAutoData(
    val INTAKE_SEGMENTS: Map<String, List<Pair<Number, Number>>>,
    val SHOT_POWER: Double,
    val PROJECT_FILE_NAME: (Boolean) -> String, //blue -> name
    val ROOT: String,
    val PRELOAD: Boolean
)
val RedWallPreload = TrajoptAutoData (
    // Intake segments: trajectory name -> list of (startWaypointIndex, endWaypointIndex) pairs
    INTAKE_SEGMENTS = mapOf(
        "intake_1" to listOf(2 to 3),
        "intake_2" to listOf(2 to 3),
    ),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { "redwallpreload" },
    ROOT = "preload", false
)

val base = TrajoptAutoData(
    INTAKE_SEGMENTS= mapOf(
        "intake_1" to listOf(1 to 2),
        "intake_2" to listOf(1 to 2),
    ),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { if(it) "base-mirrored" else "base" },
    ROOT = "intake_1",
    PRELOAD = true
)

val baseFar = TrajoptAutoData(
    INTAKE_SEGMENTS= mapOf(
        "intake_1" to listOf(1 to 2),
        "intake_2" to listOf(1 to 2),
    ),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { if(it) "basefar" else "basefarred" },
    ROOT = "intake_1",
    PRELOAD = true
)

val intakeTestAuto = TrajoptAutoData(
    INTAKE_SEGMENTS= mapOf(
        "Trajectory 1" to listOf(0 to 1),
        "Trajectory 2" to listOf(0 to 0.5),
    ),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { "intake" },
    ROOT = "Trajectory 1",
    PRELOAD = false
)

val traj3test = TrajoptAutoData(
    INTAKE_SEGMENTS= mapOf(),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { "intake" },
    ROOT = "Trajectory 3",
    PRELOAD = false
)

@Autonomous(name = "Auto Red Far", group = "Auto", preselectTeleOp = "TeleopRed")
class AutoRedFar: TrajoptAuto(base,false)

@Autonomous(name = "Auto Blue Far", group = "Auto", preselectTeleOp = "TeleopBlue")
class AutoBlueFar: TrajoptAuto(base, true)

@Autonomous(name = "Auto Red Far Full", group = "Auto", preselectTeleOp = "TeleopRed")
class AutoRedFarFull: TrajoptAuto(baseFar,false)

@Autonomous(name = "Auto Blue Far Full", group = "Auto", preselectTeleOp = "TeleopBlue")
class AutoBlueFarFull: TrajoptAuto(baseFar, true)

@Autonomous(name = "RedWallAuto", group = "Auto", preselectTeleOp = "TeleopRed")
class RedWallAuto: TrajoptAuto(RedWallPreload, false)


@Autonomous(name = "IntakeTestAuto", group = "Auto")
class IntakeTestAuto: TrajoptAuto(intakeTestAuto, false)

@Autonomous(name = "FAST", group = "Auto")
class FAST: TrajoptAuto(traj3test, false)

open class TrajoptAuto(
    val data: TrajoptAutoData,
    val blue: Boolean
) : SigmaOpMode() {
    lateinit var aiming: AimingSystem

    override fun runOpMode() {
        val robot = Robot(io,blue)

        val robotDir = TrajoptLoader.robotTrajoptDir(io is DrakeSimIO)
        val projectFile = TrajoptLoader.findProjectFiles(robotDir).find {
            it.nameWithoutExtension == data.PROJECT_FILE_NAME(blue)
        }!!
        val trajectories = TrajoptLoader.loadAllTrajectoriesOrdered(projectFile, data.ROOT)

        val initPos = trajectories.firstOrNull()?.getInitialSample()!!.let {
            it.pos.let {
                val v = Vector2d()
                it.v.sub(drivetrainCenter.rotate(it.rot),v)
                Pose2d(v,it.rot)
            }
        }
        robot.use {
            robot.init(initPos,false)
            robot.startMPC()

            robot.logic.spindexerState = mutableListOf(
                Balls.Green,
                Balls.Purple,
                Balls.Purple
            )
            (io as? DrakeSimIO)?.preloadSpindexer(robot.logic.spindexerState)

            waitForStart()

            val startTime = io.time()
            var zero = false

            val schedule = robot.scope.launch {
                if(data.PRELOAD) shootAllBalls(robot.logic)
                println("TrajoptAuto: shooting complete")
                for (traj in trajectories) {
                    if(traj.name == "intake_3") {
                        robot.logic.fsm.curState = SpindexerLogic.State.ZERO
                        zero = true
                    }
                    println("TrajoptAuto: following traj ${traj.name}")
                    followTrajectory(traj, robot.runner!!, robot.logic)
                    println("TrajoptAuto: done traj ${traj.name}")
                }
                println("TrajoptAuto: All trajectories complete")
                delay(3000)
            }

            // Main loop
            while (opModeIsActive() && !schedule.isCompleted) {
                robot.update()
                io.update()
            }

            // Save final pose for teleop to resume from
            PosePersistence.savePose(io.position(), storageDir())
        }
    }

    /**
     * Follow a single trajectory, toggling intake by sample index and
     * shooting when the MPC progress reaches a "shoot" event marker.
     * Blocks until the trajectory is complete AND all shots have fired.
     */
    private suspend fun followTrajectory(
        traj: TrajoptTrajectory,
        mpc: MPCRunner,
        spindexerLogic: SpindexerLogic,
    ) {
        println("TrajoptAuto: Starting trajectory '${traj.name}'")

        val intakeRanges = computeIntakeSampleRanges(traj)
        val shootSampleIndices = traj.eventMarkers
            .filter { it.name == "shoot" }
            .map { eventMarkerToSampleIndex(traj, it) }
            .sorted()

        println("TrajoptAuto: intakeRanges=$intakeRanges")
        println("TrajoptAuto: shootSampleIndices=$shootSampleIndices")

        mpc.setTarget(traj)
        var intaking = false
        var nextShootIdx = 0

        while (!mpc.isTrajectoryComplete(traj)) {
            val sampleI = mpc.latestSampleI
            println("TrajoptAuto: sampleI=$sampleI")
            // Toggle intake based on sample index
            val shouldIntake = intakeRanges.any { sampleI in it }
            when {
                shouldIntake -> {
                    println("TrajoptAuto: intaking")
                    spindexerLogic.startIntaking()
                    intaking = true
                }
                !shouldIntake && intaking -> {
                    println("TrajoptAuto: stop intaking")
                    spindexerLogic.stopIntaking()
                    intaking = false
                }
            }

            // Shoot when we reach a shoot marker -- blocks until all balls are fired
            if (nextShootIdx < shootSampleIndices.size && sampleI >= shootSampleIndices[nextShootIdx]) {
                println("TrajoptAuto: Reached shoot marker at sample $sampleI")
                if (intaking) {
                    spindexerLogic.stopIntaking()
                    intaking = false
                }
                shootAllBalls(spindexerLogic)
                nextShootIdx++
            }

            delay(1)
        }

        if (intaking) spindexerLogic.stopIntaking()

        // Fire any shoot markers not reached during trajectory (e.g. marker at final waypoint)
        while (nextShootIdx < shootSampleIndices.size) {
            println("TrajoptAuto: Shooting remaining marker at trajectory end")
            shootAllBalls(spindexerLogic)
            nextShootIdx++
        }

        println("TrajoptAuto: Trajectory '${traj.name}' complete")
    }

    /**
     * Shoot all loaded balls and wait for the spindexer FSM to return to IDLE.
     */
    private suspend fun shootAllBalls(spindexerLogic: SpindexerLogic) {
        println("TrajoptAuto: Shooting")

        while (spindexerLogic.spindexerState.any { it != null }) {
            spindexerLogic.shootingRequested = true
            spindexerLogic.shoot()
            delay(100)
        }

        spindexerLogic.shootingRequested = false
        println("TrajoptAuto: Finished shooting, waiting for IDLE")

        println("TrajoptAuto: FSM idle, continuing")
    }

    // ---- Sample-index helpers ----

    private fun waypointToSampleIndex(traj: TrajoptTrajectory, waypointIdx: Number): Int? {
        val i = waypointIdx.toDouble().toInt()
        val startTime = traj.waypointTimes.getOrNull(i)!!
        val endTime = traj.waypointTimes.getOrNull(i+1) ?: traj.waypointTimes.last()
        val targetTime = startTime + (endTime - startTime) * (waypointIdx.toDouble() - i.toDouble())
        return traj.samples.indexOfFirst { it.timestamp >= targetTime }
            .takeIf { it >= 0 } ?: traj.samples.lastIndex
    }

    private fun eventMarkerToSampleIndex(traj: TrajoptTrajectory, marker: TrajoptEventMarker): Int {
        val startTime = traj.waypointTimes.getOrNull(marker.waypointIndex)
            ?: return traj.samples.lastIndex
        val endTime = traj.waypointTimes.getOrNull(marker.waypointIndex + 1) ?: startTime
        val targetTime = startTime + (endTime - startTime) * marker.percentage
        return traj.samples.indexOfFirst { it.timestamp >= targetTime }
            .takeIf { it >= 0 } ?: traj.samples.lastIndex
    }

    private fun computeIntakeSampleRanges(traj: TrajoptTrajectory): List<IntRange> {
        val segments = data.INTAKE_SEGMENTS[traj.name] ?: return emptyList()
        return segments.mapNotNull { (startWp, endWp) ->
            val startIdx = waypointToSampleIndex(traj, startWp) ?: return@mapNotNull null
            val endIdx = waypointToSampleIndex(traj, endWp) ?: return@mapNotNull null
            startIdx..endIdx
        }
    }
}
