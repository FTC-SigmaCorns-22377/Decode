package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.joml.Vector2d
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.turretRange
import sigmacorns.control.PollableDispatcher
import sigmacorns.control.subsystem.ShotPowers
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.control.subsystem.Turret
import sigmacorns.control.aim.AutoAimGTSAM
import sigmacorns.control.aim.AutoAimTurretController
import sigmacorns.control.mpc.ContourSelectionMode
import sigmacorns.control.mpc.MPCClient
import sigmacorns.control.mpc.MPCRunner
import sigmacorns.control.mpc.TrajoptEventMarker
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.control.mpc.TrajoptTrajectory
import sigmacorns.io.HardwareIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.test.AutoAimGTSAMTest
import sigmacorns.opmode.test.AutoAimGTSAMTest.Companion.applyRuntimeConfig
import sigmacorns.sim.Balls
import kotlin.math.hypot
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "RedWallPreLoadPlus9Ball", group = "Auto")
class RedWallPreloadPlus9Ball : SigmaOpMode() {
    companion object {
        // Intake segments: trajectory name -> list of (startWaypointIndex, endWaypointIndex) pairs
        val INTAKE_SEGMENTS: Map<String, List<Pair<Int, Int>>> = mapOf(
            "intake_1" to listOf(2 to 3),
            "intake_2" to listOf(2 to 3),
            "intake_3" to listOf(2 to 3),
        )
        val SHOT_POWER = ShotPowers.longShotPower
        val PROJECT_FILE_NAME = "RedWallPreload"
        val ROOT = "preload"
    }

    lateinit var autoAim: AutoAimGTSAM
    lateinit var turret: Turret
    lateinit var goalPosition: Vector2d

    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir).find {
            it.nameWithoutExtension == PROJECT_FILE_NAME
        }!!

        val trajectories = TrajoptLoader.loadAllTrajectoriesOrdered(projectFile, ROOT)

        val spindexerLogic = SpindexerLogic(io)
        val dispatcher = PollableDispatcher(io)
        turret = Turret(turretRange,io)

        // Set initial position from first trajectory
        trajectories.firstOrNull()?.getInitialSample()?.let { io.setPosition(it.pos) }

        val blue = false
        goalPosition = Vector2d(if(blue)-1.480126 else 1.480126, 1.598982)
        autoAim = AutoAimGTSAM(
            landmarkPositions = mapOf(),
            goalPosition,
            initialPose = io.position(),
            estimatorConfig = AutoAimGTSAMTest.buildEstimatorConfig()
        )

        applyRuntimeConfig(autoAim)
        autoAim.enabled = true

        // Configure Limelight
        (io as? HardwareIO)?.limelight?.pipelineSwitch(1)

        MPCClient(
            drivetrainParameters,
            Network.LIMELIGHT,
            contourSelectionMode = ContourSelectionMode.POSITION,
            preIntegrate = 30.milliseconds,
            sampleLookahead = 0
        ).use { mpc ->
            val state = State(
                0.0, io.position(), Pose2d(),
                Pose2d(Vector2d(), 0.0), 0.0, 0.0, 0.seconds
            )

            val runner = MPCRunner(mpc)

            spindexerLogic.spindexerState = mutableListOf(
                Balls.Green,
                Balls.Purple,
                Balls.Purple
            )

            waitForStart()

            runner.start()

            val schedule = CoroutineScope(dispatcher).launch {
                shootAllBalls(spindexerLogic)
                for (traj in trajectories) {
                    followTrajectory(traj, mpc, spindexerLogic)
                }
                println("TrajoptAuto: All trajectories complete")
                delay(3000)
            }

            // Main loop
            var lastTime = io.time()
            while (opModeIsActive() && !schedule.isCompleted) {
                val t = io.time()
                state.update(io)
                dispatcher.update()

                runner.updateState(state.mecanumState, 12.0, t)
                runner.driveWithMPC(io, io.voltage())

                val dt = t - lastTime
                spindexerLogic.update( dt, 12.0 / io.voltage())
                lastTime = t

                autoAim.update(io.position(), turret.pos, null)
                AutoAimTurretController.update(
                    autoAim, turret, io,
                    goalPosition.x, goalPosition.y, dt
                )

                val pose = autoAim.fusedPose
                val goal = goalPosition
                val targetDistance = hypot(goal.x - pose.v.x, goal.y - pose.v.y)

                spindexerLogic.targetShotPower = when {
                    targetDistance < ShotPowers.shortDistanceLimit -> ShotPowers.shortShotPower
                    targetDistance < ShotPowers.midDistanceLimit -> ShotPowers.midShotPower
                    else -> ShotPowers.longShotPower
                }

                io.update()
            }

            runner.stop()
        }
    }

    /**
     * Follow a single trajectory, toggling intake by sample index and
     * shooting when the MPC progress reaches a "shoot" event marker.
     * Blocks until the trajectory is complete AND all shots have fired.
     */
    private suspend fun followTrajectory(
        traj: TrajoptTrajectory,
        mpc: MPCClient,
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

        while (!mpc.isTrajectoryComplete()) {
            val sampleI = mpc.latestSampleI
            println("TrajoptAuto: sampleI=$sampleI")
            // Toggle intake based on sample index
            val shouldIntake = intakeRanges.any { sampleI in it }
            when {
                shouldIntake && !intaking -> {
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
        spindexerLogic.targetShotPower = SHOT_POWER

        while (spindexerLogic.spindexerState.any { it != null }) {
            spindexerLogic.shootingRequested = true
            spindexerLogic.shoot()
            delay(100)
        }

        spindexerLogic.shootingRequested = false
        println("TrajoptAuto: Finished shooting, waiting for IDLE")

        while (spindexerLogic.currentState != SpindexerLogic.State.IDLE) {
            delay(10)
        }

        println("TrajoptAuto: FSM idle, continuing")
    }

    // ---- Sample-index helpers ----

    private fun waypointToSampleIndex(traj: TrajoptTrajectory, waypointIdx: Int): Int? {
        val waypointTime = traj.waypointTimes.getOrNull(waypointIdx) ?: return null
        return traj.samples.indexOfFirst { it.timestamp >= waypointTime }
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
        val segments = INTAKE_SEGMENTS[traj.name] ?: return emptyList()
        return segments.mapNotNull { (startWp, endWp) ->
            val startIdx = waypointToSampleIndex(traj, startWp) ?: return@mapNotNull null
            val endIdx = waypointToSampleIndex(traj, endWp) ?: return@mapNotNull null
            startIdx..endIdx
        }
    }
}



