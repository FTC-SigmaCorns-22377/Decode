package sigmacorns.opmode.auto

import android.app.ActivityManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.joml.Vector2d
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainCenter
import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.flywheelMotor
import sigmacorns.constants.flywheelParameters
import sigmacorns.constants.turretRange
import sigmacorns.control.AimingSystem
import sigmacorns.control.Flywheel
import sigmacorns.control.PollableDispatcher
import sigmacorns.control.ShotPowers
import sigmacorns.control.SpindexerLogic
import sigmacorns.control.Turret
import sigmacorns.control.aim.AutoAimGTSAM
import sigmacorns.control.aim.AutoAimTurretController
import sigmacorns.control.mpc.ContourSelectionMode
import sigmacorns.control.mpc.MPCClient
import sigmacorns.control.mpc.MPCRunner
import sigmacorns.control.mpc.TrajoptEventMarker
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.control.mpc.TrajoptTrajectory
import sigmacorns.io.HardwareIO
import sigmacorns.io.PosePersistence
import sigmacorns.io.rotate
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.test.AutoAimGTSAMTest
import sigmacorns.opmode.test.AutoAimGTSAMTest.Companion.applyRuntimeConfig
import sigmacorns.sim.Balls
import kotlin.math.hypot
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

data class TrajoptAutoData(
    val INTAKE_SEGMENTS: Map<String, List<Pair<Int, Int>>>,
    val SHOT_POWER: Double ,
    val PROJECT_FILE_NAME: (Boolean) -> String , //blue -> name
    val ROOT: String ,
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


open class TrajoptAuto(
    val data: TrajoptAutoData,
    val blue: Boolean
) : SigmaOpMode() {
    lateinit var aiming: AimingSystem

    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir).find {
            it.nameWithoutExtension == data.PROJECT_FILE_NAME(blue)
        }!!

        val trajectories = TrajoptLoader.loadAllTrajectoriesOrdered(projectFile, data.ROOT)

        // Initialize with Flywheel controller
        val flywheel = Flywheel(flywheelMotor, flywheelParameters.inertia, io)
        val spindexerLogic = SpindexerLogic(io, flywheel)
        val dispatcher = PollableDispatcher(io)

        io.configurePinpoint()

        // Set initial position from first trajectory
        trajectories.firstOrNull()?.getInitialSample()!!.let {
            io.setPosition(it.pos.let {
                val v = Vector2d()
                it.v.sub(drivetrainCenter.rotate(it.rot),v)
                Pose2d(v,it.rot)
            })
        }

        // Initialize AimingSystem
        aiming = AimingSystem(io, blue)
        aiming.init(io.position())

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

            runner.start()

            waitForStart()

            val startTime = io.time()
            var zero = false

            val schedule = CoroutineScope(dispatcher).launch {
                if(data.PRELOAD) shootAllBalls(spindexerLogic)
                println("TrajoptAuto: shooting complete")
                for (traj in trajectories) {
                    if(traj.name == "intake_3") {
                        spindexerLogic.fsm.curState = SpindexerLogic.State.ZERO
                        zero = true
                    }
                    println("TrajoptAuto: following traj ${traj.name}")
                    followTrajectory(traj, runner, spindexerLogic)
                    println("TrajoptAuto: done traj ${traj.name}")
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

                // Update aiming system
                if(!zero) aiming.update(dt) else {
                    aiming.turret.fieldRelativeMode = false
                    aiming.turret.targetAngle = 0.0
                    aiming.turret.update(dt)
                }

                // Use AdaptiveTuner for velocity, fallback to zones if not calibrated
                val recommendedVelocity = aiming.getRecommendedFlywheelVelocity()
                if (recommendedVelocity != null) {
                    spindexerLogic.targetVelocityOverride = recommendedVelocity
                } else {
                    val targetDistance = aiming.targetDistance
                    spindexerLogic.targetShotPower = when {
                        targetDistance < ShotPowers.shortDistanceLimit -> ShotPowers.shortShotPower
                        targetDistance < ShotPowers.midDistanceLimit -> ShotPowers.midShotPower
                        else -> ShotPowers.longShotPower
                    }
                    spindexerLogic.targetVelocityOverride = null
                }

                io.update()
            }

            // Save final pose for teleop to resume from
            PosePersistence.savePose(io.position(), storageDir())

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

        // Use AdaptiveTuner if available, otherwise fallback to SHOT_POWER
        val recommendedVelocity = aiming.getRecommendedFlywheelVelocity()
        if (recommendedVelocity != null) {
            spindexerLogic.targetVelocityOverride = recommendedVelocity
        } else {
            spindexerLogic.targetShotPower = data.SHOT_POWER
            spindexerLogic.targetVelocityOverride = null
        }

        while (spindexerLogic.spindexerState.any { it != null }) {
            spindexerLogic.shootingRequested = true
            spindexerLogic.shoot()
            delay(100)
        }

        spindexerLogic.shootingRequested = false
        println("TrajoptAuto: Finished shooting, waiting for IDLE")

//        while (spindexerLogic.currentState != SpindexerLogic.State.IDLE) {
//            delay(10)
//        }

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
        val segments = data.INTAKE_SEGMENTS[traj.name] ?: return emptyList()
        return segments.mapNotNull { (startWp, endWp) ->
            val startIdx = waypointToSampleIndex(traj, startWp) ?: return@mapNotNull null
            val endIdx = waypointToSampleIndex(traj, endWp) ?: return@mapNotNull null
            startIdx..endIdx
        }
    }
}
