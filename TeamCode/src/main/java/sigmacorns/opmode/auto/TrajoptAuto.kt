package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import org.joml.Vector2d
import sigmacorns.constants.drivetrainCenter
import sigmacorns.constants.turretRange
import sigmacorns.constants.turretTicksPerRad
import sigmacorns.control.subsystem.AimingSystem
import sigmacorns.control.Robot
import sigmacorns.control.subsystem.ShotPowers
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.control.mpc.MPCRunner
import sigmacorns.control.mpc.TrajoptEventMarker
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.control.mpc.TrajoptTrajectory
import sigmacorns.io.MotifPersistence
import sigmacorns.io.PosePersistence
import sigmacorns.io.rotate
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.Balls
import kotlin.math.PI
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

data class TrajoptAutoData(
    val INTAKE_SEGMENTS: Map<String, List<Pair<Number, Number>>>,
    val SHOT_POWER: Double,
    val PROJECT_FILE_NAME: (Boolean) -> String, //blue -> name
    val ROOT: String,
    val PRELOAD: Boolean,
    val initTurretAngle: Double = 0.0,
    val RUN_MOTIF: Boolean = false
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
        "intake_1" to listOf(1 to 2.3),
        "intake_2" to listOf(1 to 2.3),
    ),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { if(it) "base-mirrored" else "base" },
    ROOT = "intake_1",
    PRELOAD = true,
    initTurretAngle = -PI/2.0
)

val baseBlue = TrajoptAutoData(
    INTAKE_SEGMENTS= mapOf(
        "intake_1" to listOf(1 to 2),
        "intake_2" to listOf(1 to 2),
        "intake_3" to listOf(1 to 2)
    ),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { if(it) "base-mirrored" else "baseRed" },
    ROOT = "intake_1",
    PRELOAD = true,
    initTurretAngle = -PI/2.0
)

val baseFar = TrajoptAutoData(
    INTAKE_SEGMENTS= mapOf(
        "intake_1" to listOf(1 to 2.2),
        "intake_2" to listOf(1 to 2.2),
    ),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { if(it) "basefar" else "basefar_mirrored" },
    ROOT = "intake_1",
    PRELOAD = true,
    initTurretAngle = -PI/2.0
)

val baseFarShort = TrajoptAutoData(
    INTAKE_SEGMENTS= mapOf(
        "intake_1" to listOf(1 to 2.2),
    ),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { if(it) "basefarShort" else "basefarShort_mirrored" },
    ROOT = "intake_1",
    PRELOAD = true,
    initTurretAngle = -PI/2.0
)

val intakeTestAuto = TrajoptAutoData(
    INTAKE_SEGMENTS= mapOf(
        "Trajectory 1" to listOf(1 to 2),
        "Trajectory 2" to listOf(0 to 0.5),
        "Trajectory 4" to listOf(1 to 2),
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

val near = TrajoptAutoData(
    INTAKE_SEGMENTS= mapOf(
        "Trajectory 2" to listOf(2 to 3.3),
        "Trajectory 3" to listOf(1 to 2.3),
    ),
    SHOT_POWER = ShotPowers.longShotPower,
    PROJECT_FILE_NAME = { if(it) "near" else "near_mirrored" },
    ROOT = "Trajectory 1",
    PRELOAD = false,
    RUN_MOTIF = true
)

@Autonomous(name = "Auto Red Far", group = "Auto", preselectTeleOp = "TeleopRed")
class AutoRedFar: TrajoptAuto(base,false)

@Autonomous(name = "Auto Blue Far", group = "Auto", preselectTeleOp = "TeleopBlue")
class AutoBlueFar: TrajoptAuto(base, true)

@Autonomous(name = "Auto Red Far Full", group = "Auto", preselectTeleOp = "TeleopRed")
class AutoRedFarFull: TrajoptAuto(baseFar,false)

@Autonomous(name = "Auto Blue Far Full", group = "Auto", preselectTeleOp = "TeleopBlue")
class AutoBlueFarFull: TrajoptAuto(baseFar, true)

@Autonomous(name = "Auto Red Far Short", group = "Auto", preselectTeleOp = "TeleopRed")
class AutoRedFarShort: TrajoptAuto(baseFarShort,false)

@Autonomous(name = "Auto Blue Far Short", group = "Auto", preselectTeleOp = "TeleopBlue")
class AutoBlueFarShort: TrajoptAuto(baseFarShort, true)

//@Autonomous(name = "RedWallAuto", group = "Auto", preselectTeleOp = "TeleopRed")
//class RedWallAuto: TrajoptAuto(RedWallPreload, false)

@Autonomous(name = "BlueMixed", group = "Auto", preselectTeleOp = "TeleopBlue")
class BlueMixed: TrajoptAuto(baseBlue, true)


//@Autonomous(name = "IntakeTestAuto", group = "Auto")
//class IntakeTestAuto: TrajoptAuto(intakeTestAuto, false)
//
//@Autonomous(name = "FAST", group = "Auto")
//class FAST: TrajoptAuto(traj3test, false)

@Autonomous(name = "BlueNear", group = "Auto")
class BlueNear: TrajoptAuto(near, true)

@Autonomous(name = "RedNear", group = "Auto")
class RedNear: TrajoptAuto(near, false)

open class TrajoptAuto(
    val data: TrajoptAutoData,
    val blue: Boolean
) : SigmaOpMode() {
    lateinit var aiming: AimingSystem

    var runMotif = data.RUN_MOTIF

    var zero = false
    var zeroTime: Duration? = null

    override fun runOpMode() {
        val robot = Robot(io,blue)
        //val turretInitTicks = turretRange.posToTick(data.initTurretAngle).toInt()
        val turretInitTicks = (data.initTurretAngle*turretTicksPerRad*if(blue)1.0 else -1.0)
        println("TURRET INIT TICKS=$turretInitTicks")
        io.setTurretPosition(turretInitTicks)

        val robotDir = TrajoptLoader.robotTrajoptDir()
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

            robot.logic.spindexerState = mutableListOf(
                Balls.Green,
                Balls.Purple,
                Balls.Purple
            )

            // Start MPC solver on limelight and pre-warm with first trajectory
            println("TrajoptAuto: sleeping 2000ms before startMPCSolver")
            Thread.sleep(2000)
            println("TrajoptAuto: calling startMPCSolver at t=${io.time()}")
            robot.startMPCSolver()
            println("TrajoptAuto: startMPCSolver done, sleeping 1000ms")
            Thread.sleep(1000)
            println("TrajoptAuto: calling initMPC at t=${io.time()}")
            robot.initMPC()
            println("TrajoptAuto: initMPC done at t=${io.time()}")

            // Switch to apriltag for motif detection during init
            robot.startApriltag()
            var detectedMotifId: Int? = null
            while (opModeInInit()) {
                val id = robot.pollMotif()
                if (id != null) detectedMotifId = id
                telemetry.addData("Motif", detectedMotifId?.toString() ?: "Scanning...")
                telemetry.addData("Pipeline", robot.pipeline())
                telemetry.addData("Position", io.position())
                telemetry.update()
            }

            // Init done — start MPC runner
            println("TrajoptAuto: calling startMPCRunner at t=${io.time()}")
            robot.startMPCRunner()
            println("TrajoptAuto: startMPCRunner done at t=${io.time()}")

            // Apply motif if already detected during init
            if (detectedMotifId != null) {
                robot.applyDetectedMotif(detectedMotifId)
                MotifPersistence.saveMotif(detectedMotifId, storageDir())
                robot.idleLimelight()
            }
            robot.logic.spinupRequested = true

            val schedule = robot.scope.launch {
                if(data.PRELOAD) shootAllBalls(robot.logic)
                robot.prewarm = false
                robot.logic.spinupRequested = true
                println("TrajoptAuto: shooting complete")
                for (traj in trajectories) {
                    if(traj.name == "zero") {
                        robot.logic.fsm.curState = SpindexerLogic.State.ZERO
                        if(!zero) zeroTime = io.time()
                        zero = true
                    }
                    println("TrajoptAuto: following traj ${traj.name}")
                    followTrajectory(traj, robot.runner!!, robot.logic)
                    println("TrajoptAuto: done traj ${traj.name}")
                }
                println("TrajoptAuto: All trajectories complete")
                delay(3000)
                if(zeroTime!=null) while (io.time() < zeroTime!! + 2.seconds) yield()
            }

            // Main loop — continue polling for motif if not yet detected
            var motifDetected = detectedMotifId != null
            val startTime = io.time()
            val goal = robot.aim.autoAim.turretTargeting.goalPosition

            trajectories.firstOrNull()?.let {
                println("TrajoptAuto: calling prewarmMPC for '${it.name}' at t=${io.time()}")
                robot.prewarmMPC(it)
                println("TrajoptAuto: prewarmMPC done at t=${io.time()}")
            }

            while (opModeIsActive() && !schedule.isCompleted && zeroTime?.let {
                io.time() - it > 2.seconds
                } ?: false) {
                val newRunMotif = data.RUN_MOTIF && !motifDetected && (io.time() - startTime)<5.seconds

                if(io.time()-startTime > 28.seconds) {
                    if(!zero) zeroTime = io.time()
                    zero = true
                    robot.logic.fsm.curState = SpindexerLogic.State.ZERO
                }

                if(!newRunMotif && runMotif) robot.idleLimelight()

                runMotif = newRunMotif

                if (runMotif) {
                    robot.aim.autoAim.turretTargeting.goalPosition = Vector2d(0.0, 2.3)
                    val id = robot.pollMotif()
                    if (id != null) {
                        detectedMotifId = id
                        robot.applyDetectedMotif(id)
                        MotifPersistence.saveMotif(id, storageDir())
                        motifDetected = true
                    }
                } else {
                    robot.aim.autoAim.turretTargeting.goalPosition = goal
                }

                val motifDisplay = detectedMotifId?.let { "ID $it -> ${robot.logic.motif}" } ?: "Scanning..."
                telemetry.addData("Motif", motifDisplay)
                telemetry.addData("State", robot.logic.currentState.name)
                telemetry.update()
                robot.zero = zero
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
        var flywheelPreSpun = false

        // Pre-spin flywheel a few samples before the shoot marker so it's at speed when we arrive
        val PRESPIN_LEAD_SAMPLES = 20

        while (!mpc.isTrajectoryComplete(traj)) {
            if(zero) {
                spindexerLogic.fsm.curState = SpindexerLogic.State.ZERO
            }
            val sampleI = mpc.latestSampleI
            // Toggle intake based on sample index
            val shouldIntake = intakeRanges.any { sampleI in it }
            when {
                shouldIntake -> {
                    println("TrajoptAuto: intaking")
                    if (spindexerLogic.currentState!= SpindexerLogic.State.FULL) spindexerLogic.startIntaking()
                    intaking = true
                }
                !shouldIntake && intaking -> {
                    println("TrajoptAuto: stop intaking")
                    spindexerLogic.stopIntaking()
                    intaking = false
                }
            }

            // Pre-spin flywheel before reaching shoot marker
            if (!flywheelPreSpun && nextShootIdx < shootSampleIndices.size
                && sampleI >= shootSampleIndices[nextShootIdx] - PRESPIN_LEAD_SAMPLES) {
                spindexerLogic.shotPower = data.SHOT_POWER
                spindexerLogic.flywheel?.let {
                    it.target = spindexerLogic.shotVelocity ?: (data.SHOT_POWER * sigmacorns.constants.flywheelMotor.freeSpeed)
                    it.hold = false
                }
                flywheelPreSpun = true
                println("TrajoptAuto: Pre-spinning flywheel at sample $sampleI")
            }

            // Shoot when we reach a shoot marker -- blocks until all balls are fired
            if (nextShootIdx < shootSampleIndices.size && sampleI >= shootSampleIndices[nextShootIdx]) {
                println("TrajoptAuto: Reached shoot marker at sample $sampleI")
                if (intaking) {
                    spindexerLogic.stopIntaking()
                    intaking = false
                }
                shootAllBalls(spindexerLogic)
                spindexerLogic.spinupRequested = true
                nextShootIdx++
                flywheelPreSpun = false
            }

            delay(1)
        }

        if(zero) {
            spindexerLogic.fsm.curState = SpindexerLogic.State.ZERO
            return
        }

        if (intaking) spindexerLogic.stopIntaking()

        // Fire any shoot markers not reached during trajectory (e.g. marker at final waypoint)
        while (nextShootIdx < shootSampleIndices.size) {
            println("TrajoptAuto: Shooting remaining marker at trajectory end")
            shootAllBalls(spindexerLogic)
            spindexerLogic.spinupRequested = true
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
            delay(10)
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
