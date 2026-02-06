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
import sigmacorns.control.DriveController
import sigmacorns.control.PollableDispatcher
import sigmacorns.control.ShotPowers
import sigmacorns.control.SpindexerLogic
import sigmacorns.control.Turret
import sigmacorns.control.aim.AutoAimGTSAM
import sigmacorns.io.ContourSelectionMode
import sigmacorns.io.HardwareIO
import sigmacorns.io.MPCClient
import sigmacorns.io.TrajoptEventMarker
import sigmacorns.io.TrajoptLoader
import sigmacorns.io.TrajoptTrajectory
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.test.AutoAimGTSAMTest
import sigmacorns.opmode.test.AutoAimGTSAMTest.Companion.applyRuntimeConfig
import sigmacorns.sim.Balls
import sigmacorns.sim.MecanumState
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.thread
import kotlin.concurrent.withLock
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.system.measureNanoTime
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

@Autonomous(name = "Trajopt Auto", group = "Auto")
class TrajoptAuto : SigmaOpMode() {

    companion object {
        // Intake segments: trajectory name -> list of (startWaypointIndex, endWaypointIndex) pairs
        val INTAKE_SEGMENTS: Map<String, List<Pair<Int, Int>>> = mapOf(
            "intake_1" to listOf(1 to 2),
            "intake_2" to listOf(1 to 2),
        )
        val SHOT_POWER = ShotPowers.longShotPower
        val PROJECT_FILE_NAME = "base"
        val ROOT = "intake_1"
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
        val driveController = DriveController()
        turret = Turret(turretRange,io)

        // Preload spindexer with 3 balls
//        spindexerlogic.spindexerstate[0] = balls.purple
//        spindexerlogic.spindexerstate[1] = balls.green
//        spindexerlogic.spindexerstate[2] = balls.green


        // Set initial position from first trajectory
        trajectories.firstOrNull()?.getInitialSample()?.let { io.setPosition(it.pos) }

        val blue = false
        goalPosition = Vector2d(if(blue)-1.480126 else 1.480126, 1.598982)
        autoAim = AutoAimGTSAM(
            landmarkPositions = mapOf(),
            goalPosition,
            initialPose = io.position(), // Use current pose as initial
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

            // Shared state for MPC thread communication
            val mpcLock = ReentrantLock()
            var latestU = doubleArrayOf(0.0, 0.0, 0.0)
            var sharedMecanumState: MecanumState? = null
            var sharedVoltage = 12.0
            var sharedTime: Duration = 0.seconds
            val mpcRunning = AtomicBoolean(true)

            waitForStart()

            // Start MPC thread
            val mpcThread = thread(name = "MPC-Thread") {
                while (mpcRunning.get()) {
                    val (mecanumState, voltage, time) = mpcLock.withLock {
                        Triple(sharedMecanumState, sharedVoltage, sharedTime)
                    }

                    if (mecanumState != null) {
                        val n = measureNanoTime {
                            val u = mpc.getU(time)
                            mpcLock.withLock { latestU = u }
                            mpc.update(mecanumState, voltage, time)
                        }
                        println("MPC thread update took ${n.toDouble() / 1_000_000} ms")
                    }

                    Thread.sleep(1)
                }
            }

            val schedule = CoroutineScope(dispatcher).launch {
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

                val u: List<Double>
                mpcLock.withLock {
                    sharedMecanumState = state.mecanumState
                    sharedVoltage = 12.0
                    sharedTime = t
                    u = latestU.copyOf().map { it * 12.0 / io.voltage() }
                }

                driveController.drive(Pose2d(u[0], u[1], u[2]), io)

                val dt = t - lastTime
                spindexerLogic.update(io.spindexerPosition(), dt, 12.0 / io.voltage())
                lastTime = t

                autoAim.update(io.position(), turret.pos, null)
                processTurret(dt)

                io.update()
            }

            // Stop MPC thread
            mpcRunning.set(false)
            mpcThread.join(50)
            if (mpcThread.isAlive) mpcThread.interrupt()
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
                    spindexerLogic.spindexerState[0] = Balls.Purple
                    spindexerLogic.spindexerState[1] = Balls.Purple
                    spindexerLogic.spindexerState[2] = Balls.Purple
                    intaking = false
                }
            }

            // Shoot when we reach a shoot marker — blocks until all balls are fired
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

    /**
     * Find the sample index closest to the given waypoint's time.
     */
    private fun waypointToSampleIndex(traj: TrajoptTrajectory, waypointIdx: Int): Int? {
        val waypointTime = traj.waypointTimes.getOrNull(waypointIdx) ?: return null
        return traj.samples.indexOfFirst { it.timestamp >= waypointTime }
            .takeIf { it >= 0 } ?: traj.samples.lastIndex
    }

    /**
     * Convert an event marker (waypointIndex + percentage between that waypoint
     * and the next) to the corresponding sample index.
     */
    private fun eventMarkerToSampleIndex(traj: TrajoptTrajectory, marker: TrajoptEventMarker): Int {
        val startTime = traj.waypointTimes.getOrNull(marker.waypointIndex)
            ?: return traj.samples.lastIndex
        val endTime = traj.waypointTimes.getOrNull(marker.waypointIndex + 1) ?: startTime
        val targetTime = startTime + (endTime - startTime) * marker.percentage
        return traj.samples.indexOfFirst { it.timestamp >= targetTime }
            .takeIf { it >= 0 } ?: traj.samples.lastIndex
    }

    /**
     * Convert INTAKE_SEGMENTS waypoint-index pairs into sample-index ranges.
     */
    private fun computeIntakeSampleRanges(traj: TrajoptTrajectory): List<IntRange> {
        val segments = INTAKE_SEGMENTS[traj.name] ?: return emptyList()
        return segments.mapNotNull { (startWp, endWp) ->
            val startIdx = waypointToSampleIndex(traj, startWp) ?: return@mapNotNull null
            val endIdx = waypointToSampleIndex(traj, endWp) ?: return@mapNotNull null
            startIdx..endIdx
        }
    }


    private fun processTurret(dt: Duration) {
        // Update robot heading for field-relative aiming
        turret.robotHeading = autoAim.fusedPose.rot
        turret.robotAngularVelocity = io.velocity().rot

        // Calculate target distance using fused pose and goal position
        val pose = autoAim.fusedPose
        var targetDistance = 0.0
        goalPosition?.let { goal ->
            targetDistance = hypot(goal.x - pose.v.x, goal.y - pose.v.y)
        }

        if (autoAim.enabled && autoAim.hasTarget) {
            if (turret.fieldRelativeMode) {
                // Use field-relative target angle from sensor fusion
                autoAim.getTargetFieldAngle()?.let { fieldAngle ->
                    turret.fieldTargetAngle = fieldAngle
                }
            } else {
                autoAim.getTargetTurretAngle()?.let { robotAngle ->
                    turret.targetAngle = robotAngle
                }
            }
            // Limit target distance for flywheel calculations
            targetDistance = targetDistance.coerceIn(0.1, 10.0)
        }

        turret.targetDistance = targetDistance

        // Update turret PID
        turret.update(dt)
    }
}
