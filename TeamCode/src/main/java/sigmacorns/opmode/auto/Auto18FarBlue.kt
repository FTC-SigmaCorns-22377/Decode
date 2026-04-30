package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import sigmacorns.Robot
import sigmacorns.constants.antiWheelieFilter
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.ltv.LTVClient
import sigmacorns.control.trajopt.TrajoptLoader
import sigmacorns.control.trajopt.TrajoptTrajectory
import sigmacorns.math.Pose2d
import sigmacorns.opmode.PosePersistence
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import kotlin.math.max
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

@Autonomous(name = "Auto 18 Far Blue", group = "Competition")
class Auto18FarBlue : SigmaOpMode() {
    companion object {
        private val MAX_TRANSFER_DURATION = 600.milliseconds
        private val NORMAL_FORCE_TRANSFER_TIMEOUT = 550.milliseconds
        private val FORCE_TRANSFER_TIMEOUT = 600.milliseconds
        private val WAYPOINT_STOP_HOLD = 300.milliseconds
        private val SHOOT_EMPTY_DEBOUNCE = 120.milliseconds
        private const val DISABLE_HOLD_DURING_SHOOT = false
    }

    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir)
            .find { it.nameWithoutExtension == "18farredhp_mirrored" }
            ?: throw IllegalStateException("18farredhp_mirrored.json not found in $robotDir")

        fun requireTrajectory(name: String) =
            TrajoptLoader.loadTrajectory(projectFile, name)
                ?: throw IllegalStateException("'$name' not found in ${projectFile.name}")

        val trajectories = buildList {
            add(requireTrajectory("ShootRow1"))
            add(requireTrajectory("ShootHP1"))
            add(requireTrajectory("ShootHP2"))
            add(requireTrajectory("ShootHP3"))
            add(requireTrajectory("ShootHP4"))
            add(requireTrajectory("Leave"))
        }

        val initialSample = trajectories.first().getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        io.setPosition(initialSample.pos)

        val robot = Robot(io, blue = true, useNativeAim = true)
        robot.init(initialSample.pos, apriltagTracking = false)
        robot.aimTurret = true
        robot.aimFlywheel = false

        val loadedTrajectories = trajectories.map { traj ->
            val solveStart = robot.io.time()
            val handle = robot.ltv.loadTrajectory(traj)
            val solveTime = robot.io.time() - solveStart
            println("Solving path ${traj.name} took: ${solveTime.inWholeMilliseconds} ms")
            traj to handle
        }

        telemetry.addData("LTV", "Loaded ${loadedTrajectories.size} trajectory handles")
        telemetry.update()

        waitForStart()

        val job = robot.scope.launch {
            robot.aimFlywheel = true

            shootUntilEmpty(robot)

            loadedTrajectories.forEachIndexed { i, (traj, handle) ->
                val isPreload = i == 0
                runTrajectoryWithIntakeShoot(robot, traj, handle, enableIntake = !isPreload)

                val isLeave = traj.name.equals("Leave", ignoreCase = true)
                if (isLeave) {
                    return@forEachIndexed
                }

                val holdPose = traj.getFinalSample()!!.pos
                var hold = robot.scope.launch { holdPosition(robot, holdPose) }
                try {
                    val settleStart = robot.io.time()
                    while (robot.io.time() - settleStart < WAYPOINT_STOP_HOLD) {
                        val hasBalls = robot.beamBreak.slots.any { it } ||
                            robot.io.beamBreak1() || robot.io.beamBreak2() || robot.io.beamBreak3()
                        if (!hasBalls) break
                        yield()
                    }

                    if (DISABLE_HOLD_DURING_SHOOT) {
                        hold.cancel()
                        io.driveFL = 0.0
                        io.driveBL = 0.0
                        io.driveBR = 0.0
                        io.driveFR = 0.0
                    }
                    println(robot.aim.autoAim.fusedPose)
                    shootUntilEmpty(robot)
                    if (DISABLE_HOLD_DURING_SHOOT) {
                        hold = robot.scope.launch { holdPosition(robot, holdPose) }
                    }
                } finally {
                    hold.cancel()
                }
            }
        }

        ioLoop { _, _ ->
            robot.update()
            job.isCompleted
        }

        val endTime = io.time()
        ioLoop { _, _ ->
            robot.update()
            io.time() - endTime > 400.milliseconds
        }

        PosePersistence.save(storageDir(), io.position())

        io.driveFL = 0.0
        io.driveBL = 0.0
        io.driveBR = 0.0
        io.driveFR = 0.0
        robot.shooter.flywheelTarget = 0.0
        io.update()
    }

    private suspend fun holdPosition(robot: Robot, pose: Pose2d) {
        try {
            while (true) {
                robot.ltv.holdPos(robot.io, pose)
                yield()
            }
        } catch (_: CancellationException) {
        }
    }

    private suspend fun shootUntilEmpty(robot: Robot) {
        runShotSequence(
            robot = robot,
            duration = MAX_TRANSFER_DURATION,
            allowForcedTransfer = true,
        )
    }

    private suspend fun runShotSequence(
        robot: Robot,
        duration: Duration,
        allowForcedTransfer: Boolean,
    ) {
        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        robot.aimFlywheel = true
        robot.aim.shotRequested = false
        robot.intakeCoordinator.overrideShot = false

        var transferAt: Duration? = null
        val forceTimeout = if (allowForcedTransfer) FORCE_TRANSFER_TIMEOUT else NORMAL_FORCE_TRANSFER_TIMEOUT

        val startedAt = robot.io.time()
        while (transferAt == null) {
            robot.aim.shotRequested = true
            val transferring = robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING

            if (transferring) {
                transferAt = robot.io.time()
            } else if (robot.io.time() - startedAt >= forceTimeout) {
                transferAt = robot.io.time()
                robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                robot.intakeCoordinator.overrideShot = true
            } else {
                yield()
            }
        }

        var sawBeamBreakDuringTransfer = robot.beamBreak.slots.any { it } ||
            robot.io.beamBreak1() || robot.io.beamBreak2() || robot.io.beamBreak3()
        var emptySince: Duration? = null
        while (true) {
            val now = robot.io.time()
            robot.aim.shotRequested = true
            val elapsed = now - transferAt
            val beamBreak1 = robot.io.beamBreak1()
            val beamBreak2 = robot.io.beamBreak2()
            val beamBreak3 = robot.io.beamBreak3()
            val anyBeamBreakRaw = beamBreak1 || beamBreak2 || beamBreak3
            if (anyBeamBreakRaw) {
                sawBeamBreakDuringTransfer = true
                emptySince = null
            }
            val beamBreaksEmptyRaw = !anyBeamBreakRaw
            if (beamBreaksEmptyRaw && emptySince == null) {
                emptySince = now
            }
            val beamBreaksEmptyDebounced = emptySince != null && (now - emptySince >= SHOOT_EMPTY_DEBOUNCE)

            if (sawBeamBreakDuringTransfer && beamBreaksEmptyDebounced) {
                break
            }
            if (!sawBeamBreakDuringTransfer && beamBreaksEmptyDebounced && elapsed > 150.milliseconds) {
                break
            }
            if (elapsed >= duration) {
                break
            }
            yield()
        }

        robot.intakeCoordinator.overrideShot = false
        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        robot.aim.shotRequested = false
    }

    private suspend fun runTrajectoryWithIntakeShoot(
        robot: Robot,
        traj: TrajoptTrajectory,
        handle: Int,
        enableIntake: Boolean,
    ) {
        robot.ltv.setTrajectory(handle)

        robot.aimFlywheel = true
        robot.aim.shotRequested = false
        robot.intakeCoordinator.overrideShot = false

        val trajStartTime = io.time()
        val path = robot.scope.launch { robot.ltv.runLoadedTrajectory(robot.io) }

        if (enableIntake) {
            fun elapsed() = (io.time() - trajStartTime).inWholeMilliseconds / 1000.0
            val intakeStart = traj.samples[(traj.samples.size * 0.2).toInt()].timestamp
            val intakeEnd = traj.samples[(traj.samples.size * 0.8).toInt()].timestamp

            while (!path.isCompleted && elapsed() < intakeStart) yield()
            robot.intakeCoordinator.startIntake()

            while (!path.isCompleted && elapsed() < intakeEnd) yield()
        }

        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        robot.aimFlywheel = true
        robot.aim.shotRequested = false

        path.join()
    }
}
