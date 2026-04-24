package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import sigmacorns.Robot
import sigmacorns.control.trajopt.TrajoptLoader
import sigmacorns.control.trajopt.TrajoptTrajectory
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

@Autonomous(name = "Auto 15 Close Red", group = "Competition")
class Auto15CloseRed : SigmaOpMode() {
    companion object {
        private val MAX_TRANSFER_DURATION = 600.milliseconds
        private val NORMAL_FORCE_TRANSFER_TIMEOUT = 550.milliseconds
        private val FORCE_TRANSFER_TIMEOUT = 600.milliseconds
        private val GO_TO_GATE_HOLD_DURATION = 900.milliseconds
        private val GO_TO_GATE_FULL_BEAMBREAK_HOLD = 200.milliseconds
        private val WAYPOINT_STOP_HOLD = 450.milliseconds
        private val SHOT_AIM_SETTLE_HOLD = 200.milliseconds
        private val FIRST_SHOT_SPINUP_HOLD = 450.milliseconds
    }

    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir)
            .find { it.nameWithoutExtension == "15closeredbetter" }
            ?: TrajoptLoader.findProjectFiles(robotDir)
                .find { it.nameWithoutExtension == "15closered" }
            ?: throw IllegalStateException("15closered.json (or 15closeredbetter.json) not found in $robotDir")

        fun requireTrajectory(name: String) =
            TrajoptLoader.loadTrajectory(projectFile, name)
                ?: throw IllegalStateException("'$name' not found in ${projectFile.name}")

        val trajectories = buildList {
            add(requireTrajectory("ShootPreload"))
            add(requireTrajectory("ShootRow1"))
            add(requireTrajectory("ShootRow2"))
            TrajoptLoader.loadTrajectory(projectFile, "GoToGate1")?.let(::add)
            add(requireTrajectory("ShootGate1"))
        }

        val initialSample = trajectories.first().getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        io.setPosition(initialSample.pos)

        val robot = Robot(io, blue = false)
        robot.init(initialSample.pos, apriltagTracking = false)
        robot.aimTurret = true
        robot.aimFlywheel = false

        waitForStart()

        val job = robot.scope.launch {
            val firstPose = trajectories.first().getInitialSample()!!.pos
            val spinupStartedAt = robot.io.time()
            robot.aimFlywheel = true
            while (robot.io.time() - spinupStartedAt < FIRST_SHOT_SPINUP_HOLD) {
                robot.ltv.holdPos(robot.io, firstPose)
                yield()
            }

            trajectories.forEachIndexed { i, traj ->
                runTrajectoryWithIntakeShoot(robot, traj, enableIntake = i != 0)

                val holdPose = traj.getFinalSample()!!.pos
                val hold = robot.scope.launch { holdPosition(robot, holdPose) }
                try {
                    waitDuration(robot, WAYPOINT_STOP_HOLD)
                    if (traj.name.startsWith("GoToGate", ignoreCase = true)) {
                        holdAndIntakeAtGate(robot, GO_TO_GATE_HOLD_DURATION)
                        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                    } else {
                        println(robot.aim.autoAim.fusedPose)
                        shootUntilEmpty(robot)
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

        io.driveFL = 0.0
        io.driveBL = 0.0
        io.driveBR = 0.0
        io.driveFR = 0.0
        robot.shooter.flywheelTarget = 0.0
        io.update()
    }

    private suspend fun waitDuration(robot: Robot, duration: Duration) {
        val startedAt = robot.io.time()
        while (robot.io.time() - startedAt < duration) {
            yield()
        }
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
            allowForcedTransfer = false,
        )
    }

    private suspend fun holdAndIntakeAtGate(robot: Robot, duration: Duration) {
        robot.aimFlywheel = false
        robot.aim.shotRequested = false
        robot.intakeCoordinator.overrideShot = false
        robot.intakeTransfer.state = IntakeTransfer.State.INTAKING

        val startedAt = robot.io.time()
        var allBeamBreaksSince: Duration? = null

        while (true) {
            val now = robot.io.time()
            val allBeamBreaksTriggered = robot.beamBreak.isFull ||
                (robot.io.beamBreak1() && robot.io.beamBreak2() && robot.io.beamBreak3())
            if (allBeamBreaksTriggered) {
                if (allBeamBreaksSince == null) {
                    allBeamBreaksSince = now
                } else if (now - allBeamBreaksSince >= GO_TO_GATE_FULL_BEAMBREAK_HOLD) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                    break
                }
            } else {
                allBeamBreaksSince = null
            }

            if (now - startedAt >= duration) {
                break
            }

            yield()
        }

        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        robot.aim.shotRequested = false
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

        val settleStartedAt = robot.io.time()
        while (robot.io.time() - settleStartedAt < SHOT_AIM_SETTLE_HOLD) {
            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            robot.aim.shotRequested = false
            yield()
        }

        val startedAt = robot.io.time()
        while (transferAt == null) {
            robot.aim.shotRequested = true
            val transferring = robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING

            if (transferring) {
                transferAt = robot.io.time()
                robot.aim.shotRequested = false
            } else if (robot.io.time() - startedAt >= forceTimeout) {
                transferAt = robot.io.time()
                robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                robot.intakeCoordinator.overrideShot = true
                robot.aim.shotRequested = false
            } else {
                yield()
            }
        }

        var sawBeamBreakDuringTransfer = robot.beamBreak.slots.any { it } ||
            robot.io.beamBreak1() || robot.io.beamBreak2() || robot.io.beamBreak3()
        while (true) {
            val elapsed = robot.io.time() - transferAt
            val beamBreak1 = robot.io.beamBreak1()
            val beamBreak2 = robot.io.beamBreak2()
            val beamBreak3 = robot.io.beamBreak3()
            val anyBeamBreakRaw = beamBreak1 || beamBreak2 || beamBreak3
            if (anyBeamBreakRaw) {
                sawBeamBreakDuringTransfer = true
            }
            val beamBreaksEmptyRaw = !anyBeamBreakRaw

            if (sawBeamBreakDuringTransfer && beamBreaksEmptyRaw) {
                break
            }
            if (elapsed >= duration) {
                break
            }
            yield()
        }

        robot.intakeCoordinator.overrideShot = false
        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        robot.aimFlywheel = false
        robot.aim.shotRequested = false
    }

    private suspend fun runTrajectoryWithIntakeShoot(
        robot: Robot,
        traj: TrajoptTrajectory,
        enableIntake: Boolean,
    ) {
        robot.ltv.loadTrajectory(traj)
        val isGoToGate = traj.name.startsWith("GoToGate", ignoreCase = true)
        robot.aimFlywheel = !isGoToGate
        robot.aim.shotRequested = !isGoToGate
        robot.intakeCoordinator.overrideShot = false

        val trajStartTime = io.time()
        val path = robot.scope.launch { robot.ltv.runLoadedTrajectory(robot.io) }

        if (isGoToGate) {
            var intakeEnabled = true
            while (!path.isCompleted) {
                val full = robot.beamBreak.isFull ||
                    (robot.io.beamBreak1() && robot.io.beamBreak2() && robot.io.beamBreak3())
                if (intakeEnabled && full) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                    intakeEnabled = false
                } else if (intakeEnabled) {
                    robot.intakeTransfer.state = IntakeTransfer.State.INTAKING
                }
                yield()
            }
            path.join()
            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            return
        }

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

        path.join()
    }
}
