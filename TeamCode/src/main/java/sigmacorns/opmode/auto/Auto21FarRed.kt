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
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "Auto 21 Far Red", group = "Competition")
class Auto21FarRed : SigmaOpMode() {
    companion object {
        private val MAX_TRANSFER_DURATION = 600.milliseconds
        private val NORMAL_FORCE_TRANSFER_TIMEOUT = 550.milliseconds
        private val FORCE_TRANSFER_TIMEOUT = 600.milliseconds
        private val GO_TO_GATE_HOLD_DURATION = 2.5.seconds
        private val GO_TO_GATE_FULL_BEAMBREAK_HOLD = 200.milliseconds
        private val WAYPOINT_STOP_HOLD = 300.milliseconds
        private val FIRST_SHOT_SPINUP_HOLD = 450.milliseconds
        private val SHOOT_EMPTY_DEBOUNCE = 120.milliseconds
        private const val DISABLE_HOLD_DURING_SHOOT = false
    }

    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir)
            .find { it.nameWithoutExtension == "21-far-red" }
            ?: throw IllegalStateException("21-far-red.json not found in $robotDir")

        fun requireTrajectory(name: String) =
            TrajoptLoader.loadTrajectory(projectFile, name)
                ?: throw IllegalStateException("'$name' not found in ${projectFile.name}")

        val trajectories = buildList {
            add(requireTrajectory("shoot+intake_1"))
            add(requireTrajectory("intake_2"))
            add(requireTrajectory("GoToGate_1"))
            add(requireTrajectory("ShootGate_1"))
            add(requireTrajectory("GoToGate_2"))
            add(requireTrajectory("ShootGate_2"))
            add(requireTrajectory("GoToGate_3"))
            add(requireTrajectory("ShootGate_3"))
            add(requireTrajectory("GoToGate_4"))
            add(requireTrajectory("ShootGate_4"))
            add(requireTrajectory("Leave"))
        }

        val initialSample = trajectories.first().getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        io.setPosition(initialSample.pos)

        val robot = Robot(io, blue = false, useNativeAim = true)
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

            loadedTrajectories.forEachIndexed { _, (traj, handle) ->
                val isPreload = false
                runTrajectoryWithIntakeShoot(robot, traj, handle, enableIntake = !isPreload)

                val holdPose = traj.getFinalSample()!!.pos
                var hold = robot.scope.launch { holdPosition(robot, holdPose) }
                try {
                    if (traj.name.startsWith("GoToGate", ignoreCase = true)) {
                        holdAndIntakeAtGate(robot, GO_TO_GATE_HOLD_DURATION)
                        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                    } else {
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

    private suspend fun holdAndIntakeAtGate(robot: Robot, duration: Duration) {
        robot.aimFlywheel = false
        robot.aim.shotRequested = false
        robot.intakeCoordinator.overrideShot = false
        robot.intakeTransfer.state = IntakeTransfer.State.INTAKING

        val startedAt = robot.io.time()
        var allBeamBreaksSince: Duration? = null

        while (true) {
            val now = robot.io.time()
            val allBeamBreaksTriggered = robot.beamBreak.isFull
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

        val isGoToGate = traj.name.startsWith("GoToGate", ignoreCase = true)
        robot.aimFlywheel = !isGoToGate
        robot.aim.shotRequested = false
        robot.intakeCoordinator.overrideShot = false

        val trajStartTime = io.time()
        val path = robot.scope.launch { robot.ltv.runLoadedTrajectory(robot.io) }

        if (isGoToGate) {
            var intakeEnabled = true
            while (!path.isCompleted) {
                val full = robot.beamBreak.isFull
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
        robot.aim.shotRequested = false

        path.join()
    }
}
