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
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "21 Ball Red Far", group = "Competition")
class `21BallMixedRed`: SigmaOpMode() {
    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir)
            .find {it.nameWithoutExtension == "21RedFar"}
            ?: throw IllegalStateException("21RedFar.json not found in $robotDir")

        val trajectories = listOf("Shoot+intake_1","intake2", "intake_3", "descore_1", "descore_2", "descore_3", "leave")
            .map {TrajoptLoader.loadTrajectory(projectFile,it)!!}

        val initialSample = trajectories.first().getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        io.setPosition(initialSample.pos)

        val robot = Robot(io, blue = false)
        robot.init(initialSample.pos, apriltagTracking = false)
        robot.aimTurret = true
        robot.aimFlywheel = true

        waitForStart()

        val job = robot.scope.launch {
            //shoot preloads
            shootUntilEmpty(robot)

            trajectories.forEachIndexed { i, traj ->
                val last = i == trajectories.lastIndex
                runTrajectoryWithIntakeShoot(robot,traj, last)

                //once the pathing is done, hold your robot position with LTV while
                //the shot sequence runs, then cancal the hold before the next
                //trajectory takes over drive

                val holdPose = traj.getFinalSample()!!.pos
                val hold = robot.scope.launch { holdPosition(robot, holdPose) }
                try {
                    shootUntilEmpty(robot)
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
    private suspend fun holdPosition(robot: Robot, pose: Pose2d) {
        try {
            while (true) {
                robot.ltv.holdPos(robot.io, pose)
                yield()
            }
        } catch (_: CancellationException) { }
    }

    private suspend fun shootUntilEmpty(robot: Robot) {
        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        robot.aimFlywheel = true

        val startedAt = robot.io.time()
        var shootAt: Duration? = null

        while (shootAt == null || robot.io.time() - shootAt < 2.seconds) {
            robot.aim.shotRequested = true
            val transferring = robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING

            if(transferring && shootAt == null) shootAt = robot.io.time()

            // Force-start transferring if aim hasn't kicked it off within 1s.
            if (!robot.intakeCoordinator.overrideShot && !transferring &&
                robot.io.time() - startedAt >= 1.seconds) {
                robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                robot.intakeCoordinator.overrideShot = true
            }

            yield()
        }

        robot.intakeCoordinator.overrideShot = false
        robot.aimFlywheel = false
        robot.aim.shotRequested = false
    }

    private suspend fun runTrajectoryWithIntakeShoot(
        robot: Robot,
        traj: TrajoptTrajectory,
        last: Boolean,
    ) {
        // Load synchronously before launching the follower so the orchestrator
        // sees the new samples (and reset progressIdx) on its very first
        // estimateProgress call. Otherwise the path coroutine would not have
        // run loadTrajectory yet and time() would read the previous traj.
        robot.ltv.loadTrajectory(traj)

        val intakeStart = traj.samples[(traj.samples.size*0.2).toInt()].timestamp
        val intakeEnd = traj.samples[(traj.samples.size*0.8).toInt()].timestamp

        val trajStartTime = io.time()
        val path = robot.scope.launch { robot.ltv.runLoadedTrajectory(robot.io) }

        fun elapsed() = (io.time() - trajStartTime).inWholeMilliseconds / 1000.0

        println(traj.waypointTimes)
        println("starting traj at time ${io.time()}")
        println("intakeStart=$intakeStart, intakeEnd=$intakeEnd")
        while (!path.isCompleted && elapsed() < intakeStart) {
            println("running traj at time ${io.time()}")
            yield()
        }
        robot.intakeCoordinator.startIntake()

        println("starting intake at ${io.time()}")

        while (!path.isCompleted && elapsed() < intakeEnd) yield()
        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        robot.aimFlywheel = true

        println("done with intake at ${io.time()}")
        path.join()
    }

}


