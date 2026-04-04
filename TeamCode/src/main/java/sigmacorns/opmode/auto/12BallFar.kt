package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import sigmacorns.Robot
import sigmacorns.State
import sigmacorns.control.trajopt.TrajoptLoader
import sigmacorns.control.trajopt.TrajoptTrajectory
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

@Autonomous(name = "Auto 12 Ball Far", group = "Competition")
class Auto12Far: SigmaOpMode() {
    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir)
            .find { it.nameWithoutExtension == "12ballfar" }
            ?: throw IllegalStateException("12ballfar.json not found in $robotDir")

        val traj1 = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 1")!!
        val traj2 = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 2")!!
        val traj3 = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 3")!!

        val initialSample = traj1.getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        io.setPosition(initialSample.pos)

        val robot = Robot(io, blue = false)
        robot.init(initialSample.pos, apriltagTracking = !SIM)
        robot.aimTurret = true
        robot.aimFlywheel = false

        val state = State(
            0.0, io.position(), Pose2d(), Pose2d(), 0.0, 0.0, 0.seconds,
        )

        waitForStart()

        // Phase 1: Shoot all preloads using aimFlywheel
        robot.aimFlywheel = true
        robot.intakeTransfer.state = IntakeTransfer.State.IDLE

        while (opModeIsActive()) {
            state.update(io)
            robot.update()
            io.update()

            robot.aim.shotRequested = true
            if (robot.beamBreak.ballCount == 0) break
            robot.aimFlywheel = false
        }

        robot.intakeTransfer.state = IntakeTransfer.State.IDLE

        // Phase 2: Run 3 trajectories with intake/shoot cycles
        val trajectories = listOf(traj1, traj2, traj3)

        for (traj in trajectories) {
            if (!opModeIsActive()) break

            runTrajectoryWithIntakeShoot(robot, state, traj,traj==traj3)
        }

        io.driveFL = 0.0
        io.driveBL = 0.0
        io.driveBR = 0.0
        io.driveFR = 0.0
        robot.shooter.flywheelTarget = 0.0
        io.update()
    }

    private fun runTrajectoryWithIntakeShoot(
        robot: Robot,
        state: State,
        traj: TrajoptTrajectory,
        last: Boolean
    ) {
        robot.ltv.loadTrajectory(traj)
        val startTime = io.time()

        while (opModeIsActive()) {
            state.update(io)
            val elapsed = io.time() - startTime
            val elapsedSeconds = elapsed.toDouble(DurationUnit.SECONDS)

            // Drive via LTV
            if (elapsedSeconds <= traj.totalTime) {
                val u = robot.ltv.solve(state.mecanumState, elapsed)
                io.driveFL = u[0]
                io.driveBL = u[1]
                io.driveBR = u[2]
                io.driveFR = u[3]
            } else {
                io.driveFL = 0.0
                io.driveBL = 0.0
                io.driveBR = 0.0
                io.driveFR = 0.0
            }

            // Handle waypoint-based intake and shooting
            // Turn on intake between waypoint 2 and 3
            val inIntakeZone =
                elapsedSeconds >= (traj.waypointTimes.getOrNull(if(last) 0 else 1) ?: Double.MAX_VALUE) &&
                        elapsedSeconds < (traj.waypointTimes.getOrNull(if(last) 1 else 2) ?: Double.MAX_VALUE)

            if (inIntakeZone) {
                robot.intakeCoordinator.startIntake()
            } else if (elapsedSeconds >= (traj.waypointTimes.getOrNull(if(last) 1 else 2) ?: Double.MAX_VALUE) &&
                elapsedSeconds < (traj.waypointTimes.getOrNull(if(last) 2 else 3) ?: Double.MAX_VALUE)
            ) {
                // Shoot at waypoint 3
                robot.aimFlywheel = true
                robot.aim.shotRequested = true
            } else {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                robot.aimFlywheel = false
            }

            robot.update()

            telemetry.addData("elapsed", "%.2f / %.2f s".format(elapsedSeconds, traj.totalTime))
            telemetry.addData("state", robot.intakeTransfer.state)
            telemetry.addData("flywheel", if (robot.shooter.flywheelTarget > 0) "SPINNING" else "OFF")
            telemetry.addData("pos", "(%.3f, %.3f, %.1f°)".format(
                state.driveTrainPosition.v.x,
                state.driveTrainPosition.v.y,
                Math.toDegrees(state.driveTrainPosition.rot),
            ))
            telemetry.update()

            io.update()

            if (SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)

            if (elapsedSeconds > traj.totalTime + 1.0) break
        }
    }
}