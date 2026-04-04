package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import sigmacorns.Robot
import sigmacorns.State
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.ltv.LTVClient
import sigmacorns.control.trajopt.TrajoptLoader
import sigmacorns.io.JoltSimIO
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.io.BallColor
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

/**
 * Mirrored 12-ball autonomous (red side).
 */
@Autonomous(name = "Auto 12 Ball Mirrored", group = "Competition")
class Auto12BallMirrored : SigmaOpMode() {
    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir)
            .find { it.nameWithoutExtension == "test12ball_mirrored" }
            ?: throw IllegalStateException("test12ball_mirrored.json not found in $robotDir")

        val traj = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 1")
            ?: throw IllegalStateException("'Trajectory 1' not found in test12ball_mirrored")

        val initialSample = traj.getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        // Preload 3 balls in sim
        val joltSim = io as? JoltSimIO
        repeat(3) { joltSim?.heldBalls?.add(BallColor.GREEN) }

        val robot = Robot(io, blue = false)
        robot.init(initialSample.pos, apriltagTracking = !SIM)
        robot.aimTurret = true

        if (SIM) {
            robot.aim.goalPosition = org.joml.Vector2d(-robot.aim.goalPosition.x, robot.aim.goalPosition.y)
        }

        val ltv = LTVClient(drivetrainParameters).also { it.loadTrajectory(traj) }

        val shootTimes = traj.eventMarkers.map { it.timestamp }.sorted()
        val shootDuration = 1.5
        val spinUpLeadTime = 1.0

        ltv.use {
            val state = State(
                0.0, io.position(), Pose2d(), Pose2d(), 0.0, 0.0, 0.seconds,
            )

            waitForStart()

            // Phase 1: Spin up flywheel then shoot preloads via TRANSFERRING
            robot.aimFlywheel = false
            robot.shooter.flywheelTarget = Auto12Ball.FLYWHEEL_TARGET
            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            val preShootStart = io.time()
            while (opModeIsActive()) {
                state.update(io)
                val t = (io.time() - preShootStart).toDouble(DurationUnit.SECONDS)

                if (t >= 1.0) {
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                }

                robot.update()
                io.update()
                if (SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)
                if (t >= 1.0 + shootDuration) break
            }

            // Phase 2: Follow trajectory with intake/shoot cycles
            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            robot.shooter.flywheelTarget = 0.0

            io.setPosition(initialSample.pos)
            val startTime = io.time()

            while (opModeIsActive()) {
                state.update(io)
                val elapsed = io.time() - startTime
                val elapsedSeconds = elapsed.toDouble(DurationUnit.SECONDS)

                if (elapsedSeconds <= traj.totalTime) {
                    val u = ltv.solve(state.mecanumState, elapsed)
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

                val isShooting = shootTimes.any { shootTime ->
                    elapsedSeconds >= shootTime && elapsedSeconds <= shootTime + shootDuration
                }

                val shouldSpinUp = isShooting || shootTimes.any { shootTime ->
                    elapsedSeconds >= shootTime - spinUpLeadTime && elapsedSeconds < shootTime
                }
                robot.aimFlywheel = false
                robot.shooter.flywheelTarget = if (shouldSpinUp) Auto12Ball.FLYWHEEL_TARGET else 0.0

                when {
                    isShooting -> {
                        robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                    }
                    traj.isIntakeZone(elapsedSeconds) -> {
                        robot.intakeCoordinator.startIntake()
                    }
                    else -> {
                        if (robot.intakeTransfer.state == IntakeTransfer.State.INTAKING ||
                            robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING) {
                            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                        }
                    }
                }

                robot.update()

                telemetry.addData("elapsed", "%.2f / %.2f s".format(elapsedSeconds, traj.totalTime))
                telemetry.addData("state", robot.intakeTransfer.state)
                telemetry.addData("flywheel", if (robot.shooter.flywheelTarget > 0) "SPINNING" else "OFF")
                telemetry.addData("held", joltSim?.heldBalls?.size ?: "N/A")
                telemetry.addData("pos", "(%.3f, %.3f, %.1f°)".format(
                    state.driveTrainPosition.v.x,
                    state.driveTrainPosition.v.y,
                    Math.toDegrees(state.driveTrainPosition.rot),
                ))
                telemetry.update()

                io.update()

                if (SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)

                if (elapsedSeconds > traj.totalTime + 2.0) break
            }

            io.driveFL = 0.0; io.driveBL = 0.0
            io.driveBR = 0.0; io.driveFR = 0.0
            robot.shooter.flywheelTarget = 0.0
            io.update()
        }

        robot.close()
    }
}
