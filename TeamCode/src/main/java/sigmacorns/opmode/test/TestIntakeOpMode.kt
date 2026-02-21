package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.joml.Vector2d
import sigmacorns.constants.drivetrainCenter
import sigmacorns.control.Robot
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.control.mpc.TrajoptSample
import sigmacorns.control.mpc.TrajoptTrajectory
import sigmacorns.io.rotate
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode

@TeleOp(name = "Test Intake", group = "test")
class TestIntakeOpMode : SigmaOpMode() {

    companion object {
        const val TRAJECTORY_NAME = "intake_1"
        const val PROJECT_FILE_NAME = "base"
    }

    override fun runOpMode() {
        val robot = Robot(io, blue = false)

        val projectFile = TrajoptLoader.findProjectFiles(TrajoptLoader.robotTrajoptDir()).find {
            it.nameWithoutExtension == PROJECT_FILE_NAME
        } ?: throw IllegalStateException("Project file '$PROJECT_FILE_NAME' not found")

        val traj = TrajoptLoader.loadTrajectory(projectFile, TRAJECTORY_NAME)
            ?: throw IllegalStateException("Trajectory '$TRAJECTORY_NAME' not found")

        val returnTraj = traj.reversed()

        val initPos = traj.getInitialSample()!!.pos.let {
            val v = Vector2d()
            it.v.sub(drivetrainCenter.rotate(it.rot), v)
            Pose2d(v, it.rot)
        }

        robot.use {
            robot.init(initPos, false)
            robot.startMPC()

            waitForStart()

            val schedule = robot.scope.launch {
                robot.logic.startIntaking()

                // Drive forward trajectory
                robot.runner!!.setTarget(traj)
                while (!robot.runner!!.isTrajectoryComplete(traj)) {
                    delay(1)
                }

                // Drive back to start
                robot.runner!!.setTarget(returnTraj)
                while (!robot.runner!!.isTrajectoryComplete(returnTraj)) {
                    delay(1)
                }

                robot.logic.stopIntaking()
            }

            while (opModeIsActive() && !schedule.isCompleted) {
                robot.update()
                io.update()
            }
        }
    }

    private fun TrajoptTrajectory.reversed(): TrajoptTrajectory {
        val reversedSamples = samples.reversed().map { sample ->
            TrajoptSample(
                timestamp = totalTime - sample.timestamp,
                vx = -sample.vx,
                vy = -sample.vy,
                omega = -sample.omega,
                x = sample.x,
                y = sample.y,
                heading = sample.heading,
            )
        }
        return TrajoptTrajectory(
            name = "$name-return",
            samples = reversedSamples,
            totalTime = totalTime,
        )
    }
}
