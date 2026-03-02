package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.joml.Vector2d
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.PollableDispatcher
import sigmacorns.control.mpc.ContourSelectionMode
import sigmacorns.control.mpc.MPCClient
import sigmacorns.control.mpc.MPCRunner
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.io.HardwareIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.LinearDcMotor
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@Autonomous
class RedAutoPathing: SigmaOpMode() {
    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir).find { it.nameWithoutExtension == "base" }!!

        val intake1 = TrajoptLoader.loadTrajectory(projectFile,"intake_1")!!
        val intake2 = TrajoptLoader.loadTrajectory(projectFile,"intake_2")!!
        val intake3 = TrajoptLoader.loadTrajectory(projectFile,"intake_3")!!

        val dispatcher = PollableDispatcher(io)

        val ll = (io as? HardwareIO)?.limelight
        ll!!.pipelineSwitch(1)

        io.setPosition(intake1.getInitialSample()!!.pos)

        MPCClient(
            drivetrainParameters.let {
                val p = it.copy()
                p.motor = LinearDcMotor(it.motor.freeSpeed, it.motor.stallTorque*0.8)
                p
            },
            Network.LIMELIGHT,
            contourSelectionMode = ContourSelectionMode.POSITION,
            preIntegrate = 30.milliseconds,
            sampleLookahead = 0
        ).use { mpc ->
            val state = State(
                0.0,
                io.position(),
                Pose2d(),
                Pose2d(Vector2d(), 0.0),
                0.0,
                0.0,
                0.seconds
            )

            val runner = MPCRunner(mpc)

            waitForStart()

            runner.start()

            val schedule = CoroutineScope(dispatcher).launch {
                mpc.runTrajectory(intake1,1.milliseconds)()
                delay(1000)
                mpc.runTrajectory(intake2,1.milliseconds)()
                delay(1000)
                mpc.runTrajectory(intake3,1.milliseconds)()
                delay(3000)
            }

            while (opModeIsActive() && !schedule.isCompleted) {
                val t = io.time()
                state.update(io)

                dispatcher.update()

                runner.updateState(state.mecanumState, kotlin.math.min(12.0, io.voltage()), t)
                runner.driveWithMPC(io, io.voltage())

                io.update()
            }

            runner.stop()
        }
    }
}
