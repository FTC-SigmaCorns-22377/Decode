package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.DriveController
import sigmacorns.control.PollableDispatcher
import sigmacorns.io.ContourSelectionMode
import sigmacorns.io.HardwareIO
import sigmacorns.io.MPCClient
import sigmacorns.io.TrajoptLoader
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import java.io.File
import kotlin.math.hypot
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
            drivetrainParameters,
            Network.LIMELIGHT,
            contourSelectionMode = ContourSelectionMode.POSITION,
            preIntegrate = 40.milliseconds,
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

            waitForStart()

            val driveController = DriveController()

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

                mpc.update(
                    state.mecanumState,
                    io.voltage(),
                    t
                )
                val u = mpc.getU(t)

                // u is now [drive, strafe, turn]
                val drive = u[0]
                val strafe = u[1]
                val turn = u[2]

                // Convert drive, strafe, turn to motor powers using DriveController
                val robotPower = Pose2d(drive, strafe, turn)
                driveController.drive(robotPower, io)

                io.update()
            }
        }
    }
}