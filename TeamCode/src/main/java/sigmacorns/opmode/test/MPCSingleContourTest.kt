package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.Vector2d
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.Contour
import sigmacorns.io.MPCClient
import sigmacorns.io.RerunLogging
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.io.SigmaIO
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumState
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@TeleOp(group = "test")
class MPCSingleContourTest(): SigmaOpMode() {
    override fun runOpMode() {
        val contours = listOf(
            Contour(Pose2d(0.0,0.0,0.0), Pose2d(0.5,0.0,0.0),0.0,1.0, Vector2d(0.0,0.0))
        )

        val mpc = MPCClient(
            drivetrainParameters,
            if (LIMELIGHT_CONNECTED) Network.LIMELIGHT else Network.SIM_MPC
        )

        RerunLogging.save("MPCSingleContourTest","/sdcard/FIRST/MPCSingleContourTest.rrd").use { rr ->
            mpc.setTargetContours(contours)

            val state = State(
                0.0,
                contours[0].pos,
                contours[0].vel,
                Pose2d(Vector2d(), 0.0),
                0.0,
                0.0,
                0.seconds
            )

            io.setPosition(state.driveTrainPosition)

            waitForStart()

            while (opModeIsActive() ) {
                val t = io.time()

                println("t=$t")
                mpc.update(MecanumState(state.driveTrainVelocity,state.driveTrainPosition),io.voltage(),t)
                val u  = mpc.getU(t)

                io.driveFL = u[0]
                io.driveBL = u[1]
                io.driveBR = u[2]
                io.driveFR = u[3]

                io.update()

                state.update(io)

                rr.logState(state)
                rr.logInputs(io)
            }

            io.driveFL = 0.0
            io.driveBL = 0.0
            io.driveBR = 0.0
            io.driveFR = 0.0

            io.update()

            mpc.close()
        }
    }
}
