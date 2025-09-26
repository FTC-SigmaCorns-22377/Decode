package sigmacorns.opmode.test

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

class MPCSingleContourTest(io: SigmaIO): SigmaOpMode(io) {
    override fun runOpMode() {
        val contours = listOf(
            Contour(Pose2d(0.0,0.0,0.0), Pose2d(0.5,0.0,0.0),0.0,100.0)
        )

        val mpc = MPCClient(
            drivetrainParameters,
            if (LIMELIGHT_CONNECTED) Network.LIMELIGHT else Network.SIM_MPC
        )

        RerunLogging.connect("MPCBenchmarkTest","rerun+http://127.0.0.1:9876/proxy").use { rr ->
            val sim = SimIO()

            mpc.setTarget(contours)

            val state = State(
                0.0,
                contours[0].pos,
                contours[0].vel,
                Pose2d(Vector2d(), 0.0),
                0.0,
                0.0,
                0.seconds
            )

            sim.setPosition(state.driveTrainPosition)

            while (sim.time() < 5.seconds) {
                val t = sim.time()
                mpc.update(MecanumState(state.driveTrainVelocity,state.driveTrainPosition),12.0,t)
                val u  = mpc.getU(t)

                sim.driveFL = u[0]
                sim.driveBL = u[1]
                sim.driveBR = u[2]
                sim.driveFR = u[3]

                sim.update()
                Thread.sleep(SIM_UPDATE_TIME.inWholeMilliseconds)

                state.update(sim)

                rr.logState(state)

            }
        }
    }
}