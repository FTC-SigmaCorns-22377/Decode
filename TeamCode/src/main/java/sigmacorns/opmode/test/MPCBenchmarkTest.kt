package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullftc.choreolib.Choreo
import dev.nullftc.choreolib.sample.MecanumSample
import org.joml.Vector2d
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.ContourLoader
import sigmacorns.io.MPCClient
import sigmacorns.io.RerunLogging
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.io.SigmaIO
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumState
import kotlin.time.Duration.Companion.seconds

@TeleOp
class MPCBenchmarkTest(io: SigmaIO): SigmaOpMode(io) {
    override fun runOpMode() {
        waitForStart()

        val contours = ContourLoader.load(Choreo().loadTrajectory<MecanumSample>("New Path (1)").get())

        val mpc = MPCClient(drivetrainParameters, if(LIMELIGHT_CONNECTED) Network.LIMELIGHT else Network.SIM_MPC, sampleLookahead = 2)

        RerunLogging.connect("MPCBenchmarkTest","rerun+http://127.0.0.1:9876/proxy").use { rr ->
            val sim = SimIO()

            mpc.setTarget(contours)

            println(contours.map { it.toString() })

            val state = State(
                0.0,
                contours[0].pos,
                contours[0].vel,
                Pose2d(Vector2d(),0.0),
                0.0,
                0.0,
                0.seconds
            )

            sim.setPosition(state.driveTrainPosition)

            rr.logState(state)
            rr.logLineStrip("benchmarkPath",contours.map { it.pos.v })

            while (sim.time() < 8.seconds) {
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