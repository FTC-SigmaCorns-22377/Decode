package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullftc.choreolib.Choreo
import dev.nullftc.choreolib.sample.MecanumSample
import org.joml.Vector2d
import sigmacorns.State
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.ContourLoader
import sigmacorns.io.MPCClient
import sigmacorns.io.RerunLogging
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumState
import kotlin.time.Duration.Companion.seconds

@TeleOp(group = "test")
class MPCForward: SigmaOpMode() {
    override fun runOpMode() {
        waitForStart()

        val contours = ContourLoader.load(Choreo().loadTrajectory<MecanumSample>("forward").get())

        val mpc = MPCClient(drivetrainParameters, solverIP(), sampleLookahead = 4)

        RerunLogging.save("MPCBenchmarkTest", "/sdcard/FIRST/MPCForward.rrd").use { rr ->
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

            io.setPosition(state.driveTrainPosition)

            rr.logState(state)
            rr.logLineStrip("benchmarkPath",contours.map { it.pos.v })

            while (io.time() < 8.seconds) {
                val t = io.time()
                mpc.update(MecanumState(state.driveTrainVelocity, state.driveTrainPosition),12.0,t)
                val u  = mpc.getU(t)

                rr.logLineStrip("predictedPath",mpc.getPredictedEvolution().map { it.v })

                io.driveFL = u[0]
                io.driveBL = u[1]
                io.driveBR = u[2]
                io.driveFR = u[3]

                io.update()

                state.update(io)

                rr.logState(state)

            }
        }
    }
}