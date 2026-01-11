package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullftc.choreolib.Choreo
import dev.nullftc.choreolib.sample.MecanumSample
import org.joml.Vector2d
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.HardwareIO
import sigmacorns.io.MPCClient
import sigmacorns.io.RerunLogging
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.io.SolverRequestType
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumState
import kotlin.time.Duration.Companion.seconds


enum class LaunchZones(val launchSpeed: Double) {
    Close(1.0),
    Far(2.0)
}

@TeleOp(group = "test")
class MPCForward: MPCTest("forward")

@TeleOp(group = "test")
class MPCReturn: MPCTest("test return")

open class MPCTest(val trajName: String): SigmaOpMode() {
    override fun runOpMode() {

        val traj = Choreo().loadTrajectory<MecanumSample>(trajName).get()

        val ll = (io as HardwareIO).limelight
        ll?.pipelineSwitch(1)

        MPCClient(drivetrainParameters, solverIP(), sampleLookahead = 2).use { mpc ->
            rerunSink("MPCTest($trajName)").use { rr ->
                mpc.setTarget(traj, SolverRequestType.CONTOURING)

                val state = State(
                    0.0,
                    mpc.startPose(),
                    Pose2d(),
                    Pose2d(Vector2d(), 0.0),
                    0.0,
                    0.0,
                    0.seconds
                )


                waitForStart()

                io.setPosition(mpc.startPose())

                rr.logState(state)
                //rr.logLineStrip("path/pos", mpc.path!!.map { it.pos.v })

                while (opModeIsActive()) {
                    val t = io.time()
                    state.update(io)
                    mpc.update(
                        MecanumState(io.velocity(), io.position()),
                        io.voltage(),
                        t
                    )
                    val u = mpc.getU(t)

                    rr.logLineStrip("predictedPath", mpc.predictedEvolution.map { it.pos.v })

                    io.driveFL = u[0]
                    io.driveBL = u[1]
                    io.driveBR = u[2]
                    io.driveFR = u[3]

                    io.update()

                    rr.logState(state)
                    rr.logInputs(io)

                    println("ROTATION=${io.position().rot}, state=${state.driveTrainPosition.rot}")

                    if(SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)

                    mpc.lastTargetContour?.let {
//                        rr.logScalar("path/px", it.pos.v.x)
//                        rr.logScalar("path/py", it.pos.v.y)
//                        rr.logScalar("path/theta", it.pos.rot)
//
//                        rr.logScalar("path/vx", it.vel.v.x)
//                        rr.logScalar("path/vy", it.vel.v.y)
//                        rr.logScalar("path/omega", it.vel.rot)
                    }
                }
            }
        }
    }
}