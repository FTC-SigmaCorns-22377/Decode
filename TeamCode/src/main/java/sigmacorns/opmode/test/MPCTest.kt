package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullftc.choreolib.Choreo
import dev.nullftc.choreolib.sample.MecanumSample
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.joml.Vector2d
import org.joml.Vector3d
import kotlin.math.hypot
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.ContourSelectionMode
import sigmacorns.io.HardwareIO
import sigmacorns.io.MPCClient
import sigmacorns.io.RerunLogging
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.io.SolverRequestType
import sigmacorns.math.Pose2d
import sigmacorns.math.toPose2d
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
    companion object {
        private val MAX_VELOCITY = drivetrainParameters.motor.freeSpeed*drivetrainParameters.wheelRadius // m/s, adjust based on your robot's max speed

        fun velocityToColor(vx: Double, vy: Double): Int {
            val speed = hypot(vx, vy)
            val t = (speed / MAX_VELOCITY).coerceIn(0.0, 1.0)
            // Gradient from blue (slow) to green (medium) to red (fast)
            val r: Int
            val g: Int
            val b: Int
            if (t < 0.5) {
                // Blue to Green
                val t2 = t * 2
                r = 0
                g = (255 * t2).toInt()
                b = (255 * (1 - t2)).toInt()
            } else {
                // Green to Red
                val t2 = (t - 0.5) * 2
                r = (255 * t2).toInt()
                g = (255 * (1 - t2)).toInt()
                b = 0
            }
            return (0xFF shl 24) or (r shl 16) or (g shl 8) or b
        }
    }

    override fun runOpMode() {

        val traj = Choreo().loadTrajectory<MecanumSample>(trajName).get()

        val ll = (io as? HardwareIO)?.limelight
        ll?.pipelineSwitch(1)

        io.setPosition(traj.getInitialSample()!!.pose.toPose2d())

        MPCClient(drivetrainParameters, Network.LIMELIGHT, contourSelectionMode = ContourSelectionMode.POSITION, sampleLookahead = 2).use { mpc ->
            rerunSink("MPCTest($trajName)").use { rr ->
                mpc.setTarget(traj, SolverRequestType.CONTOURING)

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

                io.setPosition(mpc.startPose())

                rr.logState(state)
                rr.logLineStrip("path/pos", mpc.path?.map { it.pos.v } ?: mpc.trackingReferencePositions)

                while (opModeIsActive()) {
                    val t = io.time()
                    state.update(io)
                    mpc.update(
                        MecanumState(io.velocity(), io.position()),
                        io.voltage(),
                        t
                    )
                    val u = mpc.getU(t)

                    // Log predicted path as velocity-colored points
                    val predictedPoints = mpc.predictedEvolution.map { Vector3d(it.pos.v.x, it.pos.v.y, 0.0) }
                    val predictedColors = mpc.predictedEvolution.map { velocityToColor(it.vel.v.x, it.vel.v.y) }.toIntArray()
                    rr.logPoints3DWithColors("predictedPath", predictedPoints, predictedColors, 0.02f)

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
                        // Log target contour as velocity-colored point
                        val contourPoint = listOf(Vector3d(it.pos.v.x, it.pos.v.y, 0.0))
                        val contourColor = intArrayOf(velocityToColor(it.vel.v.x, it.vel.v.y))
                        rr.logPoints3DWithColors("contour/target", contourPoint, contourColor, 0.04f)

                        rr.logScalar("contour/px", it.pos.v.x)
                        rr.logScalar("contour/py", it.pos.v.y)
                        rr.logScalar("contour/theta", it.pos.rot)

                        rr.logScalar("contour/vx", it.vel.v.x)
                        rr.logScalar("contour/vy", it.vel.v.y)
                        rr.logScalar("contour/omega", it.vel.rot)

                        rr.logScalar("contour/index", mpc.latestSampleI)
                    }
                }
            }
        }
    }
}