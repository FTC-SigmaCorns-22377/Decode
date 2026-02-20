package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.Vector2d
import org.joml.Vector3d
import kotlin.math.hypot
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.subsystem.DriveController
import sigmacorns.control.mpc.ContourSelectionMode
import sigmacorns.math.Pose2d
import sigmacorns.io.HardwareIO
import sigmacorns.control.mpc.MPCClient
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.io.DrakeSimIO
import sigmacorns.sim.viz.ErrorState
import sigmacorns.sim.viz.PathPoint
import java.io.File
import kotlin.math.PI
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

@TeleOp(group = "test")
class MPCForward: MPCTest("forward")

@TeleOp(group = "test")
class MPCIntake1: MPCTest("intake_1")

@TeleOp(group = "test")
class MPCReturn: MPCTest("Trajectory 2")

open class MPCTest(val trajName: String): SigmaOpMode() {
    companion object {
        private val MAX_VELOCITY = drivetrainParameters.motor.freeSpeed * drivetrainParameters.wheelRadius

        fun velocityToColor(vx: Double, vy: Double): Int {
            val speed = hypot(vx, vy)
            val t = (speed / MAX_VELOCITY).coerceIn(0.0, 1.0)
            val r: Int
            val g: Int
            val b: Int
            if (t < 0.5) {
                val t2 = t * 2
                r = 0
                g = (255 * t2).toInt()
                b = (255 * (1 - t2)).toInt()
            } else {
                val t2 = (t - 0.5) * 2
                r = (255 * t2).toInt()
                g = (255 * (1 - t2)).toInt()
                b = 0
            }
            return (0xFF shl 24) or (r shl 16) or (g shl 8) or b
        }

        /** Find the first trajectory project file in the trajopt directory. */
        fun findProjectFile(): File? {
            val robotDir = TrajoptLoader.robotTrajoptDir()
            if (robotDir.exists()) {
                return TrajoptLoader.findProjectFiles(robotDir).firstOrNull()
            }
            return null
        }
    }

    override fun runOpMode() {
        // Load trajectory from trajopt project file
        val projectFile = findProjectFile()
            ?: throw IllegalStateException("No trajopt project file found in ${TrajoptLoader.robotTrajoptDir()}")

        val traj = TrajoptLoader.loadTrajectory(projectFile, trajName)
            ?: throw IllegalStateException("Trajectory '$trajName' not found in project file")

        val ll = (io as? HardwareIO)?.limelight
        ll?.pipelineSwitch(1)

        val initialSample = traj.getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        io.setPosition(initialSample.pos)

        MPCClient(
            drivetrainParameters,
            Network.LIMELIGHT,
            contourSelectionMode = ContourSelectionMode.POSITION,
            preIntegrate = 40.milliseconds,
            sampleLookahead = 0
        ).use { mpc ->
            rerunSink("MPCTest($trajName)").use { rr ->
                rr.disable = true
                mpc.setTarget(traj)

                val drakeIO = io as? DrakeSimIO
                if (drakeIO != null) {
                    val pathPoints = traj.samples.map { PathPoint(it.x, it.y) }
                    drakeIO.setChoreoPath(pathPoints)
                }

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

                // Log path as line strip
                val pathPoints = mpc.path?.map { it.lineP } ?: emptyList()
                rr.logLineStrip("path/pos", pathPoints)

                val driveController = DriveController()

                while (opModeIsActive()) {
                    val t = io.time()
                    state.update(io)

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

                    // Log drive, strafe, turn values
                    rr.logScalar("mpc/drive", drive)
                    rr.logScalar("mpc/strafe", strafe)
                    rr.logScalar("mpc/turn", turn)

                    // Log predicted path as velocity-colored points
                    val predictedPoints = mpc.predictedEvolution.map {
                        Vector3d(it.pos.v.x, it.pos.v.y, 0.0)
                    }
                    val predictedColors = mpc.predictedEvolution.map {
                        velocityToColor(it.vel.v.x, it.vel.v.y)
                    }.toIntArray()
                    rr.logPoints3DWithColors("predictedPath", predictedPoints, predictedColors, 0.02f)

                    // Convert drive, strafe, turn to motor powers using DriveController
                    val robotPower = Pose2d(drive, strafe, turn)
                    driveController.drive(robotPower, io)

                    io.update()

                    rr.logState(state)
                    rr.logInputs(io)

                    if (drakeIO != null) {
                        val targetPoints = mpc.predictedEvolution
                        drakeIO.setTrackingTarget(targetPoints.map { PathPoint(it.pos.v.x, it.pos.v.y) })
                        val pos = io.position()
                        val vel = io.velocity()
                        val closest = traj.samples.minByOrNull {
                            hypot(pos.v.x - it.x, pos.v.y - it.y)
                        }
                        if (closest != null) {
                            val errX = pos.v.x - closest.x
                            val errY = pos.v.y - closest.y
                            var errYaw = pos.rot - closest.heading
                            while (errYaw > PI) errYaw -= 2 * PI
                            while (errYaw < -PI) errYaw += 2 * PI
                            val errVx = vel.v.x - closest.vx
                            val errVy = vel.v.y - closest.vy
                            val errOmega = vel.rot - closest.omega
                            drakeIO.setTrackingError(ErrorState(errX, errY, errYaw, errVx, errVy, errOmega))
                        } else {
                            drakeIO.setTrackingError(null)
                        }
                    }

                    println("ROTATION=${io.position().rot}, state=${state.driveTrainPosition.rot}")

                    if (SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)

                    mpc.lastTargetContour?.let { contour ->
                        // Log target contour as point
                        val contourPoint = listOf(Vector3d(contour.lineP.x, contour.lineP.y, 0.0))
                        val contourColor = intArrayOf(0xFFFF0000.toInt()) // Red
                        rr.logPoints3DWithColors("contour/target", contourPoint, contourColor, 0.04f)

                        rr.logScalar("contour/px", contour.lineP.x)
                        rr.logScalar("contour/py", contour.lineP.y)
                        rr.logScalar("contour/theta", contour.targetTheta)
                        rr.logScalar("contour/omega", contour.targetOmega)
                        rr.logScalar("contour/index", mpc.latestSampleI)
                    }
                }
            }
        }
    }
}
