package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.State
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.ltv.LTVClient
import sigmacorns.control.ltv.LTVRerunLogger
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.system.measureNanoTime
import kotlin.time.Duration.Companion.nanoseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

@TeleOp(group = "test")
class LTVSimple : LTVTest("Trajectory 1")

open class LTVTest(val trajName: String) : SigmaOpMode() {
    override fun runOpMode() {
        val projectFile = MPCTest.findProjectFile()
            ?: throw IllegalStateException("No trajopt project file found in ${TrajoptLoader.robotTrajoptDir()}")

        val traj = TrajoptLoader.loadTrajectory(projectFile, trajName)
            ?: throw IllegalStateException("Trajectory '$trajName' not found in project file")

        val initialSample = traj.getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        io.setPosition(initialSample.pos)

        val rr = if (SIM) LTVRerunLogger(rerunSink("LTVTest($trajName)")) else null

        LTVClient(drivetrainParameters).use { ltv ->
            ltv.loadTrajectory(traj)
            rr?.logTrajectoryPath(traj)

            val state = State(
                0.0,
                io.position(),
                Pose2d(),
                Pose2d(),
                0.0,
                0.0,
                0.seconds,
            )

            waitForStart()

            io.setPosition(initialSample.pos)
            val startTime = io.time()

            while (opModeIsActive()) {
                state.update(io)
                val elapsed = io.time() - startTime

                //if (ltv.isComplete(elapsed)) break
                var u = doubleArrayOf()
                val nanos = measureNanoTime {
                     u = ltv.solve(state.mecanumState, elapsed)
                }
                println("solved in ${nanos.nanoseconds.toDouble(DurationUnit.MILLISECONDS)}ms")


                // u = [FL, BL, BR, FR] in SigmaIO order
                io.driveFL = u[0]*12.0/io.voltage()
                io.driveBL = u[1]*12.0/io.voltage()
                io.driveBR = u[2]*12.0/io.voltage()
                io.driveFR = u[3]*12.0/io.voltage()

                rr?.logState(state)
                rr?.logInputs(io)
                rr?.logControls(u)
                rr?.logTrackingError(state, traj, elapsed.toDouble(DurationUnit.SECONDS))

                telemetry.addData("elapsed", "%.2f s".format(elapsed.toDouble(DurationUnit.SECONDS)))
                telemetry.addData("FL", "%.3f".format(u[0]))
                telemetry.addData("BL", "%.3f".format(u[1]))
                telemetry.addData("BR", "%.3f".format(u[2]))
                telemetry.addData("FR", "%.3f".format(u[3]))
                telemetry.addData("pos", "(%.3f, %.3f, %.1f°)".format(
                    state.driveTrainPosition.v.x,
                    state.driveTrainPosition.v.y,
                    Math.toDegrees(state.driveTrainPosition.rot),
                ))
                telemetry.update()

                io.update()

                if (SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)
            }

            // Stop motors
            io.driveFL = 0.0
            io.driveBL = 0.0
            io.driveBR = 0.0
            io.driveFR = 0.0
            io.update()
        }

        rr?.close()
    }
}
