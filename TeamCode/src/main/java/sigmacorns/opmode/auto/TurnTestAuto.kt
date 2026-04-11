package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import sigmacorns.State
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.ltv.LTVClient
import sigmacorns.control.trajopt.TrajoptLoader
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

@Autonomous
class TurnTestAuto : SigmaOpMode() {
    override fun runOpMode() {
        // 1. Find the "turntest" project file in the trajopt directory
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir)
            .find { it.nameWithoutExtension == "turntest" }
            ?: throw IllegalStateException("turntest.json not found in $robotDir")

        // 2. Load "Trajectory 1" (needed for initial position + telemetry)
        val traj = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 1")
            ?: throw IllegalStateException("'Trajectory 1' not found in turntest project")

        // 3. Set the robot's starting position to match the trajectory's first waypoint
        val initialSample = traj.getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")
        io.setPosition(initialSample.pos)

        // 4. Create the LTV controller — use precomputed .bin if available, else fall back
        val ltv = LTVClient(drivetrainParameters).also { it.loadTrajectory(traj) }
        ltv.use {

            // 5. Build the state object that tracks robot pose/velocity
            val state = State(
                0.0,
                io.position(),
                Pose2d(),
                Pose2d(),
                0.0,
                0.0,
                0.seconds,
            )

            // --- Wait for driver to press START ---
            waitForStart()

            // Re-sync position in case the robot moved during init
            io.setPosition(initialSample.pos)
            val startTime = io.time()

            // 6. Main control loop
            while (opModeIsActive()) {
                // Update state from hardware (encoders, IMU, etc.)
                state.update(io)

                // How far into the trajectory we are
                val elapsed = io.time() - startTime

                // LTV solver returns [FL, BL, BR, FR] motor powers
                val u = ltv.solve(state.mecanumState, elapsed)

                // Apply motor powers
                io.driveFL = u[0]
                io.driveBL = u[1]
                io.driveBR = u[2]
                io.driveFR = u[3]

                // Telemetry so you can see what's happening on the driver station
                telemetry.addData("elapsed", "%.2f / %.2f s".format(
                    elapsed.toDouble(DurationUnit.SECONDS),
                    traj.totalTime
                ))
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
            }

            // 7. Stop motors when OpMode ends
            io.driveFL = 0.0
            io.driveBL = 0.0
            io.driveBR = 0.0
            io.driveFR = 0.0
            io.update()
        }
    }
}