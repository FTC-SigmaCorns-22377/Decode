package sigmacorns.test.jolt

import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.Robot
import sigmacorns.State
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.ltv.LTVClient
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.io.JoltSimIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.viz.SimVizServer
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

class JoltAutoTest {
    private lateinit var sim: JoltSimIO

    @BeforeEach
    fun setUp() {
        SigmaOpMode.SIM = true
        sim = JoltSimIO()
    }

    @AfterEach
    fun tearDown() {
        sim.close()
    }

    private fun runTrajectory(projectName: String, trajectoryName: String) {
        val trajoptDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(trajoptDir)
            .find { it.nameWithoutExtension == projectName }
            ?: throw IllegalStateException("$projectName.json not found in $trajoptDir")

        val traj = TrajoptLoader.loadTrajectory(projectFile, trajectoryName)
            ?: throw IllegalStateException("'$trajectoryName' not found in $projectName")

        val initialSample = traj.getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        sim.setPosition(initialSample.pos)

        val robot = Robot(sim, blue = true)
        robot.init(initialSample.pos, apriltagTracking = false)

        val server = SimVizServer(sim)
        server.start()

        val ltv = LTVClient(drivetrainParameters)
        ltv.loadTrajectory(traj)

        ltv.use {
            val state = State(
                0.0,
                sim.position(),
                Pose2d(),
                Pose2d(),
                0.0,
                0.0,
                0.seconds,
            )

            println("Running '$trajectoryName' from $projectName (%.2fs)".format(traj.totalTime))
            println("Visualizer at http://localhost:8080")

            val startTime = sim.time()
            var frameCount = 0

            while (true) {
                state.update(sim)
                val elapsed = sim.time() - startTime
                val elapsedSeconds = elapsed.toDouble(DurationUnit.SECONDS)

                if (elapsedSeconds > traj.totalTime + 1.0) break

                val u = ltv.solve(state.mecanumState, elapsed)
                sim.driveFL = u[0]
                sim.driveBL = u[1]
                sim.driveBR = u[2]
                sim.driveFR = u[3]

                robot.update()
                sim.update()

                frameCount++
                if (frameCount % 4 == 0) {
                    server.broadcastState()
                }
                Thread.sleep(5)
            }

            // Stop motors
            sim.driveFL = 0.0
            sim.driveBL = 0.0
            sim.driveBR = 0.0
            sim.driveFR = 0.0
            sim.update()
            server.broadcastState()

            val finalPos = sim.position()
            println("Final position: (%.3f, %.3f, %.1f°)".format(
                finalPos.v.x, finalPos.v.y, Math.toDegrees(finalPos.rot)
            ))
        }

        robot.close()
        server.stop()
    }

    @Test
    fun testAuto1() {
        runTrajectory("auto-1", "Trajectory 1")
    }

    @Test
    fun testAuto1Mirrored() {
        runTrajectory("auto-1_mirrored", "Trajectory 1")
    }
}
