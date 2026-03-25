package sigmacorns.test.jolt

import org.joml.Vector2d
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

    private fun runTrajectory(projectName: String, trajectoryName: String, keepAlive: Boolean = false) {
        val trajoptDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(trajoptDir)
            .find { it.nameWithoutExtension == projectName }
            ?: throw IllegalStateException("$projectName.json not found in $trajoptDir")

        val traj = TrajoptLoader.loadTrajectory(projectFile, trajectoryName)
            ?: throw IllegalStateException("'$trajectoryName' not found in $projectName")

        val initialSample = traj.getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        sim.spawnFieldBalls()
        sim.setPosition(initialSample.pos)

        val robot = Robot(sim, blue = true)
        robot.init(initialSample.pos, apriltagTracking = false)
        // Sim X axis is negated relative to real field — flip goal X for aiming
        robot.aim.goalPosition = Vector2d(-robot.aim.goalPosition.x, robot.aim.goalPosition.y)
        robot.aim.targeting.goalPosition = robot.aim.goalPosition

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
            println("Visualizer at http://localhost:8080 — waiting for client to connect...")
            server.awaitClient()
            // Brief delay to let the browser finish initializing the 3D scene
            Thread.sleep(500)
            println("Client connected, starting trajectory.")

            val startTime = sim.time()
            var frameCount = 0

            while (true) {
                state.update(sim)
                val elapsed = sim.time() - startTime
                val elapsedSeconds = elapsed.toDouble(DurationUnit.SECONDS)

                if (!keepAlive && elapsedSeconds > traj.totalTime + 1.0) break

                if (elapsedSeconds <= traj.totalTime) {
                    val u = ltv.solve(state.mecanumState, elapsed)
                    sim.driveFL = u[0]
                    sim.driveBL = u[1]
                    sim.driveBR = u[2]
                    sim.driveFR = u[3]
                } else {
                    sim.driveFL = 0.0
                    sim.driveBL = 0.0
                    sim.driveBR = 0.0
                    sim.driveFR = 0.0
                }

                // Activate intake during intake zones
                sim.intake = if (traj.isIntakeZone(elapsedSeconds)) 1.0 else 0.0

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

    /**
     * Helper: run sim loop ticks, calling [body] each tick.
     * Returns when [body] returns false.
     */
    private fun simLoop(
        server: SimVizServer,
        robot: Robot,
        state: State,
        body: (elapsedSeconds: Double) -> Boolean
    ) {
        val startTime = sim.time()
        var frameCount = 0
        while (true) {
            state.update(sim)
            val elapsed = sim.time() - startTime
            val elapsedSeconds = elapsed.toDouble(DurationUnit.SECONDS)

            if (!body(elapsedSeconds)) break

            robot.update()
            sim.update()
            frameCount++
            if (frameCount % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }
    }

    private fun stopDrive() {
        sim.driveFL = 0.0; sim.driveBL = 0.0
        sim.driveBR = 0.0; sim.driveFR = 0.0
    }

    @Test
    fun testIntakeAndShoot() {
        val trajoptDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(trajoptDir)
            .find { it.nameWithoutExtension == "test-intake-and-shoot" }!!
        val traj = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 1")!!
        val initialSample = traj.getInitialSample()!!

        sim.spawnFieldBalls()
        sim.setPosition(initialSample.pos)

        val shotDataPath = javaClass.getResource("/shot_tuning_data.json")?.path
            ?: error("shot_tuning_data.json not found in test resources")
        val robot = Robot(sim, blue = true, shotDataPath = shotDataPath)
        robot.init(initialSample.pos, apriltagTracking = false)
        // Sim X axis is negated relative to real field — flip goal X for aiming
        robot.aim.goalPosition = Vector2d(-robot.aim.goalPosition.x, robot.aim.goalPosition.y)
        robot.aim.targeting.goalPosition = robot.aim.goalPosition

        val server = SimVizServer(sim)
        server.start()

        val ltv = LTVClient(drivetrainParameters)
        ltv.loadTrajectory(traj)

        ltv.use {
            val state = State(0.0, sim.position(), Pose2d(), Pose2d(), 0.0, 0.0, 0.seconds)

            println("Running intake-and-shoot (%.2fs)".format(traj.totalTime))
            println("Visualizer at http://localhost:8080 — waiting for client to connect...")
            server.awaitClient()
            Thread.sleep(500)
            println("Client connected, starting.")

            // Phase 1: Follow trajectory with intake active during intake zones
            val trajStartTime = sim.time()
            simLoop(server, robot, state) { t ->
                if (t <= traj.totalTime) {
                    val elapsed = sim.time() - trajStartTime
                    val u = ltv.solve(state.mecanumState, elapsed)
                    sim.driveFL = u[0]; sim.driveBL = u[1]
                    sim.driveBR = u[2]; sim.driveFR = u[3]
                } else {
                    stopDrive()
                }
                sim.intake = if (traj.isIntakeZone(t)) 1.0 else 0.0
                t <= traj.totalTime + 0.5
            }

            stopDrive()
            sim.intake = 0.0
            println("Trajectory complete. Held balls: ${sim.heldBalls.size}")

            // Phase 2: Spin up flywheel
            println("Spinning up flywheel...")
            robot.aimFlywheel = true
            simLoop(server, robot, state) { t ->
                t <= 1.0 // spin up for 1s
            }

            // Phase 3: Shoot all held balls
            val ballCount = sim.heldBalls.size
            println("Shooting $ballCount balls...")
            for (i in 0 until ballCount) {
                sim.shootBall()
                println("  Shot ${i + 1}/$ballCount")
                // Wait between shots for flywheel recovery
                simLoop(server, robot, state) { t ->
                    t <= 0.5
                }
            }

            println("All balls shot. Remaining held: ${sim.heldBalls.size}")

            // Keep alive for visualizer
            val finalPos = sim.position()
            println("Final position: (%.3f, %.3f, %.1f°)".format(
                finalPos.v.x, finalPos.v.y, Math.toDegrees(finalPos.rot)
            ))

            simLoop(server, robot, state) { true }
        }

        robot.close()
        server.stop()
    }

    @Test
    fun testIntakeAndShootMirrored() {
        val trajoptDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(trajoptDir)
            .find { it.nameWithoutExtension == "test-intake-and-shoot_mirrored" }!!
        val traj = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 1")!!
        val initialSample = traj.getInitialSample()!!

        sim.spawnFieldBalls()
        sim.setPosition(initialSample.pos)

        val shotDataPath = javaClass.getResource("/shot_tuning_data.json")?.path
            ?: error("shot_tuning_data.json not found in test resources")
        val robot = Robot(sim, blue = false, shotDataPath = shotDataPath)
        robot.init(initialSample.pos, apriltagTracking = false)
        robot.aim.goalPosition = Vector2d(-robot.aim.goalPosition.x, robot.aim.goalPosition.y)
        robot.aim.targeting.goalPosition = robot.aim.goalPosition

        val server = SimVizServer(sim)
        server.start()

        val ltv = LTVClient(drivetrainParameters)
        ltv.loadTrajectory(traj)

        ltv.use {
            val state = State(0.0, sim.position(), Pose2d(), Pose2d(), 0.0, 0.0, 0.seconds)

            println("Running intake-and-shoot mirrored (%.2fs)".format(traj.totalTime))
            println("Visualizer at http://localhost:8080 — waiting for client to connect...")
            server.awaitClient()
            Thread.sleep(500)
            println("Client connected, starting.")

            // Phase 1: Follow trajectory with intake active during intake zones
            val trajStartTime = sim.time()
            simLoop(server, robot, state) { t ->
                if (t <= traj.totalTime) {
                    val elapsed = sim.time() - trajStartTime
                    val u = ltv.solve(state.mecanumState, elapsed)
                    sim.driveFL = u[0]; sim.driveBL = u[1]
                    sim.driveBR = u[2]; sim.driveFR = u[3]
                } else {
                    stopDrive()
                }
                sim.intake = if (traj.isIntakeZone(t)) 1.0 else 0.0
                t <= traj.totalTime + 0.5
            }

            stopDrive()
            sim.intake = 0.0
            println("Trajectory complete. Held balls: ${sim.heldBalls.size}")

            // Phase 2: Spin up flywheel
            println("Spinning up flywheel...")
            robot.aimFlywheel = true
            simLoop(server, robot, state) { t ->
                t <= 1.0
            }

            // Phase 3: Shoot all held balls
            val ballCount = sim.heldBalls.size
            println("Shooting $ballCount balls...")
            for (i in 0 until ballCount) {
                sim.shootBall()
                println("  Shot ${i + 1}/$ballCount")
                simLoop(server, robot, state) { t ->
                    t <= 0.5
                }
            }

            println("All balls shot. Remaining held: ${sim.heldBalls.size}")

            val finalPos = sim.position()
            println("Final position: (%.3f, %.3f, %.1f°)".format(
                finalPos.v.x, finalPos.v.y, Math.toDegrees(finalPos.rot)
            ))

            simLoop(server, robot, state) { true }
        }

        robot.close()
        server.stop()
    }
}
