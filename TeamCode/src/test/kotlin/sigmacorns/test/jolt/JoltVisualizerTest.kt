package sigmacorns.test.jolt

import com.qualcomm.robotcore.hardware.Gamepad
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.io.BallColor
import sigmacorns.io.JoltSimIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.SimGamepad
import sigmacorns.subsystem.DriveController

class JoltVisualizerTest {
    private lateinit var sim: JoltSimIO

    @BeforeEach
    fun setUp() {
        sim = JoltSimIO()
    }

    @AfterEach
    fun tearDown() {
        sim.close()
    }

    @Test
    fun testShooterWithVisualizer() {
        // Place 3 balls just ahead of the robot for easy intake
        sim.spawnBall(0.2f, 0.0f, 0.0635f, BallColor.GREEN)
        sim.spawnBall(0.25f, 0.05f, 0.0635f, BallColor.PURPLE)
        sim.spawnBall(0.25f, -0.05f, 0.0635f, BallColor.GREEN)

        val server = sigmacorns.sim.viz.SimVizServer(sim, 8081)
        server.start()

        println("Shooter visualizer running at http://localhost:8081")

        // Phase 1: Drive forward with intake to pick up balls (1 second)
        sim.driveFL = 0.3
        sim.driveBL = 0.3
        sim.driveBR = 0.3
        sim.driveFR = 0.3
        sim.intake = 1.0
        repeat(200) { i ->
            sim.update()
            if (i % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }
        sim.driveFL = 0.0
        sim.driveBL = 0.0
        sim.driveBR = 0.0
        sim.driveFR = 0.0
        sim.intake = 0.0

        println("Picked up ${sim.heldBalls.size} balls")

        // Phase 2: Spin up flywheel (2 seconds)
        sim.flywheel = 1.0
        repeat(400) { i ->
            sim.update()
            if (i % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }

        // Phase 3: Rotate turret left while shooting (servo=0.65 = ~54° left)
        sim.turret = 0.65
        val ballsToShoot = sim.heldBalls.size

        for (shot in 0 until ballsToShoot) {
            // Rotate turret for 500ms
            repeat(100) { i ->
                sim.update()
                if (i % 4 == 0) server.broadcastState()
                Thread.sleep(5)
            }

            println("Shot $shot: turret=${Math.toDegrees(sim.turretPosition())}°, " +
                    "flywheel=${(sim.flywheelVelocity() * 60 / (2 * Math.PI)).toInt()} RPM")
            sim.shootBall()

            // Let ball fly for 1 second
            repeat(200) { i ->
                sim.update()
                if (i % 4 == 0) server.broadcastState()
                Thread.sleep(5)
            }
        }

        // Phase 4: Drive forward while turret sweeps
        sim.driveFL = 0.3
        sim.driveBL = 0.3
        sim.driveBR = 0.3
        sim.driveFR = 0.3
        sim.turret = 0.25 // servo=0.25 targets -π/2 (sweep right)

        repeat(600) { i ->
            sim.update()
            if (i % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }

        // Assertions on final state
        val pos = sim.position()
        assertTrue(pos.v.x > 0.1, "Robot should have moved forward")
        assertEquals(0, sim.heldBalls.size, "All balls should have been shot")

        server.stop()
    }

    @Test
    fun testWithWasdDrive() {
        sim.spawnFieldBalls()

        val server = sigmacorns.sim.viz.SimVizServer(sim)
        server.start()

        val driveController = DriveController()

        println("Visualizer running at http://localhost:8080")
        println("Use WASD keys in the browser to drive. Press Ctrl+C to stop.")

        var frameCount = 0
        while (true) {
            val wasd = server.wasdState
            val vx = if (wasd.w) 1.0 else if (wasd.s) -1.0 else 0.0
            val vy = if (wasd.a) -1.0 else if (wasd.d) 1.0 else 0.0
            val omega = if (wasd.q) -1.0 else if (wasd.e) 1.0 else 0.0
            driveController.drive(Pose2d(vx, vy, omega), sim)
            sim.turret = 0.5

            sim.update()
            frameCount++
            if (frameCount % 4 == 0) {
                server.broadcastState()
            }
            Thread.sleep(5)
        }
    }

    @Test
    fun testWithVisualizer() {
        sim.spawnFieldBalls()

        val server = sigmacorns.sim.viz.SimVizServer(sim)
        server.start()

        val gamepad = Gamepad()
        val simGamepad = SimGamepad(gamepad)
        val driveController = DriveController()

        println("Visualizer running at http://localhost:8080")
        println("Use a connected gamepad to drive. Press Ctrl+C to stop.")

        var frameCount = 0
        while (true) {
            simGamepad.tick()
            driveController.update(gamepad, sim)

            // Right trigger = intake, left bumper = flywheel, right bumper = shoot
            sim.intake = gamepad.right_trigger.toDouble()
            sim.flywheel = if (gamepad.left_bumper) 1.0 else 0.0
            if (gamepad.right_bumper) sim.shootBall()

            sim.update()
            frameCount++
            if (frameCount % 4 == 0) {
                server.broadcastState()
            }
            Thread.sleep(5)
        }
    }
}
