package sigmacorns.test

import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.io.BallColor
import sigmacorns.io.JoltSimIO
import kotlin.math.abs

class JoltSimTest {
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
    fun testBasicPhysics() {
        // Drive forward (positive FL,BL,BR,FR powers)
        sim.driveFL = 0.5
        sim.driveBL = 0.5
        sim.driveBR = 0.5
        sim.driveFR = 0.5

        // Run for 500ms
        repeat(100) { sim.update() }

        val pos = sim.position()
        // Robot should have moved in some direction
        val distSq = pos.v.x * pos.v.x + pos.v.y * pos.v.y
        assertTrue(distSq > 0.001, "Robot should have moved, pos=$pos")
    }

    @Test
    fun testBallSpawning() {
        sim.spawnBall(0.5f, 0.5f, 0.0635f, BallColor.GREEN)
        sim.spawnBall(-0.5f, 0.5f, 0.0635f, BallColor.PURPLE)

        assertEquals(2, sim.getBallCount())

        val balls = sim.getBallStates()
        assertEquals(2, balls.size)
        assertEquals(BallColor.GREEN, balls[0].color)
        assertEquals(BallColor.PURPLE, balls[1].color)
    }

    @Test
    fun testFieldBallSpawning() {
        sim.spawnFieldBalls()
        assertEquals(18, sim.getBallCount(), "Should spawn 18 field balls")
    }

    @Test
    fun testIntakePickup() {
        // Place a ball right at the intake sensor area
        sim.spawnBall(0.0f, 0.2f, 0.0635f, BallColor.GREEN)
        assertEquals(1, sim.getBallCount())

        // Turn on intake and step
        sim.intake = 1.0
        repeat(10) { sim.update() }

        // Ball should be picked up
        assertTrue(sim.heldBalls.size > 0 || sim.getBallCount() == 0,
            "Ball should be picked up by intake or still in field")
    }

    @Test
    fun testWallCollision() {
        // Place robot near the +X wall
        sim.setPosition(sigmacorns.math.Pose2d(1.7, 0.0, 0.0))

        // Drive toward the wall (all wheels forward = robot moves in +Z in Jolt = +Y in sim...
        // Actually depends on heading. At heading=0, forward is +Z in Jolt = +Y in sim
        // Let's drive with heading facing the wall
        sim.setPosition(sigmacorns.math.Pose2d(1.6, 0.0, 0.0))

        // Drive hard toward +X (strafe with heading=0)
        sim.driveFL = 1.0
        sim.driveBL = -1.0
        sim.driveBR = -1.0
        sim.driveFR = 1.0

        // Run for 2 seconds
        repeat(400) { sim.update() }

        val pos = sim.position()
        val halfField = 3.6576 / 2.0
        assertTrue(pos.v.x < halfField + 0.01,
            "Robot should not pass through wall, x=${pos.v.x}, limit=$halfField")
    }

    @Test
    fun testMaxHeldBalls() {
        // Spawn 5 balls near intake
        for (i in 0 until 5) {
            sim.spawnBall(0.0f, 0.15f + i * 0.01f, 0.0635f, BallColor.GREEN)
        }

        sim.intake = 1.0
        repeat(200) { sim.update() }

        assertTrue(sim.heldBalls.size <= 3, "Should hold at most 3 balls, held=${sim.heldBalls.size}")
    }

    @Test
    fun testWithVisualizer() {
        sim.spawnFieldBalls()

        val server = sigmacorns.sim.viz.SimVizServer(sim)
        server.start()

        println("Visualizer running at http://localhost:8080")
        println("Press Ctrl+C to stop")

        // Drive in a circle
        sim.driveFL = 0.3
        sim.driveBL = 0.3
        sim.driveBR = 0.5
        sim.driveFR = 0.5

        var frameCount = 0
        while (true) {
            sim.update()
            frameCount++
            if (frameCount % 4 == 0) { // ~50Hz broadcast at 200Hz sim
                server.broadcastState()
            }
            Thread.sleep(5)
        }
    }
}
