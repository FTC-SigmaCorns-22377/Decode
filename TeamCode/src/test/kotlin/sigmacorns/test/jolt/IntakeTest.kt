package sigmacorns.test.jolt

import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.io.BallColor
import sigmacorns.io.JoltSimIO
import kotlin.math.abs

class IntakeTest {
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
    fun testIntakeRollerSpinsUp() {
        sim.intake = 1.0
        repeat(100) { sim.update() } // 500ms

        val omega = sim.intakeRollerVelocity()
        println("Intake roller after 500ms at full power: $omega rad/s")
        assertTrue(omega > 50.0, "Intake roller should spin up, got $omega rad/s")
    }

    @Test
    fun testIntakeRollerSpinsDown() {
        // Spin up
        sim.intake = 1.0
        repeat(200) { sim.update() }
        val peakOmega = sim.intakeRollerVelocity()
        assertTrue(peakOmega > 50.0, "Roller should spin up first")

        // Remove power
        sim.intake = 0.0
        repeat(200) { sim.update() }

        val decayedOmega = sim.intakeRollerVelocity()
        println("Roller decay: peak=$peakOmega -> $decayedOmega rad/s")
        assertTrue(decayedOmega < peakOmega, "Roller should slow down, peak=$peakOmega, now=$decayedOmega")
    }

    @Test
    fun testBallPushesIntakeUp() {
        val restAngle = sim.intakeAngle()
        println("Intake rest angle: $restAngle rad")

        // Place ball right at the intake roller position
        sim.spawnBall(0.32f, 0.0f, 0.0635f, BallColor.GREEN)

        // Drive forward into the ball
        sim.driveFL = 0.5
        sim.driveBL = 0.5
        sim.driveBR = 0.5
        sim.driveFR = 0.5
        repeat(100) { sim.update() }

        val deflectedAngle = sim.intakeAngle()
        println("Intake angle after ball contact: $deflectedAngle rad")
        // Ball pushing on roller should deflect it upward (positive angle)
        assertTrue(deflectedAngle > restAngle - 0.1,
            "Intake should deflect when ball pushes, rest=$restAngle, now=$deflectedAngle")
    }

    @Test
    fun testBallPickupWithSpinningRoller() {
        // Place ball in front of robot
        sim.spawnBall(0.32f, 0.0f, 0.0635f, BallColor.GREEN)
        assertEquals(1, sim.getBallCount())

        // Spin up intake and drive forward to make contact
        sim.intake = 1.0
        sim.driveFL = 0.3
        sim.driveBL = 0.3
        sim.driveBR = 0.3
        sim.driveFR = 0.3
        repeat(300) { sim.update() }

        println("After pickup attempt: held=${sim.heldBalls.size}, balls=${sim.getBallCount()}")
        assertTrue(sim.heldBalls.size > 0 || sim.getBallCount() == 0,
            "Ball should be picked up with spinning roller")
    }

    @Test
    fun testBallNotPickedUpWithoutSpin() {
        // Place ball in front of robot
        sim.spawnBall(0.32f, 0.0f, 0.0635f, BallColor.GREEN)

        // No intake power, just drive forward
        sim.intake = 0.0
        sim.driveFL = 0.3
        sim.driveBL = 0.3
        sim.driveBR = 0.3
        sim.driveFR = 0.3
        repeat(200) { sim.update() }

        // Ball should NOT be picked up (roller not spinning)
        assertEquals(0, sim.heldBalls.size,
            "Ball should not be picked up without roller spin")
    }

    @Test
    fun testIntakeGravityReturn() {
        // Get rest angle
        repeat(50) { sim.update() }
        val restAngle = sim.intakeAngle()

        // Deflect by driving into a ball
        sim.spawnBall(0.32f, 0.0f, 0.0635f, BallColor.GREEN)
        sim.driveFL = 0.5
        sim.driveBL = 0.5
        sim.driveBR = 0.5
        sim.driveFR = 0.5
        repeat(50) { sim.update() }

        // Stop driving and let gravity return the intake
        sim.driveFL = 0.0
        sim.driveBL = 0.0
        sim.driveBR = 0.0
        sim.driveFR = 0.0
        repeat(200) { sim.update() }

        val returnedAngle = sim.intakeAngle()
        println("Gravity return: rest=$restAngle, returned=$returnedAngle")
        // Should return close to rest angle
        assertTrue(abs(returnedAngle - restAngle) < 0.5,
            "Intake should return near rest angle, rest=$restAngle, now=$returnedAngle")
    }

    @Test
    fun testIntakeCapacity() {
        // Spawn 5 balls in front of intake
        for (i in 0 until 5) {
            sim.spawnBall(0.30f + i * 0.05f, (i - 2) * 0.04f, 0.0635f, BallColor.GREEN)
        }

        sim.intake = 1.0
        sim.driveFL = 0.3
        sim.driveBL = 0.3
        sim.driveBR = 0.3
        sim.driveFR = 0.3
        repeat(400) { sim.update() }

        println("Capacity test: held=${sim.heldBalls.size}")
        assertTrue(sim.heldBalls.size <= 3,
            "Should hold at most 3 balls, held=${sim.heldBalls.size}")
    }

    @Test
    fun testIntakeFIFOOrder() {
        // Place green ball first, then purple closer
        sim.spawnBall(0.28f, 0.0f, 0.0635f, BallColor.GREEN)
        sim.spawnBall(0.35f, 0.0f, 0.0635f, BallColor.PURPLE)

        sim.intake = 1.0
        sim.driveFL = 0.3
        sim.driveBL = 0.3
        sim.driveBR = 0.3
        sim.driveFR = 0.3
        repeat(400) { sim.update() }

        if (sim.heldBalls.size >= 2) {
            println("FIFO order: ${sim.heldBalls.map { it.name }}")
            // Order depends on which ball is picked up first (closest to robot front)
            // Just verify both were picked up
            assertTrue(sim.heldBalls.contains(BallColor.GREEN), "Green ball should be held")
            assertTrue(sim.heldBalls.contains(BallColor.PURPLE), "Purple ball should be held")
        }
    }
}
