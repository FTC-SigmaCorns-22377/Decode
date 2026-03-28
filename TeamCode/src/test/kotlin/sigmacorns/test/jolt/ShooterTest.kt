package sigmacorns.test.jolt

import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.io.BallColor
import sigmacorns.io.JoltSimIO
import kotlin.math.abs

class ShooterTest {
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
    fun testHoodServoTracksTarget() {
        // Set hood to max (1.0 = 70 degrees)
        sim.hood = 1.0
        repeat(200) { sim.update() } // 1 second

        val angle = Math.toDegrees(sim.hoodPosition())
        println("Hood at servo=1.0 after 1s: ${angle}°")
        assertTrue(angle > 60.0, "Hood should approach 70°, got $angle°")
    }

    @Test
    fun testHoodServoClamps() {
        // Try to exceed range
        sim.hood = 2.0
        repeat(400) { sim.update() }

        val angle = Math.toDegrees(sim.hoodPosition())
        println("Hood clamped: ${angle}°")
        assertTrue(angle <= 70.1, "Hood should clamp to 70°, got $angle°")
        assertTrue(angle >= -0.1, "Hood should not go negative, got $angle°")

        // Try negative
        sim.hood = -1.0
        repeat(400) { sim.update() }

        val angle2 = Math.toDegrees(sim.hoodPosition())
        assertTrue(angle2 >= -0.1, "Hood should clamp to 0°, got $angle2°")
    }

    @Test
    fun testHoodAffectsLaunchAngle() {
        // Shoot at hood=0 (nearly horizontal)
        sim.heldBalls.add(BallColor.GREEN)
        sim.flywheel = 1.0
        sim.hood = 0.0
        repeat(400) { sim.update() } // spin up flywheel + settle hood

        sim.shootBall()
        repeat(20) { sim.update() }

        val lowBalls = sim.getBallStates()
        val lowZ = if (lowBalls.isNotEmpty()) lowBalls.last().z else 0f

        // Reset
        sim.close()
        sim = JoltSimIO()

        // Shoot at hood=1.0 (steep ~70°)
        sim.heldBalls.add(BallColor.GREEN)
        sim.flywheel = 1.0
        sim.hood = 1.0
        repeat(400) { sim.update() }

        sim.shootBall()
        repeat(20) { sim.update() }

        val highBalls = sim.getBallStates()
        val highZ = if (highBalls.isNotEmpty()) highBalls.last().z else 0f

        println("Low hood z=$lowZ, High hood z=$highZ")
        assertTrue(highZ > lowZ, "Steep hood should launch higher, lowZ=$lowZ, highZ=$highZ")
    }

    @Test
    fun testShootFIFOOrder() {
        // Load 3 balls with different colors
        sim.heldBalls.add(BallColor.GREEN)
        sim.heldBalls.add(BallColor.PURPLE)
        sim.heldBalls.add(BallColor.GREEN)

        // Spin up flywheel
        sim.flywheel = 1.0
        repeat(400) { sim.update() }

        // Shoot first ball - should be GREEN
        sim.shootBall()
        assertEquals(2, sim.heldBalls.size)
        assertEquals(BallColor.PURPLE, sim.heldBalls[0], "Second ball should now be first (PURPLE)")

        // Shoot second - should be PURPLE
        sim.shootBall()
        assertEquals(1, sim.heldBalls.size)
        assertEquals(BallColor.GREEN, sim.heldBalls[0], "Third ball should now be first (GREEN)")

        // Shoot third
        sim.shootBall()
        assertEquals(0, sim.heldBalls.size)
    }

    @Test
    fun testHoodAtMinAngle() {
        sim.hood = 0.0
        repeat(400) { sim.update() }

        val angle = sim.hoodPosition()
        println("Hood min angle: ${Math.toDegrees(angle)}°")
        assertTrue(angle < Math.toRadians(5.0), "Hood at min should be near 0°, got ${Math.toDegrees(angle)}°")
    }

    @Test
    fun testHoodAtMaxAngle() {
        sim.hood = 1.0
        repeat(400) { sim.update() }

        val angle = sim.hoodPosition()
        println("Hood max angle: ${Math.toDegrees(angle)}°")
        assertTrue(angle > Math.toRadians(65.0), "Hood at max should be near 70°, got ${Math.toDegrees(angle)}°")
    }
}
