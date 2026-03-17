package sigmacorns.test

import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.io.BallColor
import sigmacorns.io.JoltSimIO

/**
 * Tests for goal scoring and classifier ramp behavior.
 *
 * Coordinate mapping: sim (x, y, z) → Jolt (-y, z, x)
 *   sim x = Jolt Z (forward), sim y = -Jolt X (left), sim z = Jolt Y (up)
 *
 * Red goal (zSign=+1): corner at Jolt (-HF, 0, +HF), triangle offset from +Z wall by CRAMP_WIDTH.
 * Scoring region: deltaX = JoltX + HF, deltaZ = HF - JoltZ - CRAMP_WIDTH,
 *                 inside when deltaX >= 0, deltaZ >= 0, deltaX + deltaZ <= GOAL_LEG.
 */
class GoalScoringTest {
    private lateinit var sim: JoltSimIO

    @BeforeEach
    fun setUp() {
        sim = JoltSimIO()
    }

    @AfterEach
    fun tearDown() {
        sim.close()
    }

    companion object {
        const val HALF_FIELD = 3.6576f / 2f   // 1.8288
        const val GOAL_LEG = 0.6858f
        const val CRAMP_WIDTH = 0.16f
        const val CRAMP_LENGTH = 1.00f
        const val CRAMP_START_H = 0.49f
        const val CRAMP_END_H = 0.127f
        const val GOAL_LIP_HEIGHT = 0.9843f
        const val BALL_RADIUS = 0.0635f
    }

    @Test
    fun testBallInsideRedGoalScores() {
        // Place a ball inside the red goal triangle, below lip height.
        // In sim coords: x ≈ HF - CRAMP_WIDTH - 0.2, y ≈ HF - 0.2, z = 0.3
        // This gives deltaX = 0.2, deltaZ = 0.2, sum = 0.4 < GOAL_LEG ✓
        val ballX = HALF_FIELD - CRAMP_WIDTH - 0.2f
        val ballY = HALF_FIELD - 0.2f
        val ballZ = 0.3f

        sim.spawnBall(ballX, ballY, ballZ, BallColor.GREEN)
        assertEquals(1, sim.getBallCount())

        // Step physics — ball should be scored and removed
        repeat(20) { sim.update() }

        val goalState = sim.getGoalState()
        assertEquals(1, goalState.redScore, "Ball inside red goal should be scored")
        assertEquals(0, sim.getBallCount(), "Scored ball should be removed from world")
    }

    @Test
    fun testBallInsideBlueGoalScores() {
        // Blue goal (zSign=-1): corner at Jolt (-HF, 0, -HF)
        // In sim coords: x ≈ -(HF - CRAMP_WIDTH - 0.2), y ≈ HF - 0.2
        val ballX = -(HALF_FIELD - CRAMP_WIDTH - 0.2f)
        val ballY = HALF_FIELD - 0.2f
        val ballZ = 0.3f

        sim.spawnBall(ballX, ballY, ballZ, BallColor.PURPLE)
        assertEquals(1, sim.getBallCount())

        repeat(20) { sim.update() }

        val goalState = sim.getGoalState()
        assertEquals(1, goalState.blueScore, "Ball inside blue goal should be scored")
        assertEquals(0, sim.getBallCount(), "Scored ball should be removed from world")
    }

    @Test
    fun testBallDroppedIntoGoalScores() {
        // Drop a ball from above the lip height, close to the corner (away from the
        // front wall diagonal so it doesn't land on the lip).
        // deltaX=0.05, deltaZ=0.05 → well inside triangle, far from diagonal.
        val ballX = HALF_FIELD - CRAMP_WIDTH - 0.05f  // sim x (near corner in Z)
        val ballY = HALF_FIELD - 0.05f                  // sim y (near corner in X)
        val ballZ = GOAL_LIP_HEIGHT + 0.1f              // above lip, below total height

        sim.spawnBall(ballX, ballY, ballZ, BallColor.GREEN)
        assertEquals(1, sim.getBallCount())

        // Let it fall through the open top and get scored
        repeat(400) { sim.update() } // 2 seconds

        val goalState = sim.getGoalState()
        assertTrue(goalState.redScore >= 1,
            "Ball dropped from above lip near corner should fall into goal and score, " +
            "redScore=${goalState.redScore}")
    }

    @Test
    fun testMultipleBallsScoreIndependently() {
        // Place 3 balls inside the red goal
        for (i in 0 until 3) {
            val offset = 0.1f + i * 0.1f
            sim.spawnBall(
                HALF_FIELD - CRAMP_WIDTH - offset,
                HALF_FIELD - offset,
                0.3f + i * 0.05f,
                if (i % 2 == 0) BallColor.GREEN else BallColor.PURPLE
            )
        }
        assertEquals(3, sim.getBallCount())

        repeat(40) { sim.update() }

        val goalState = sim.getGoalState()
        assertEquals(3, goalState.redScore, "All 3 balls should score in red goal")
        assertEquals(0, sim.getBallCount(), "All scored balls should be removed")
    }

    @Test
    fun testBallOutsideGoalDoesNotScore() {
        // Place a ball at field center — far from any goal
        sim.spawnBall(0f, 0f, BALL_RADIUS, BallColor.GREEN)
        assertEquals(1, sim.getBallCount())

        repeat(40) { sim.update() }

        val goalState = sim.getGoalState()
        assertEquals(0, goalState.redScore, "Ball at center should not score in red")
        assertEquals(0, goalState.blueScore, "Ball at center should not score in blue")
        assertEquals(1, sim.getBallCount(), "Ball should still exist")
    }

    @Test
    fun testBallOnRampRollsTowardGate() {
        // Place a ball at the ramp entrance (high end, near goal exit).
        // Ramp entrance in Jolt: X = rampStartX = -HF + GOAL_LEG, Z = rampZ = HF - CRAMP_WIDTH/2
        // sim: x = rampZ, y = -rampStartX
        val rampStartJoltX = -HALF_FIELD + GOAL_LEG
        val rampZ = HALF_FIELD - CRAMP_WIDTH / 2f

        val ballSimX = rampZ                               // sim x = Jolt Z
        val ballSimY = -rampStartJoltX                     // sim y = -Jolt X
        val ballSimZ = CRAMP_START_H + BALL_RADIUS + 0.02f // just above ramp surface

        sim.spawnBall(ballSimX, ballSimY, ballSimZ, BallColor.GREEN)
        val initialBalls = sim.getBallStates()
        assertTrue(initialBalls.isNotEmpty(), "Ball should be spawned")
        val initialY = initialBalls[0].y

        // Step physics for 2 seconds — ball should roll down the slope toward the gate
        repeat(400) { sim.update() }

        val balls = sim.getBallStates()
        assertTrue(balls.isNotEmpty(), "Ball should still exist (gate blocks it)")

        val finalY = balls[0].y
        // Ramp slopes from rampStartX to rampEndX (Jolt +X direction = sim -Y direction)
        assertTrue(finalY < initialY - 0.1f,
            "Ball should roll down ramp toward gate. initialY=$initialY, finalY=$finalY")
    }

    @Test
    fun testBallOnRampReachesGateEnd() {
        // Place a ball on the ramp and let it roll all the way to the gate.
        val rampStartJoltX = -HALF_FIELD + GOAL_LEG
        val rampZ = HALF_FIELD - CRAMP_WIDTH / 2f
        val rampEndJoltX = rampStartJoltX + CRAMP_LENGTH

        val ballSimX = rampZ
        val ballSimY = -rampStartJoltX
        val ballSimZ = CRAMP_START_H + BALL_RADIUS + 0.02f

        sim.spawnBall(ballSimX, ballSimY, ballSimZ, BallColor.GREEN)

        // Step for 5 seconds — enough time for ball to roll the full ramp length
        repeat(1000) { sim.update() }

        val balls = sim.getBallStates()
        assertTrue(balls.isNotEmpty(), "Ball should still exist (blocked by gate)")

        // Gate is at rampEndX in Jolt → sim_y = -rampEndJoltX
        val gateSimY = -rampEndJoltX
        val finalY = balls[0].y
        assertTrue(finalY < gateSimY + 0.2f,
            "Ball should have rolled near the gate end. finalY=$finalY, gateSimY=$gateSimY")

        // Ball height should have dropped close to CRAMP_END_H
        val finalZ = balls[0].z
        assertTrue(finalZ < CRAMP_START_H,
            "Ball should be lower than ramp start height. finalZ=$finalZ")
    }
}
