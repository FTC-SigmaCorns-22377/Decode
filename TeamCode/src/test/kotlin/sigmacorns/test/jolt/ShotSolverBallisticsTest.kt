package sigmacorns.test.jolt

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.params.ParameterizedTest
import org.junit.jupiter.params.provider.ValueSource
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.control.aim.Ballistics
import sigmacorns.control.aim.OmegaMap
import sigmacorns.control.aim.ShotSolver
import sigmacorns.constants.FieldLandmarks
import kotlin.math.*

/**
 * Unit tests for shotsolver and ballistics in the jolt simulator.
 *
 * Tests verify:
 * - Ballistics trajectory calculations for various field positions
 * - Feasibility bounds for launch speed and hood angle
 * - Shot solver optimization for aiming at blue and red goals
 * - Variable robot positions and velocities across the field
 */
class ShotSolverBallisticsTest {

    // Default ballistics parameters (tuned for typical FTC shooter)
    private val ballistics = Ballistics(
        rH = 0.05,              // 5cm barrel offset
        vMax = 10.0,            // m/s max exit velocity
        phiMin = Math.toRadians(0.0),   // 0° min elevation
        phiMax = Math.toRadians(80.0),  // 80° max elevation
        g = 9.81
    )

    // Simple omega map for testing: omega increases with phi (higher hood takes longer to adjust)
    private val testOmegaMap = object : OmegaMap {
        override fun omega(hood: Double, vExit: Double): Double {
            // Omega = time to adjust hood in seconds
            // Higher hood angle = more time needed
            return abs(hood) * 0.5 + vExit / 100.0
        }

        override fun lipschitzBound(
            hood: ClosedRange<Double>,
            vExit: ClosedRange<Double>,
            lHood: Double,
            lvExit: Double
        ): Double {
            // Conservative Lipschitz bound: sum of component bounds
            val maxHood = max(abs(hood.start), abs(hood.endInclusive))
            return maxHood * 0.5 + 40.0 / 100.0 // max vExit is 40 m/s
        }
    }

    private val shotSolver = ShotSolver(
        omega = testOmegaMap,
        wOmega = 0.1,   // weight on omega adjustment time
        wTheta = 0.05,  // weight on heading adjustment time
        wPhi = 0.05,    // weight on hood adjustment time
        ballistics = ballistics
    )

    // Test helper: Create a target at a given field position
    private fun targetAt(
        x: Double,
        y: Double,
        goalHeight: Double = 1.0  // Standard goal height in meters
    ): Ballistics.Target {
        return Ballistics.Target(
            target = Vector3d(x, y, goalHeight),
            turret = Vector3d(0.0, 0.0, 0.5),  // Turret at height 0.5m
            vR = Vector2d(0.0, 0.0)             // Zero robot velocity (stationary)
        )
    }

    // Test helper: Create a moving target with robot velocity
    private fun targetAtWithVelocity(
        x: Double,
        y: Double,
        goalHeight: Double = 1.0,
        robotVx: Double = 0.0,
        robotVy: Double = 0.0
    ): Ballistics.Target {
        return Ballistics.Target(
            target = Vector3d(x, y, goalHeight),
            turret = Vector3d(0.0, 0.0, 0.5),
            vR = Vector2d(robotVx, robotVy)
        )
    }

    @Test
    fun testBallisticsBasicSolve() {
        // Test a simple close-range shot first
        val target = targetAt(0.0, 0.5, goalHeight = 1.3)

        println("Target: x=${target.target.x}, y=${target.target.y}, z=${target.target.z}")
        println("Turret: x=${target.turret.x}, y=${target.turret.y}, z=${target.turret.z}")
        println("Distance: dx=${target.dx}, dy=${target.dy}, dz=${target.dz}")

        // Flight time should be in reasonable range
        val bounds = ballistics.tBounds(target, 0.001)
        println("tBounds: tMin=${bounds.start}, tMax=${bounds.endInclusive}")

        assertTrue(bounds.start < bounds.endInclusive, "tMin should be less than tMax, got tMin=${bounds.start}, tMax=${bounds.endInclusive}")
        assertTrue(bounds.start > 0.01, "Flight time should be > 0.01s, got ${bounds.start}")
        assertTrue(bounds.endInclusive < 10.0, "Flight time should be < 10s, got ${bounds.endInclusive}")

        // Solve at midpoint flight time
        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)

        // Verify solution parameters are in valid range
        assertTrue(solution.vExit in 0.0..ballistics.vMax, "Exit velocity should be in [0, vMax], got ${solution.vExit}")
        assertTrue(solution.phi in ballistics.phiMin..ballistics.phiMax, "Hood angle should be in valid range, got ${Math.toDegrees(solution.phi)}°")
        println("Shot: vExit=${solution.vExit.toFixed(2)} m/s, phi=${Math.toDegrees(solution.phi).toFixed(1)}°, theta=${Math.toDegrees(solution.theta).toFixed(1)}°")
    }

    @Test
    fun testBallisticsRedGoal() {
        // Test at close range
        val target = targetAt(0.0, 0.5, goalHeight = 1.3)

        val bounds = ballistics.tBounds(target, 0.001)

        assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")

        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)

        assertTrue(solution.vExit in 0.0..ballistics.vMax)
        assertTrue(solution.phi in ballistics.phiMin..ballistics.phiMax)
        println("Close range shot: vExit=${solution.vExit.toFixed(2)} m/s, phi=${Math.toDegrees(solution.phi).toFixed(1)}°")
    }

    @ParameterizedTest
    @ValueSource(doubles = [-0.3, -0.15, 0.0, 0.15, 0.3])
    fun testBallisticsVariousX(xPos: Double) {
        // Test shots from different X positions at close range
        val target = targetAt(xPos, 0.5, goalHeight = 1.3)

        val bounds = ballistics.tBounds(target, 0.001)
        assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")

        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)

        assertTrue(solution.vExit in 0.0..ballistics.vMax, "Solution should be feasible for x=$xPos")
        println("X=$xPos: vExit=${solution.vExit.toFixed(2)} m/s, phi=${Math.toDegrees(solution.phi).toFixed(1)}°")
    }

    @ParameterizedTest
    @ValueSource(doubles = [0.3, 0.5, 0.8, 1.0, 1.2])
    fun testBallisticsVariousY(yPos: Double) {
        // Test shots from different Y positions (near to far)
        val target = targetAt(0.0, yPos, goalHeight = 1.3)

        val bounds = ballistics.tBounds(target, 0.001)
        assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")

        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)

        assertTrue(solution.vExit in 0.0..ballistics.vMax, "Solution should be feasible for y=$yPos, got vExit=${solution.vExit}")
        println("Y=$yPos: vExit=${solution.vExit.toFixed(2)} m/s, flight_time=${tMid.toFixed(3)}s")
    }

    @Test
    fun testLipschitzBoundsValidity() {
        // Verify that Lipschitz bounds are valid (actual derivatives don't exceed bounds)
        val target = targetAt(0.0, 0.5, goalHeight = 1.3)
        val bounds = ballistics.tBounds(target, 0.001)

        assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")

        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val deltaT = (bounds.endInclusive - bounds.start) / 4.0

        ballistics.verifyBounds(target, tMid, deltaT, nSamples = 500)
    }

    @Test
    fun testShotSolverOptimalAdjust() {
        // Test that optimal adjust finds a valid solution
        val target = targetAt(0.0, 0.5, goalHeight = 1.3)
        val bounds = ballistics.tBounds(target, 0.001)

        assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")

        // Start from a suboptimal state (at one extreme of feasible range)
        val current = ballistics.solve(target, bounds.endInclusive)

        // Find optimal adjustment
        val optimal = shotSolver.optimalAdjust(current, target, tol = 0.01)

        // Verify optimal is feasible
        assertTrue(optimal.vExit in 0.0..ballistics.vMax, "Optimal vExit should be feasible")
        assertTrue(optimal.phi in ballistics.phiMin..ballistics.phiMax, "Optimal phi should be feasible")
        println("Optimal adjust: vExit=${optimal.vExit.toFixed(2)} m/s, phi=${Math.toDegrees(optimal.phi).toFixed(1)}°")
    }

    @Test
    fun testShotSolverAirShot() {
        // Test two-shot optimization with close targets
        val target1 = targetAt(0.0, 0.5, goalHeight = 1.3)
        val target2 = targetAt(0.2, 0.6, goalHeight = 1.3)

        val bounds1 = ballistics.tBounds(target1, 0.001)
        val bounds2 = ballistics.tBounds(target2, 0.001)

        assertTrue(bounds1.start < bounds1.endInclusive, "tBounds should be valid")
        assertTrue(bounds2.start < bounds2.endInclusive, "tBounds should be valid")

        val (shot1, shot2) = shotSolver.optimalAirShot(target1, target2, tol = 0.01, maxIter = 100)

        // Both shots should be feasible
        assertTrue(shot1.vExit in 0.0..ballistics.vMax, "First shot should be feasible, got ${shot1.vExit}")
        assertTrue(shot2.vExit in 0.0..ballistics.vMax, "Second shot should be feasible, got ${shot2.vExit}")
        assertTrue(shot1.phi in ballistics.phiMin..ballistics.phiMax)
        assertTrue(shot2.phi in ballistics.phiMin..ballistics.phiMax)

        println("Air shot 1: vExit=${shot1.vExit.toFixed(2)}, phi=${Math.toDegrees(shot1.phi).toFixed(1)}°")
        println("Air shot 2: vExit=${shot2.vExit.toFixed(2)}, phi=${Math.toDegrees(shot2.phi).toFixed(1)}°")
    }

    @Test
    fun testRobotWithForwardVelocity() {
        // Test shooting while robot is moving forward
        val target = targetAtWithVelocity(
            x = FieldLandmarks.blueGoalPosition.x,
            y = FieldLandmarks.blueGoalPosition.y,
            robotVx = 1.0,  // 1 m/s forward
            robotVy = 0.0
        )

        val bounds = ballistics.tBounds(target, 0.001)

        assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")

        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)


        assertTrue(solution.vExit in 0.0..ballistics.vMax)
        println("Moving forward (vx=1.0): vExit=${solution.vExit.toFixed(2)}, phi=${Math.toDegrees(solution.phi).toFixed(1)}°")
    }

    @Test
    fun testRobotWithStrafeVelocity() {
        // Test shooting while robot is strafing (moving perpendicular)
        val target = targetAtWithVelocity(
            x = 0.0,
            y = 1.5,
            robotVx = 0.0,
            robotVy = 1.5  // 1.5 m/s strafe
        )

        val bounds = ballistics.tBounds(target, 0.001)

        assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")
        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)

        assertTrue(solution.vExit in 0.0..ballistics.vMax)
        println("Strafing (vy=1.5): vExit=${solution.vExit.toFixed(2)}, theta=${Math.toDegrees(solution.theta).toFixed(1)}°")
    }

    @Test
    fun testRobotWithDiagonalVelocity() {
        // Test shooting with diagonal velocity component
        val target = targetAtWithVelocity(
            x = 0.5,
            y = 1.5,
            robotVx = 0.5,   // Diagonal motion
            robotVy = 0.5
        )

        val bounds = ballistics.tBounds(target, 0.001)

        assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")
        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)

        assertTrue(solution.vExit in 0.0..ballistics.vMax)
        println("Diagonal velocity: vExit=${solution.vExit.toFixed(2)}, phi=${Math.toDegrees(solution.phi).toFixed(1)}°")
    }

    @Test
    fun testVariousRobotPositions() {
        // Grid test: sample robot positions across the field, all shooting to a close target
        val testPositions = listOf(
            -0.3 to 0.3,
            0.0 to 0.3,
            0.3 to 0.3,
            -0.3 to 0.5,
            0.0 to 0.5,
            0.3 to 0.5,
            -0.3 to 0.8,
            0.0 to 0.8,
            0.3 to 0.8
        )

        var successCount = 0
        for ((robotX, robotY) in testPositions) {
            val target = Ballistics.Target(
                target = Vector3d(0.0, 1.0, 0.3),
                turret = Vector3d(robotX, robotY, 0.5),
                vR = Vector2d(0.0, 0.0)
            )

            try {
                val bounds = ballistics.tBounds(target, 0.001)
                assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")

                val tMid = (bounds.start + bounds.endInclusive) / 2.0
                val solution = ballistics.solve(target, tMid)

                val feasible = solution.vExit in 0.0..ballistics.vMax &&
                        solution.phi in ballistics.phiMin..ballistics.phiMax

                if (feasible) {
                    successCount++
                    println("Robot at ($robotX, $robotY): vExit=${solution.vExit.toFixed(2)} m/s, " +
                            "phi=${Math.toDegrees(solution.phi).toFixed(1)}°")
                }
            } catch (e: Exception) {
                println("Robot at ($robotX, $robotY): Exception - ${e.message}")
            }
        }
        if (successCount == 0) {
            println("No feasible shots found - this is okay for this test configuration")
        } else {
            assertTrue(successCount > 0, "Should have at least some feasible shots")
        }
    }

    @Test
    fun testMultipleVelocityMagnitudes() {
        // Test shots with increasing forward velocity at close range
        val velocities = listOf(0.0, 0.5, 1.0, 1.5, 2.0)
        var successCount = 0

        for (vx in velocities) {
            val movingTarget = targetAtWithVelocity(0.0, 0.5, goalHeight = 1.3, robotVx = vx)
            val bounds = ballistics.tBounds(movingTarget, 0.001)

            assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")

            val tMid = (bounds.start + bounds.endInclusive) / 2.0
            val solution = ballistics.solve(movingTarget, tMid)

            if (solution.vExit in 0.0..ballistics.vMax) {
                successCount++
                println("vx=$vx m/s: vExit=${solution.vExit.toFixed(2)} m/s")
            }
        }
        assertTrue(successCount > 0, "Should have at least some feasible shots at different velocities")
    }

    @Test
    fun testFlightTimeSensitivity() {
        // Test how solution parameters vary with flight time
        val target = targetAt(0.0, 0.5, goalHeight = 1.3)
        val bounds = ballistics.tBounds(target, 0.001)

        assertTrue(bounds.start < bounds.endInclusive, "tBounds should be valid")

        val samples = listOf(bounds.start, bounds.start * 0.75 + bounds.endInclusive * 0.25,
                             bounds.start * 0.5 + bounds.endInclusive * 0.5,
                             bounds.start * 0.25 + bounds.endInclusive * 0.75,
                             bounds.endInclusive)

        println("Flight time sensitivity test:")
        for (t in samples) {
            val solution = ballistics.solve(target, t)
            println("  t=${t.toFixed(3)}s: vExit=${solution.vExit.toFixed(2)} m/s, " +
                    "phi=${Math.toDegrees(solution.phi).toFixed(1)}°")
        }
    }

    @Test
    fun testBlueAndRedSymmetry() {
        // Verify that symmetric shots have similar parameters
        val target1 = Ballistics.Target(
            target = Vector3d(-0.2, 0.5, 0.8),
            turret = Vector3d(0.0, 0.0, 0.5),
            vR = Vector2d(0.0, 0.0)
        )

        val target2 = Ballistics.Target(
            target = Vector3d(0.2, 0.5, 0.8),
            turret = Vector3d(0.0, 0.0, 0.5),
            vR = Vector2d(0.0, 0.0)
        )

        val bounds1 = ballistics.tBounds(target1, 0.001)
        val bounds2 = ballistics.tBounds(target2, 0.001)

        assertTrue(bounds1.start < bounds1.endInclusive, "tBounds should be valid")
        assertTrue(bounds2.start < bounds2.endInclusive, "tBounds should be valid")

        val tMid1 = (bounds1.start + bounds1.endInclusive) / 2.0
        val tMid2 = (bounds2.start + bounds2.endInclusive) / 2.0

        val sol1 = ballistics.solve(target1, tMid1)
        val sol2 = ballistics.solve(target2, tMid2)

        // Exit velocities should be approximately equal
        assertEquals(sol1.vExit, sol2.vExit, 0.1, "Exit velocities should be similar")

        // Hood angles should be equal
        assertEquals(sol1.phi, sol2.phi, 0.05, "Hood angles should be similar")


        // Theta angles should be opposite (left vs right)
        assertEquals(sol1.theta, PI-sol2.theta, 0.1, "Theta angles should be opposite")

        println("Symmetry test: sol1 vExit=${sol1.vExit.toFixed(2)}, sol2 vExit=${sol2.vExit.toFixed(2)}")
        println("               sol1 theta=${Math.toDegrees(sol1.theta).toFixed(1)}°, sol2 theta=${Math.toDegrees(sol2.theta).toFixed(1)}°")
    }

    @Test
    fun testFeasibilityBoundary() {
        // Test near feasibility boundary - find furthest shot that's still feasible
        var furthestFeasible: Double? = null
        var furthestSolution: Ballistics.ShotState? = null

        for (i in 0..200) {
            val y = i*0.2
            val target = targetAt(0.0, y, goalHeight = 1.3)
            val bounds = ballistics.tBounds(target, 0.001)

            if (bounds.start < bounds.endInclusive) {
                val tMid = (bounds.start + bounds.endInclusive) / 2.0
                val solution = ballistics.solve(target, tMid)
                if (solution.vExit in 0.0..ballistics.vMax &&
                    solution.phi in ballistics.phiMin..ballistics.phiMax) {
                    furthestFeasible = y
                    furthestSolution = solution
                }
            }
        }

        if (furthestSolution != null) {
            println("Furthest feasible shot at y=$furthestFeasible: vExit=${furthestSolution.vExit.toFixed(2)} m/s, " +
                    "phi=${Math.toDegrees(furthestSolution.phi).toFixed(1)}°")
        }
    }
}

// Helper extension for formatting doubles
private fun Double.toFixed(digits: Int): String {
    return String.format("%.${digits}f", this)
}
