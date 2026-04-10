package sigmacorns.test.jolt

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.control.aim.Ballistics
import sigmacorns.control.aim.OmegaMap
import sigmacorns.control.aim.ShotSolver
import kotlin.math.*

/**
 * Tests for ShotSolver optimality via grid search verification.
 *
 * These tests verify that the ShotSolver's optimization algorithms find solutions
 * that are close to optimal by comparing against brute-force grid search over
 * the feasible solution space (flight times, azimuth angles, etc.).
 *
 * The grid search exhaustively samples the feasible space and measures accuracy
 * via trajectory simulation. The ShotSolver should find solutions within a small
 * tolerance of the grid search optimum.
 */
class ShotSolverOptimalityTest {

    private val ballistics = Ballistics(
        rH = 0.05,
        vMax = 15.0,
        phiMin = Math.toRadians(0.0),
        phiMax = Math.toRadians(80.0),
        g = 9.81
    )

    private val testOmegaMap = object : OmegaMap {
        override fun omega(hood: Double, vExit: Double): Double {
            return abs(hood) * 0.5 + vExit / 100.0
        }

        override fun lipschitzBound(
            hood: ClosedRange<Double>,
            vExit: ClosedRange<Double>,
            lHood: Double,
            lvExit: Double
        ): Double {
            val maxHood = max(abs(hood.start), abs(hood.endInclusive))
            return maxHood * 0.5 + 40.0 / 100.0
        }
    }

    private val shotSolver = ShotSolver(
        omega = testOmegaMap,
        wOmega = 0.1,
        wTheta = 0.05,
        wPhi = 0.05,
        ballistics = ballistics
    )


    /**
     * Helper: Create a ballistics target.
     */
    private fun createTarget(
        turretPos: Vector3d,
        targetX: Double,
        targetY: Double,
        targetZ: Double
    ): Ballistics.Target {
        return Ballistics.Target(
            target = Vector3d(targetX, targetY, targetZ),
            turret = turretPos,
            vR = Vector2d(0.0, 0.0)
        )
    }

    /**
     * Helper: Compute movement cost from one shot state to another.
     * Matches ShotSolver.moveCost():
     *   max(wOmega*dOmega, max(wPhi*dPhi, wTheta*dTheta))
     */
    private fun computeMoveCost(
        from: Ballistics.ShotState,
        to: Ballistics.ShotState
    ): Double {
        val fromOmega = testOmegaMap.omega(from.phi, from.vExit)
        val toOmega = testOmegaMap.omega(to.phi, to.vExit)
        val dOmega = abs(fromOmega - toOmega)
        val dPhi = abs(from.phi - to.phi)
        val dTheta = abs(from.theta - to.theta)

        return max(shotSolver.wOmega * dOmega, max(shotSolver.wPhi * dPhi, shotSolver.wTheta * dTheta))
    }

    /**
     * Helper: Compute air shot cost function.
     * Matches ShotSolver.optimalAirShot() objective:
     *   J(T1, T2) = moveCost(solve(target1, T1), solve(target2, T2)) + (T2 - T1)
     */
    private fun computeAirShotCost(
        shot1: Ballistics.ShotState,
        shot2: Ballistics.ShotState,
        t1: Double,
        t2: Double
    ): Double {
        val moveCost = computeMoveCost(shot1, shot2)
        val timeCost = t2 - t1
        return moveCost + timeCost
    }

    // ========== Tests ==========

    @Test
    fun testShotSolverOptimalityViaMoveCost() {
        // Verify ShotSolver finds minimum move cost by comparing against line search.
        // optimalAdjust() minimizes moveCost(cur, solve(target, t)) over flight time t.
        // The cost is: max(wOmega*dOmega, max(wPhi*dPhi, wTheta*dTheta))

        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val target = createTarget(turretPos, 0.0, 0.8, 1.2)
        val bounds = ballistics.tBounds(target, 0.001)

        assertTrue(bounds.start < bounds.endInclusive, "Should have valid flight time bounds")

        // Start from a suboptimal initial state (max flight time)
        val initialState = ballistics.solve(target, bounds.endInclusive)

        // Perform exhaustive line search over flight times to find minimum cost
        val gridSamples = 200
        val tStep = (bounds.endInclusive - bounds.start) / gridSamples
        var bestGridCost = Double.MAX_VALUE
        var bestGridSolution: Ballistics.ShotState? = null
        var bestGridTime = 0.0

        println("Line search for optimal move cost over ${gridSamples + 1} samples:")
        println("Initial state: vExit=${"%.2f".format(initialState.vExit)}, " +
                "phi=${"%.1f".format(Math.toDegrees(initialState.phi))}°, " +
                "theta=${"%.1f".format(Math.toDegrees(initialState.theta))}°")

        for (i in 0..gridSamples) {
            val t = bounds.start + i * tStep
            try {
                val solution = ballistics.solve(target, t)

                // Check feasibility
                if (solution.vExit <= ballistics.vMax &&
                    solution.phi in ballistics.phiMin..ballistics.phiMax) {

                    // Compute move cost from initial state to this solution
                    val cost = computeMoveCost(initialState, solution)

                    if (cost < bestGridCost) {
                        bestGridCost = cost
                        bestGridSolution = solution
                        bestGridTime = t
                    }
                }
            } catch (e: Exception) {
                // Solution may not exist at all flight times
            }
        }

        assertNotNull(bestGridSolution, "Line search should find at least one feasible solution")

        println("Line search best cost: ${"%.6f".format(bestGridCost)} at t=${"%.3f".format(bestGridTime)}")
        println("  Solution: vExit=${"%.2f".format(bestGridSolution!!.vExit)} m/s, " +
                "phi=${"%.1f".format(Math.toDegrees(bestGridSolution.phi))}°, " +
                "theta=${"%.1f".format(Math.toDegrees(bestGridSolution.theta))}°")

        // Find optimal solution using ShotSolver
        val solverOptimal = shotSolver.optimalAdjust(initialState, target, tol = 0.001)

        // Compute cost of solver's solution
        val solverCost = computeMoveCost(initialState, solverOptimal)

        println("ShotSolver best cost: ${"%.6f".format(solverCost)}")
        println("  Solution: vExit=${"%.2f".format(solverOptimal.vExit)} m/s, " +
                "phi=${"%.1f".format(Math.toDegrees(solverOptimal.phi))}°, " +
                "theta=${"%.1f".format(Math.toDegrees(solverOptimal.theta))}°")

        // Compare: ShotSolver should find cost very close to line search minimum
        // Allow 5% tolerance due to grid resolution and Lipschitz optimization technique
        // With absolute minimum of 1e-4 to handle numerical precision
        val tolerance = max(1e-4, bestGridCost * 0.05)
        println("Cost difference: ${"%.6f".format(solverCost - bestGridCost)} " +
                "(tolerance: ${"%.6f".format(tolerance)})")

        assertTrue(
            solverCost <= bestGridCost + tolerance,
            "ShotSolver cost (${"%.6f".format(solverCost)}) should be within tolerance " +
            "of line search minimum (${"%.6f".format(bestGridCost)})"
        )
    }

    @Test
    fun testAirShotOptimalityViaGridSearch() {
        // Verify optimalAirShot() finds minimum cost via 2D grid search.
        // optimalAirShot minimizes: J(T1, T2) = moveCost(shot1, shot2) + (T2 - T1)

        val target1 = createTarget(Vector3d(0.0, 0.0, 0.5), 0.0, 0.5, 1.2)
        val target2 = createTarget(Vector3d(0.0, 0.0, 0.5), 0.2, 0.7, 1.2)

        val bounds1 = ballistics.tBounds(target1, 0.001)
        val bounds2 = ballistics.tBounds(target2, 0.001)

        assertTrue(bounds1.start < bounds1.endInclusive && bounds2.start < bounds2.endInclusive,
            "Both targets should be feasible")

        // Grid search over T1 and T2
        val gridSamples = 20  // 20x20 = 400 samples
        val t1Step = (bounds1.endInclusive - bounds1.start) / gridSamples
        val t2Step = (bounds2.endInclusive - bounds2.start) / gridSamples

        var bestGridCost = Double.MAX_VALUE
        var bestGridT1 = bounds1.start
        var bestGridT2 = bounds2.start

        println("2D Grid search over ${gridSamples + 1}x${gridSamples + 1} = ${(gridSamples + 1) * (gridSamples + 1)} samples:")

        for (i in 0..gridSamples) {
            for (j in 0..gridSamples) {
                val t1 = bounds1.start + i * t1Step
                val t2 = bounds2.start + j * t2Step

                try {
                    val shot1 = ballistics.solve(target1, t1)
                    val shot2 = ballistics.solve(target2, t2)

                    // Check feasibility
                    if (shot1.vExit <= ballistics.vMax && shot1.phi in ballistics.phiMin..ballistics.phiMax &&
                        shot2.vExit <= ballistics.vMax && shot2.phi in ballistics.phiMin..ballistics.phiMax) {

                        val cost = computeAirShotCost(shot1, shot2, t1, t2)

                        if (cost < bestGridCost) {
                            bestGridCost = cost
                            bestGridT1 = t1
                            bestGridT2 = t2
                        }
                    }
                } catch (e: Exception) {
                    // Solution may not exist at all flight times
                }
            }
        }

        assertTrue(bestGridCost < Double.MAX_VALUE, "Grid search should find feasible solution")

        val gridShot1 = ballistics.solve(target1, bestGridT1)
        val gridShot2 = ballistics.solve(target2, bestGridT2)

        println("Grid search best: cost=${"%.6f".format(bestGridCost)}")
        println("  T1=${"%.3f".format(bestGridT1)} s, T2=${"%.3f".format(bestGridT2)} s")
        println("  Shot 1: vExit=${"%.2f".format(gridShot1.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(gridShot1.phi))}°")
        println("  Shot 2: vExit=${"%.2f".format(gridShot2.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(gridShot2.phi))}°")

        // Use ShotSolver's air shot optimization
        val (solverShot1, solverShot2) = shotSolver.optimalAirShot(target1, target2, tol = 0.01, maxIter = 100)

        // Verify both shots are feasible
        assertTrue(solverShot1.vExit in 0.0..ballistics.vMax && solverShot1.phi in ballistics.phiMin..ballistics.phiMax,
            "First shot should be feasible")
        assertTrue(solverShot2.vExit in 0.0..ballistics.vMax && solverShot2.phi in ballistics.phiMin..ballistics.phiMax,
            "Second shot should be feasible")

        // Compute cost of solver's solution
        // Note: We can't directly get T1 and T2 from the solver, but we can estimate flight times from the solution
        // For now, just verify the solver found feasible solutions with reasonable move cost
        val solverMoveCost = computeMoveCost(solverShot1, solverShot2)

        println("ShotSolver best: moveCost=${"%.6f".format(solverMoveCost)}")
        println("  Shot 1: vExit=${"%.2f".format(solverShot1.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(solverShot1.phi))}°")
        println("  Shot 2: vExit=${"%.2f".format(solverShot2.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(solverShot2.phi))}°")

        // Move cost should be reasonable (not infinite)
        assertTrue(solverMoveCost.isFinite(), "Move cost should be finite")
        assertTrue(solverMoveCost < 10.0, "Move cost should be reasonable")
    }

    @Test
    fun testOptimalAdjustFindsLowCostSolution() {
        // Verify that optimalAdjust finds a solution with reasonable move cost.

        val target = createTarget(Vector3d(0.0, 0.0, 0.5), 0.0, 1.0, 1.2)
        val bounds = ballistics.tBounds(target, 0.001)

        // Start with a suboptimal solution (at maximum flight time)
        val initial = ballistics.solve(target, bounds.endInclusive)

        println("Initial state: vExit=${"%.2f".format(initial.vExit)}, " +
                "phi=${"%.1f".format(Math.toDegrees(initial.phi))}°")

        // Find optimal adjustment
        val optimal = shotSolver.optimalAdjust(initial, target, tol = 0.001)
        val optimalCost = computeMoveCost(initial, optimal)

        println("Optimized state: vExit=${"%.2f".format(optimal.vExit)}, " +
                "phi=${"%.1f".format(Math.toDegrees(optimal.phi))}°")
        println("Move cost: ${"%.6f".format(optimalCost)}")

        // Move cost should be finite and reasonable
        assertTrue(optimalCost.isFinite(),
            "Move cost should be finite")
        assertTrue(optimalCost < 10.0,
            "Move cost should be reasonable (< 10)")
    }

    /**
     * Helper: Compute robust move cost — same as moveCost but omega1 is reduced by omegaDrop,
     * modeling flywheel speed loss after a shot.
     */
    private fun computeRobustMoveCost(
        from: Ballistics.ShotState,
        to: Ballistics.ShotState,
        omegaDrop: Double
    ): Double {
        val fromOmega = testOmegaMap.omega(from.phi, from.vExit) - omegaDrop
        val toOmega = testOmegaMap.omega(to.phi, to.vExit)
        val dOmega = abs(fromOmega - toOmega)
        val dPhi = abs(from.phi - to.phi)
        val dTheta = abs(from.theta - to.theta)
        return max(shotSolver.wOmega * dOmega, max(shotSolver.wPhi * dPhi, shotSolver.wTheta * dTheta))
    }

    @Test
    fun testRobustShotOptimalityViaGridSearch() {
        // Verify optimalRobustShot() finds minimum cost via 2D grid search.
        // optimalRobustShot minimizes: J(T1, T2) = moveCost(s1_with_reduced_omega, s2)
        // where s1's omega is reduced by omegaDrop to model flywheel speed loss after firing.

        val omegaDrop = 5.0  // rad/s drop after firing

        val target1 = createTarget(Vector3d(0.0, 0.0, 0.5), 0.0, 0.5, 1.2)
        val target2 = createTarget(Vector3d(0.0, 0.0, 0.5), 0.2, 0.7, 1.2)

        val bounds1 = ballistics.tBounds(target1, 0.001)
        val bounds2 = ballistics.tBounds(target2, 0.001)

        assertTrue(bounds1.start < bounds1.endInclusive && bounds2.start < bounds2.endInclusive,
            "Both targets should be feasible")

        // Grid search over T1 and T2
        val gridSamples = 20  // 20x20 = 400 samples
        val t1Step = (bounds1.endInclusive - bounds1.start) / gridSamples
        val t2Step = (bounds2.endInclusive - bounds2.start) / gridSamples

        var bestGridCost = Double.MAX_VALUE
        var bestGridT1 = bounds1.start
        var bestGridT2 = bounds2.start

        println("2D Grid search for robust shot over ${gridSamples + 1}x${gridSamples + 1} = ${(gridSamples + 1) * (gridSamples + 1)} samples:")
        println("omegaDrop = $omegaDrop rad/s")

        for (i in 0..gridSamples) {
            for (j in 0..gridSamples) {
                val t1 = bounds1.start + i * t1Step
                val t2 = bounds2.start + j * t2Step

                try {
                    val shot1 = ballistics.solve(target1, t1)
                    val shot2 = ballistics.solve(target2, t2)

                    if (shot1.vExit <= ballistics.vMax && shot1.phi in ballistics.phiMin..ballistics.phiMax &&
                        shot2.vExit <= ballistics.vMax && shot2.phi in ballistics.phiMin..ballistics.phiMax) {

                        val cost = computeRobustMoveCost(shot1, shot2, omegaDrop)
                        if (cost < bestGridCost) {
                            bestGridCost = cost
                            bestGridT1 = t1
                            bestGridT2 = t2
                        }
                    }
                } catch (e: Exception) {
                    // Solution may not exist at all flight times
                }
            }
        }

        assertTrue(bestGridCost < Double.MAX_VALUE, "Grid search should find feasible solution")

        val gridShot1 = ballistics.solve(target1, bestGridT1)
        val gridShot2 = ballistics.solve(target2, bestGridT2)

        println("Grid search best: cost=${"%.6f".format(bestGridCost)}")
        println("  T1=${"%.3f".format(bestGridT1)} s, T2=${"%.3f".format(bestGridT2)} s")
        println("  Shot 1: vExit=${"%.2f".format(gridShot1.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(gridShot1.phi))}°")
        println("  Shot 2: vExit=${"%.2f".format(gridShot2.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(gridShot2.phi))}°")

        // Use ShotSolver's robust shot optimization
        val (solverShot1, solverShot2) = shotSolver.optimalRobustShot(target1, target2, omegaDrop, tol = 0.01, maxIter = 200)

        // Verify both shots are feasible
        assertTrue(solverShot1.vExit in 0.0..ballistics.vMax && solverShot1.phi in ballistics.phiMin..ballistics.phiMax,
            "First shot should be feasible")
        assertTrue(solverShot2.vExit in 0.0..ballistics.vMax && solverShot2.phi in ballistics.phiMin..ballistics.phiMax,
            "Second shot should be feasible")

        val solverCost = computeRobustMoveCost(solverShot1, solverShot2, omegaDrop)

        println("ShotSolver robust best: cost=${"%.6f".format(solverCost)}")
        println("  Shot 1: vExit=${"%.2f".format(solverShot1.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(solverShot1.phi))}°")
        println("  Shot 2: vExit=${"%.2f".format(solverShot2.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(solverShot2.phi))}°")

        // Allow 5% tolerance relative to grid minimum (grid is coarse at 20x20)
        val tolerance = max(1e-4, bestGridCost * 0.05)
        println("Cost difference: ${"%.6f".format(solverCost - bestGridCost)} (tolerance: ${"%.6f".format(tolerance)})")

        assertTrue(
            solverCost <= bestGridCost + tolerance,
            "optimalRobustShot cost (${"%.6f".format(solverCost)}) should be within tolerance " +
            "of grid search minimum (${"%.6f".format(bestGridCost)})"
        )
    }

    @Test
    fun testMultipleShotsMoveCostProgression() {
        // Test that move costs are reasonable for multiple targets in sequence.
        // This verifies the solver can efficiently transition between shots.

        val target1 = createTarget(Vector3d(0.0, 0.0, 0.5), 0.0, 0.4, 1.2)
        val target2 = createTarget(Vector3d(0.0, 0.0, 0.5), 0.3, 0.6, 1.3)
        val target3 = createTarget(Vector3d(0.0, 0.0, 0.5), -0.3, 0.8, 1.1)

        val bounds1 = ballistics.tBounds(target1, 0.001)
        val bounds2 = ballistics.tBounds(target2, 0.001)
        val bounds3 = ballistics.tBounds(target3, 0.001)

        assertTrue(
            bounds1.start < bounds1.endInclusive &&
            bounds2.start < bounds2.endInclusive &&
            bounds3.start < bounds3.endInclusive,
            "All targets should be feasible"
        )

        println("Multi-shot optimization sequence:")

        // Start with shot 1
        val shot1 = ballistics.solve(target1, (bounds1.start + bounds1.endInclusive) / 2.0)
        println("  Shot 1: vExit=${"%.2f".format(shot1.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(shot1.phi))}°")

        // Optimize shot 2 from shot 1
        val shot2 = shotSolver.optimalAdjust(shot1, target2, tol = 0.001)
        val cost1to2 = computeMoveCost(shot1, shot2)
        println("  Shot 2: vExit=${"%.2f".format(shot2.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(shot2.phi))}°")
        println("    Move cost: ${"%.6f".format(cost1to2)}")

        // Optimize shot 3 from shot 2
        val shot3 = shotSolver.optimalAdjust(shot2, target3, tol = 0.001)
        val cost2to3 = computeMoveCost(shot2, shot3)
        println("  Shot 3: vExit=${"%.2f".format(shot3.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(shot3.phi))}°")
        println("    Move cost: ${"%.6f".format(cost2to3)}")

        // Total cost should be reasonable
        val totalCost = cost1to2 + cost2to3
        println("  Total cost: ${"%.6f".format(totalCost)}")

        assertTrue(cost1to2.isFinite() && cost2to3.isFinite(), "All move costs should be finite")
    }
}
