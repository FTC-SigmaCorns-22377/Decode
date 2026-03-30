package sigmacorns.control.aim.ballistic

import kotlin.math.sqrt

/**
 * Computes valid travel time bounds [T_min, T_max] for the ballistic solver.
 *
 * T_min: smallest T where the required exit velocity is achievable (omega <= omega_max).
 * T_max: largest T where the geometry is still valid (v_exit^2 >= 0).
 *
 * Uses bisection refinement on [ShotGeometry.solve] feasibility.
 * See docs/OPTIMIZED_SHOOT.md Section 5 for derivation.
 */
object TravelTimeBounds {

    private const val BISECTION_ITERATIONS = 20
    private const val T_MAX_SEARCH_LIMIT = 5.0 // seconds - physical upper limit

    /**
     * Compute the valid [T_min, T_max] range for a given shot state.
     *
     * @return valid range, or null if no feasible T exists
     */
    fun compute(state: ShotState, flywheelMap: FlywheelMap): ClosedRange<Double>? {
        val vMax = flywheelMap.maxExitVelocity()
        if (vMax <= 0.0) return null

        val dx = state.xTarget - state.xRobot
        val dy = state.yTarget - state.yRobot
        val horizontalDist = sqrt(dx * dx + dy * dy)

        // Loose lower bound: minimum time at max horizontal velocity
        val looseTMin = if (horizontalDist > 0.001) horizontalDist / vMax else 0.01

        // Find T_min by bisection: smallest T where omega <= omega_max
        val tMin = bisectFeasibleMin(state, flywheelMap, looseTMin, T_MAX_SEARCH_LIMIT)
            ?: return null

        // Find T_max by bisection: largest T where solution is still feasible
        val tMax = bisectFeasibleMax(state, flywheelMap, tMin, T_MAX_SEARCH_LIMIT)
            ?: return null

        if (tMin > tMax) return null
        return tMin..tMax
    }

    /**
     * Find the smallest feasible T via bisection.
     * At small T, required velocity is too high (omega > omega_max).
     * We find where feasibility first becomes true.
     */
    private fun bisectFeasibleMin(
        state: ShotState,
        flywheelMap: FlywheelMap,
        lo: Double,
        hi: Double
    ): Double? {
        // First check that there's a feasible point somewhere in the range
        var foundFeasible = false
        var searchHi = hi
        val steps = 50
        val dt = (searchHi - lo) / steps
        for (i in 0..steps) {
            val t = lo + i * dt
            val sol = ShotGeometry.solve(state, t, flywheelMap)
            if (sol != null && sol.feasible) {
                searchHi = t
                foundFeasible = true
                break
            }
        }
        if (!foundFeasible) return null

        // Bisect between lo and the first feasible point
        var a = lo
        var b = searchHi
        repeat(BISECTION_ITERATIONS) {
            val mid = (a + b) / 2.0
            val sol = ShotGeometry.solve(state, mid, flywheelMap)
            if (sol != null && sol.feasible) {
                b = mid
            } else {
                a = mid
            }
        }
        return b
    }

    /**
     * Find the largest feasible T via bisection.
     * At large T, the geometry may become invalid or phi goes out of range.
     */
    private fun bisectFeasibleMax(
        state: ShotState,
        flywheelMap: FlywheelMap,
        tMin: Double,
        searchLimit: Double
    ): Double? {
        // Find where feasibility ends
        var lastFeasible = tMin
        val steps = 50
        val dt = (searchLimit - tMin) / steps
        for (i in 0..steps) {
            val t = tMin + i * dt
            val sol = ShotGeometry.solve(state, t, flywheelMap)
            if (sol != null && sol.feasible) {
                lastFeasible = t
            } else if (lastFeasible > tMin) {
                // Found the boundary - bisect between lastFeasible and t
                var a = lastFeasible
                var b = t
                repeat(BISECTION_ITERATIONS) {
                    val mid = (a + b) / 2.0
                    val sol2 = ShotGeometry.solve(state, mid, flywheelMap)
                    if (sol2 != null && sol2.feasible) {
                        a = mid
                    } else {
                        b = mid
                    }
                }
                return a
            }
        }

        return if (lastFeasible > tMin) lastFeasible else null
    }
}
