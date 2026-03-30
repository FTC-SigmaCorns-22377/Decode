package sigmacorns.control.aim.ballistic

/**
 * 1D optimizer that finds the travel time T minimizing transition cost.
 *
 * Uses golden-section search with warm-starting from the previous solution.
 * Runs at 200Hz in the main loop — each iteration is one [ShotGeometry.solve]
 * call (~10 trig operations).
 *
 * See docs/OPTIMIZED_SHOOT.md Section 3.
 */
class OptimalShotSearch(private val flywheelMap: FlywheelMap) {

    private var lastT: Double? = null

    /**
     * Find T* = argmin_T J_delta(T, current_state) subject to feasibility.
     *
     * @param state world state snapshot
     * @param current current actuator state
     * @param tRange valid [T_min, T_max] from [TravelTimeBounds]
     * @return best feasible solution, or null if none found
     */
    fun search(
        state: ShotState,
        current: ActuatorState,
        tRange: ClosedRange<Double>
    ): ShotSolution? {
        var a = tRange.start
        var b = tRange.endInclusive

        // Warm start: narrow search around previous solution
        val warm = lastT
        if (warm != null && warm in tRange) {
            val window = (b - a) * BallisticConstants.WARM_START_WINDOW_FRACTION
            a = maxOf(a, warm - window)
            b = minOf(b, warm + window)
        }

        // Golden section search
        val invGr = 1.0 / BallisticConstants.GOLDEN_RATIO
        var c = b - (b - a) * invGr
        var d = a + (b - a) * invGr

        for (i in 0 until BallisticConstants.MAX_SEARCH_ITERATIONS) {
            if (b - a < BallisticConstants.SEARCH_TOLERANCE) break

            val fc = costAt(state, current, c)
            val fd = costAt(state, current, d)

            if (fc < fd) {
                b = d
            } else {
                a = c
            }
            c = b - (b - a) * invGr
            d = a + (b - a) * invGr
        }

        val bestT = (a + b) / 2.0
        val solution = ShotGeometry.solve(state, bestT, flywheelMap)

        if (solution != null && solution.feasible) {
            lastT = bestT
            return solution
        }

        // If warm-started search missed, try full range
        if (warm != null) {
            lastT = null
            return search(state, current, tRange)
        }

        return null
    }

    private fun costAt(state: ShotState, current: ActuatorState, T: Double): Double {
        val shot = ShotGeometry.solve(state, T, flywheelMap) ?: return Double.MAX_VALUE
        if (!shot.feasible) return Double.MAX_VALUE
        return TransitionCost.compute(shot, current)
    }

    fun reset() {
        lastT = null
    }
}
