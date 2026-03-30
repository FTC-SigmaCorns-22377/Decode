package sigmacorns.control.aim.ballistic

/**
 * Pre-shot planner for multi-ball sequences.
 *
 * Finds (T1, T2) pairs that minimize the transition time to the second shot
 * after the flywheel loses speed from firing the first ball. Runs once when
 * the operator triggers a shot sequence (~400 evaluations, <1ms).
 *
 * See docs/OPTIMIZED_SHOOT.md Section 4.
 */
object RobustShotPlanner {

    /**
     * Result of the robust shot planning.
     *
     * @param primaryShot optimal first shot parameters
     * @param secondShot optimal second shot parameters (after flywheel drop)
     * @param secondShotTransitionTime estimated time to reach second shot state
     */
    data class RobustShotResult(
        val primaryShot: ShotSolution,
        val secondShot: ShotSolution,
        val secondShotTransitionTime: Double
    )

    /**
     * Plan a two-shot sequence that minimizes recovery time for the second shot.
     *
     * After the first ball fires, the flywheel retains [BallisticConstants.FLYWHEEL_DROP_FRACTION]
     * of its velocity. The planner finds (T1, T2) where the second shot's required
     * actuator state is closest to the post-first-shot state.
     *
     * @param state world state snapshot
     * @param current current actuator state
     * @param tRange valid [T_min, T_max]
     * @param flywheelMap calibration map
     * @return best (T1, T2) result, or null if no feasible pair exists
     */
    fun plan(
        state: ShotState,
        current: ActuatorState,
        tRange: ClosedRange<Double>,
        flywheelMap: FlywheelMap
    ): RobustShotResult? {
        val n = BallisticConstants.ROBUST_GRID_RESOLUTION
        val dt = (tRange.endInclusive - tRange.start) / n

        var bestCost = Double.MAX_VALUE
        var bestResult: RobustShotResult? = null

        for (i in 0..n) {
            val t1 = tRange.start + i * dt
            val shot1 = ShotGeometry.solve(state, t1, flywheelMap) ?: continue
            if (!shot1.feasible) continue

            // State after first shot: flywheel drops, hood/turret remain at shot1 targets
            val postShotState = ActuatorState(
                phi = shot1.phi,
                omega = shot1.omega * BallisticConstants.FLYWHEEL_DROP_FRACTION,
                theta = shot1.theta
            )

            // Note: uses the same ShotState for shot2 as shot1. The robot will move
            // between shots, so this is an approximation that improves as shots are
            // fired closer together. Acceptable when planning happens near firing time.
            for (j in 0..n) {
                val t2 = tRange.start + j * dt
                val shot2 = ShotGeometry.solve(state, t2, flywheelMap) ?: continue
                if (!shot2.feasible) continue

                val transitionCost = TransitionCost.compute(shot2, postShotState)
                if (transitionCost < bestCost) {
                    bestCost = transitionCost
                    bestResult = RobustShotResult(shot1, shot2, transitionCost)
                }
            }
        }

        return bestResult
    }
}
