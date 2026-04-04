package sigmacorns.control.aim

/**
 * JNI bridge to the native turret_planner library (libturret_planner_jni.so).
 *
 * Config array layouts (must match turret_planner_jni.cpp):
 *   physConfig  [2]:  g (m/s²), rH (barrel offset, m)
 *   bounds      [6]:  thetaMin, thetaMax, phiMin, phiMax, vExitMax, omegaMax
 *   weights     [3]:  wTheta (s/rad), wPhi (s/rad), wOmega (s/unit)
 *   omegaCoeffs [6]:  c0..c5 for ω(φ,v) = c0 + c1·v + c2·φ + c3·v² + c4·φv + c5·φ²
 *   zoneConfig  [8]:  nx, ny, d (half-plane n·p >= d), Q_process, p_low, p_high,
 *                     alpha_lpf (0=no smoothing, 1=hold), omega_idle
 *
 * Path encoding: FloatArray with 5 floats per sample: [t, x, y, vx, vy].
 *
 * All angles in radians, distances in meters, velocities in m/s.
 * flywheel speed units are whatever omegaCoeffs were calibrated with.
 */
class TurretPlannerBridge {

    // ------------------------------------------------------------------
    // Return-index constants — avoids magic numbers at call sites.
    // ------------------------------------------------------------------

    /** Index constants for the 4-element array from [solve]. */
    object SolveIdx {
        const val THETA   = 0
        const val PHI     = 1
        const val V_EXIT  = 2
        const val OMEGA   = 3
    }

    /** Index constants for the 7-element array from [optimalTCold] / [optimalTWarm]. */
    object OptimalTIdx {
        const val T_STAR   = 0
        const val TAU      = 1
        const val THETA    = 2
        const val PHI      = 3
        const val V_EXIT   = 4
        const val OMEGA    = 5
        const val FEASIBLE = 6   // 1f = feasible, 0f = infeasible
    }

    /** Index constants for the 7-element array from [findEarliestShot]. */
    object ShotScanIdx {
        const val FOUND   = 0   // 1f = found, 0f = not found
        const val T_PATH  = 1   // path time of earliest feasible shot
        const val T_STAR  = 2
        const val THETA   = 3
        const val PHI     = 4
        const val V_EXIT  = 5
        const val OMEGA   = 6
    }

    /** Index constants for the 4-element array from [computePreposition]. */
    object PrepositionIdx {
        const val THETA              = 0
        const val PHI                = 1
        const val OMEGA              = 2
        const val EXPECTED_EARLIEST  = 3
    }

    /** Index constants for the 8-element array from [updateZoneTracker]. */
    object ZoneTrackerIdx {
        const val URGENCY          = 0
        const val URGENCY_FILTERED = 1
        const val EFFORT           = 2
        const val TAU              = 3
        const val TARGET_THETA     = 4
        const val TARGET_PHI       = 5
        const val TARGET_OMEGA     = 6
        const val SHOULD_FIRE      = 7   // 1f = fire, 0f = hold
    }

    // ------------------------------------------------------------------
    // Core ballistics
    // ------------------------------------------------------------------

    /**
     * Compute shot parameters for a given flight time T.
     * @return FloatArray(4): [theta, phi, vExit, omega]  (see [SolveIdx])
     */
    external fun solve(
        turretX: Float, turretY: Float, turretZ: Float,
        targetX: Float, targetY: Float, targetZ: Float,
        robotVx: Float, robotVy: Float,
        T: Float,
        physConfig: FloatArray,
        omegaCoeffs: FloatArray
    ): FloatArray

    // ------------------------------------------------------------------
    // Flight time optimizers
    // ------------------------------------------------------------------

    /**
     * Cold-start optimal flight time via Piyavskii-Shubert.
     * Use this when no warm start is available.
     * @return FloatArray(7): [T*, tau, theta, phi, vExit, omega, feasible]  (see [OptimalTIdx])
     */
    external fun optimalTCold(
        turretX: Float, turretY: Float, turretZ: Float,
        targetX: Float, targetY: Float, targetZ: Float,
        robotVx: Float, robotVy: Float,
        curTheta: Float, curPhi: Float, curOmega: Float,
        weights: FloatArray,
        bounds: FloatArray,
        physConfig: FloatArray,
        omegaCoeffs: FloatArray,
        tol: Float = 5e-4f
    ): FloatArray

    /**
     * Warm-started Newton refinement from [TInit].
     * Typically 2–4 iterations; use when path warm-start T* is available.
     * @return FloatArray(7): [T*, tau, theta, phi, vExit, omega, feasible]  (see [OptimalTIdx])
     */
    external fun optimalTWarm(
        turretX: Float, turretY: Float, turretZ: Float,
        targetX: Float, targetY: Float, targetZ: Float,
        robotVx: Float, robotVy: Float,
        TInit: Float,
        curTheta: Float, curPhi: Float, curOmega: Float,
        weights: FloatArray,
        bounds: FloatArray,
        physConfig: FloatArray,
        omegaCoeffs: FloatArray
    ): FloatArray

    // ------------------------------------------------------------------
    // Path scan
    // ------------------------------------------------------------------

    /**
     * Find the earliest feasible shot along a path.
     *
     * [path] is a flat FloatArray with 5 floats per sample: [t, x, y, vx, vy].
     * [nSamples] is the number of samples (path.size must be >= nSamples * 5).
     *
     * @return FloatArray(7): [found, tPath, T*, theta, phi, vExit, omega]  (see [ShotScanIdx])
     */
    external fun findEarliestShot(
        path: FloatArray, nSamples: Int,
        turretX: Float, turretY: Float, turretZ: Float,
        robotVx: Float, robotVy: Float,
        targetZ: Float,
        tNow: Float,
        curTheta: Float, curPhi: Float, curOmega: Float,
        weights: FloatArray,
        bounds: FloatArray,
        physConfig: FloatArray,
        omegaCoeffs: FloatArray
    ): FloatArray

    // ------------------------------------------------------------------
    // Pre-positioning
    // ------------------------------------------------------------------

    /**
     * Compute a robust pre-position target over the first [kSamples] of a path.
     *
     * @param tAvailable seconds available before the path begins
     * @param lambdaDecay exponential decay on sample weights (0 = uniform)
     * @return FloatArray(4): [theta, phi, omega, expectedEarliestT]  (see [PrepositionIdx])
     */
    external fun computePreposition(
        path: FloatArray, nSamples: Int,
        turretX: Float, turretY: Float, turretZ: Float,
        robotVx: Float, robotVy: Float,
        targetZ: Float,
        curTheta: Float, curPhi: Float, curOmega: Float,
        tAvailable: Float,
        lambdaDecay: Float,
        kSamples: Int,
        weights: FloatArray,
        bounds: FloatArray,
        physConfig: FloatArray,
        omegaCoeffs: FloatArray
    ): FloatArray

    // ------------------------------------------------------------------
    // Zone tracker (stateful — one native object per OpMode)
    // ------------------------------------------------------------------

    /**
     * Create a native ZoneTracker. Must be paired with [destroyZoneTracker].
     *
     * Call once at OpMode init; store the returned handle.
     *
     * @return native handle (pass to update/reset/destroy)
     */
    external fun createZoneTracker(
        zoneConfig: FloatArray,
        weights: FloatArray,
        bounds: FloatArray,
        physConfig: FloatArray,
        omegaCoeffs: FloatArray
    ): Long

    /**
     * Update the zone tracker for one control tick.
     *
     * @return FloatArray(8): [urgency, urgencyFiltered, effort, tau,
     *                          targetTheta, targetPhi, targetOmega, shouldFire]
     *                        (see [ZoneTrackerIdx])
     */
    external fun updateZoneTracker(
        handle: Long,
        robotX: Float, robotY: Float, robotHeading: Float,
        robotVx: Float, robotVy: Float, robotOmega: Float,
        turretTheta: Float, turretPhi: Float, turretOmega: Float,
        targetX: Float, targetY: Float, targetZ: Float,
        dt: Float
    ): FloatArray

    /** Reset internal filter state (call between OpMode runs). */
    external fun resetZoneTracker(handle: Long)

    /** Free the native object. Call at OpMode stop. */
    external fun destroyZoneTracker(handle: Long)

    // ------------------------------------------------------------------
    // Companion
    // ------------------------------------------------------------------

    companion object {
        init {
            System.loadLibrary("turret_planner_jni")
        }

        /** Encode a path for [findEarliestShot] / [computePreposition]. */
        fun encodePath(samples: List<PathSample>): FloatArray {
            val out = FloatArray(samples.size * 5)
            samples.forEachIndexed { i, s ->
                out[i*5 + 0] = s.t
                out[i*5 + 1] = s.x
                out[i*5 + 2] = s.y
                out[i*5 + 3] = s.vx
                out[i*5 + 4] = s.vy
            }
            return out
        }
    }
}

/** One sample along a path: time, position and velocity in field frame. */
data class PathSample(
    val t: Float,
    val x: Float,
    val y: Float,
    val vx: Float = 0f,
    val vy: Float = 0f
)
