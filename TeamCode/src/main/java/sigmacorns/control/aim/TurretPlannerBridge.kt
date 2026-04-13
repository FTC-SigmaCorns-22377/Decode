package sigmacorns.control.aim

import dev.frozenmilk.sinister.util.NativeLibraryLoader
import sigmacorns.opmode.SigmaOpMode.Companion.SIM

/**
 * JNI bridge to the native turret_planner library (libturret_planner_jni.so).
 *
 * Config array layouts (must match turret_planner_jni.cpp):
 *   physConfig  [3]:  g (m/s²), rH (barrel offset, m), dragK (1/s, 0=no drag)
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

    /** Index constants for the 12-element array from [robustShot]. */
    object RobustShotIdx {
        const val FEASIBLE  = 0   // 1f = feasible, 0f = infeasible
        const val T1        = 1
        const val T2        = 2
        const val J         = 3   // best objective value
        const val S1_THETA  = 4
        const val S1_PHI    = 5
        const val S1_V_EXIT = 6
        const val S1_OMEGA  = 7
        const val S2_THETA  = 8
        const val S2_PHI    = 9
        const val S2_V_EXIT = 10
        const val S2_OMEGA  = 11
    }

    /** Index constants for the 22-element array from [robust3ShotPlan]. */
    object Robust3ShotPlanIdx {
        const val FEASIBLE  = 0
        const val IDX1      = 1
        const val IDX2      = 2
        const val IDX3      = 3
        const val J         = 4
        const val J_12      = 5
        const val J_23      = 6
        const val T1        = 7
        const val T2        = 8
        const val T3        = 9
        const val S1_THETA  = 10
        const val S1_PHI    = 11
        const val S1_V_EXIT = 12
        const val S1_OMEGA  = 13
        const val S2_THETA  = 14
        const val S2_PHI    = 15
        const val S2_V_EXIT = 16
        const val S2_OMEGA  = 17
        const val S3_THETA  = 18
        const val S3_PHI    = 19
        const val S3_V_EXIT = 20
        const val S3_OMEGA  = 21
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

    /**
     * Forward-simulate a shot and return the horizontal miss distance (m)
     * at the target height. Accounts for air drag.
     */
    external fun shotError(
        turretX: Float, turretY: Float, turretZ: Float,
        targetX: Float, targetY: Float, targetZ: Float,
        robotVx: Float, robotVy: Float,
        theta: Float, phi: Float, vExit: Float, omega: Float,
        T: Float,
        physConfig: FloatArray
    ): Float

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

    /**
     * Plan a pair of consecutive shots (target1 then target2) that are robust
     * to an expected flywheel speed drop [dropFraction] between them.
     *
     * Minimizes `J(T1, T2) = max(w_theta·|Δθ|, w_phi·|Δφ|, w_omega·|Δω_adj|)`
     * where Δω_adj accounts for scaling shot 1's required flywheel speed by
     * (1 - [dropFraction]) to model the proportional energy loss after firing.
     *
     * @return FloatArray(12): [feasible, T1, T2, J,
     *                          s1.theta, s1.phi, s1.vExit, s1.omega,
     *                          s2.theta, s2.phi, s2.vExit, s2.omega]  (see [RobustShotIdx])
     */
    external fun robustShot(
        turretX: Float, turretY: Float, turretZ: Float,
        target1X: Float, target1Y: Float, target1Z: Float,
        target2X: Float, target2Y: Float, target2Z: Float,
        robotVx: Float, robotVy: Float,
        dropFraction: Float,
        weights: FloatArray,
        bounds: FloatArray,
        physConfig: FloatArray,
        omegaCoeffs: FloatArray,
        tol: Float = 5e-4f,
        maxIter: Int = 40
    ): FloatArray

    /**
     * Plan a pair of consecutive shots from a known current turret state,
     * minimizing *total* time to fire both balls:
     *
     *   J(T1, T2) = J_Δ(cur → s1(T1))  +  J_Δ(s1(T1)_reduced → s2(T2))
     *
     * The first term is the slew cost from the current turret state to the
     * first shot; the second is the cost of transitioning from shot 1 (with
     * its flywheel speed scaled by (1-[dropFraction])) to shot 2. Same return shape
     * as [robustShot] — see [RobustShotIdx].
     *
     * Use when the robot is already inside a shooting zone and you want the
     * fastest two-shot burst from right now.
     *
     * @return FloatArray(12): same layout as [robustShot], see [RobustShotIdx]
     */
    external fun robustAdjust(
        turretX: Float, turretY: Float, turretZ: Float,
        target1X: Float, target1Y: Float, target1Z: Float,
        target2X: Float, target2Y: Float, target2Z: Float,
        robotVx: Float, robotVy: Float,
        curTheta: Float, curPhi: Float, curOmega: Float,
        dropFraction: Float,
        weights: FloatArray,
        bounds: FloatArray,
        physConfig: FloatArray,
        omegaCoeffs: FloatArray,
        tol: Float = 5e-4f,
        maxIter: Int = 40
    ): FloatArray

    // ------------------------------------------------------------------
    // Trajectory-aware N-shot planner
    // ------------------------------------------------------------------

    /**
     * Plan up to 3 consecutive shots along a predicted trajectory.
     *
     * [trajectory] is a flat FloatArray with 6 floats per state: [t, x, y, heading, vx, vy].
     * Shot timing is deterministic: shot 2 fires at shot1_time + [transferTime],
     * shot 3 at shot1_time + 2*[transferTime]. The solver sweeps over first-shot
     * timing candidates and jointly optimizes flight times (T1, T2, T3) via
     * branch-and-bound to minimize transition costs between consecutive shots.
     *
     * The first shot cannot happen before [tRemaining] on the trajectory.
     *
     * Minimizes J_0 + J_12 + J_23 subject to J_12 <= transferTime and
     * J_23 <= transferTime (transitions must complete within the transfer window).
     *
     * @return FloatArray(22): see [Robust3ShotPlanIdx]
     */
    external fun robust3ShotPlan(
        trajectory: FloatArray, nStates: Int,
        turretZ: Float,
        targetX: Float, targetY: Float, targetZ: Float,
        curTheta: Float, curPhi: Float, curOmega: Float,
        nBalls: Int,
        tRemaining: Float,
        transferTime: Float,
        dropFraction: Float,
        weights: FloatArray,
        bounds: FloatArray,
        physConfig: FloatArray,
        omegaCoeffs: FloatArray,
        tol: Float = 5e-4f,
        maxIter: Int = 80
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

    /**
     * Robust variant of [computePreposition]: for each path sample, the shot is
     * solved as the first half of a robust pair against the next sample
     * (see [robustShot]), biasing the pre-position toward turret states that
     * make the *following* shot easy under an expected flywheel drop
     * [dropFraction].
     *
     * Note: this is NOT equivalent to [computePreposition] even when
     * `dropFraction == 0`, because the two functions optimize different objectives
     * per sample — [computePreposition] minimizes τ from the current turret
     * state to the shot, while this minimizes the transition between
     * consecutive shots.
     *
     * @return FloatArray(4): [theta, phi, omega, expectedEarliestT]  (see [PrepositionIdx])
     */
    external fun computeRobustPreposition(
        path: FloatArray, nSamples: Int,
        turretX: Float, turretY: Float, turretZ: Float,
        robotVx: Float, robotVy: Float,
        targetZ: Float,
        curTheta: Float, curPhi: Float, curOmega: Float,
        tAvailable: Float,
        lambdaDecay: Float,
        kSamples: Int,
        dropFraction: Float,
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

            if(SIM) System.loadLibrary("turret_planner_jni") else {
                val resolvedPath = NativeLibraryLoader.resolveLatestHashedLibrary("turret_planner_jni")
                if (resolvedPath != null) {
                    System.load(resolvedPath)
                } else {
                    System.loadLibrary("turret_planner_jni")
                }
            }
        }

        /** Encode a trajectory for [robust3ShotPlan]. 6 floats per state: [t, x, y, heading, vx, vy]. */
        fun encodeTrajectory(states: List<TrajectoryState>): FloatArray {
            val out = FloatArray(states.size * 6)
            states.forEachIndexed { i, s ->
                out[i*6 + 0] = s.t
                out[i*6 + 1] = s.x
                out[i*6 + 2] = s.y
                out[i*6 + 3] = s.heading
                out[i*6 + 4] = s.vx
                out[i*6 + 5] = s.vy
            }
            return out
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

/** Predicted robot state at a future time, for [TurretPlannerBridge.robust3ShotPlan]. */
data class TrajectoryState(
    val t: Float,          // seconds from now
    val x: Float,          // predicted field-frame position (m)
    val y: Float,
    val heading: Float,    // predicted heading (rad)
    val vx: Float,         // predicted field-frame velocity (m/s)
    val vy: Float
)
