package sigmacorns.control.ltv

import dev.frozenmilk.sinister.util.NativeLibraryLoader
import sigmacorns.opmode.SigmaOpMode.Companion.SIM

object MecanumLTVBridge {
    init {
        if(SIM) System.loadLibrary("mecanum_ltv_jni") else {
            val resolvedPath = NativeLibraryLoader.resolveLatestHashedLibrary("mecanum_ltv_jni")
            if (resolvedPath != null) {
                System.load(resolvedPath)
            } else {
                System.loadLibrary("mecanum_ltv_jni")
            }
        }
    }

    @JvmStatic external fun nativeCreate(): Long
    @JvmStatic external fun nativeDestroy(handle: Long)

    @JvmStatic external fun nativeSetModelParams(
        handle: Long,
        mass: Double, inertia: Double,
        dampingLinear: Double, dampingAngular: Double,
        wheelRadius: Double, lx: Double, ly: Double,
        stallTorque: Double, freeSpeed: Double
    )

    @JvmStatic external fun nativeSetConfig(
        handle: Long,
        N: Int,
        qDiag: DoubleArray, rDiag: DoubleArray, qfDiag: DoubleArray,
        uMin: Double, uMax: Double,
        aTipX: Double, aTipY: Double,
    )

    /** Load trajectory as flat array of [t, px, py, theta, vx, vy, omega] per sample. Returns number of windows. */
    @JvmStatic external fun nativeLoadTrajectory(handle: Long, samples: DoubleArray, nSamples: Int, dt: Double): Int

    /** Configure cost-based window selection tuning. */
    @JvmStatic external fun nativeSetWindowSelConfig(
        handle: Long,
        posWeight: Double, timeWeight: Double, headingWeight: Double,
        searchRadius: Int, maxJump: Int, holdRadius: Double,
    )

    /** Save precomputed windows to a .bin file (v2 format). Returns 0 on success. */
    @JvmStatic external fun nativeSaveWindows(handle: Long, filepath: String): Int

    /**
     * Solve given state x0 [px, py, theta, vx, vy, omega] and time elapsed since last solve.
     * On first call after load, pass total elapsed time since trajectory start.
     * Writes N*4 controls to uOut. Returns the selected window index, or -1 on error.
     */
    @JvmStatic external fun nativeSolve(handle: Long, dtSinceLast: Double, x0: DoubleArray, uOut: DoubleArray): Int

    /** Returns the window index selected by the most recent solve() call. */
    @JvmStatic external fun nativeGetPrevIdx(handle: Long): Int

    /** Returns the ETA (seconds) computed by the most recent solveWaypoint() call. */
    @JvmStatic external fun nativeGetWaypointEta(handle: Long): Double

    /** Fills xRefOut[6] with x_ref_0 for windowIdx. Returns false on bad index. */
    @JvmStatic external fun nativeGetWindowRef(handle: Long, windowIdx: Int, xRefOut: DoubleArray): Boolean

    @JvmStatic external fun nativeNumWindows(handle: Long): Int
    @JvmStatic external fun nativeHorizonLength(handle: Long): Int
    @JvmStatic external fun nativeNumVars(handle: Long): Int

    /** Set QP solver type: 0 = FISTA, 1 = HPIPM_OCP, 2 = NEON_IPM */
    @JvmStatic external fun nativeSetSolverType(handle: Long, type: Int)

    /** Check if a solver type is available in this build. */
    @JvmStatic external fun nativeIsSolverAvailable(type: Int): Boolean

    /**
     * Solve to a target state without a preloaded trajectory.
     * x0 and xTarget are [px, py, theta, vx, vy, omega] (length 6).
     * dt:      control timestep in seconds — used if loadTrajectory has not been called.
     * lqrRef:  true = constant reference at x_target (LQR-optimal, best for zero-velocity
     *          arrival); false = Hermite-interpolated reference (better for nonzero velocity).
     * uOut receives 4 controls [V1, V2, V3, V4] for the first timestep.
     * Returns 0 on success, -1 on error.
     */
    @JvmStatic external fun nativeSolveWaypoint(
        handle: Long,
        dt: Double,
        x0: DoubleArray,
        xTarget: DoubleArray,
        tRemaining: Double,
        lqrRef: Boolean,
        qDiag: DoubleArray,
        r: Double,
        uOut: DoubleArray,
    ): Int
}
