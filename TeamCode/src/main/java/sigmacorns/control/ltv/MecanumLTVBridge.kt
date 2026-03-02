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
                System.loadLibrary("rerun")
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
        uMin: Double, uMax: Double
    )

    /** Load trajectory as flat array of [t, px, py, theta, vx, vy, omega] per sample. Returns number of windows. */
    @JvmStatic external fun nativeLoadTrajectory(handle: Long, samples: DoubleArray, nSamples: Int, dt: Double): Int

    /** Solve for window at windowIdx given state x0 [px, py, theta, vx, vy, omega]. Writes N*4 controls to uOut. */
    @JvmStatic external fun nativeSolve(handle: Long, windowIdx: Int, x0: DoubleArray, uOut: DoubleArray): Int

    @JvmStatic external fun nativeNumWindows(handle: Long): Int
    @JvmStatic external fun nativeHorizonLength(handle: Long): Int
    @JvmStatic external fun nativeNumVars(handle: Long): Int
}
