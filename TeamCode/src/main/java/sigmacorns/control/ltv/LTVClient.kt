package sigmacorns.control.ltv

import sigmacorns.control.mpc.TrajoptTrajectory
import sigmacorns.sim.MecanumParameters
import sigmacorns.sim.MecanumState
import kotlin.math.floor
import kotlin.math.min
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

enum class QpSolverType(val nativeId: Int) {
    FISTA(0),
    HPIPM_OCP(1);

    val isAvailable: Boolean get() = MecanumLTVBridge.nativeIsSolverAvailable(nativeId)
}

class LTVClient private constructor(
    private val handle: Long,
    private var dtSeconds: Double,
) : AutoCloseable {

    private var numWindows: Int = 0
    private var numVars: Int = 0

    val dt: Duration get() = dtSeconds.seconds

    /**
     * Primary constructor: configure model params + MPC tuning, then call [loadTrajectory].
     */
    constructor(
        parameters: MecanumParameters,
        horizon: Int = 30,
        dt: Duration = 20.milliseconds,
        qDiag: DoubleArray = doubleArrayOf(100.0, 100.0, 100.0, 1.0, 1.0, 1.0),
        rDiag: DoubleArray = doubleArrayOf(0.005, 0.005, 0.005, 0.005),
        qfDiag: DoubleArray = doubleArrayOf(100.0, 100.0, 100.0, 2.0, 2.0, 2.0),
        solverType: QpSolverType = QpSolverType.HPIPM_OCP,
    ) : this(MecanumLTVBridge.nativeCreate(), dt.toDouble(DurationUnit.SECONDS)) {
        MecanumLTVBridge.nativeSetModelParams(
            handle,
            mass = parameters.weight,
            inertia = parameters.rotInertia,
            dampingLinear = parameters.cDragLin,
            dampingAngular = parameters.cDragRot,
            wheelRadius = parameters.wheelRadius,
            lx = parameters.lx,
            ly = parameters.ly,
            stallTorque = parameters.motor.stallTorque,
            freeSpeed = parameters.motor.freeSpeed,
        )
        MecanumLTVBridge.nativeSetConfig(handle, horizon, qDiag, rDiag, qfDiag, -1.0, 1.0)
        MecanumLTVBridge.nativeSetSolverType(handle, solverType.nativeId)
    }

    companion object {
        /**
         * Create an LTVClient from a precomputed .bin file.
         * No model params or config needed — everything is loaded from the file.
         * This is the fast path for robot use: load time is near-instant (just fread).
         */
        fun fromPrecomputed(
            filepath: String,
            solverType: QpSolverType = QpSolverType.HPIPM_OCP,
        ): LTVClient {
            val handle = MecanumLTVBridge.nativeCreate()
            val client = LTVClient(handle, 0.0)
            val n = MecanumLTVBridge.nativeLoadWindows(handle, filepath)
            require(n > 0) { "Failed to load precomputed windows from $filepath" }
            client.numWindows = n
            client.numVars = MecanumLTVBridge.nativeNumVars(handle)
            client.dtSeconds = MecanumLTVBridge.nativeDt(handle)
            MecanumLTVBridge.nativeSetSolverType(handle, solverType.nativeId)
            return client
        }
    }

    /**
     * Load a trajectory from a TrajoptTrajectory.
     * Converts from trajopt state order [vx, vy, omega, x, y, heading] to
     * native format [t, px, py, theta, vx, vy, omega] per sample.
     */
    fun loadTrajectory(traj: TrajoptTrajectory) {
        val samples = traj.samples
        val flat = DoubleArray(samples.size * 7)
        for (i in samples.indices) {
            val s = samples[i]
            val offset = i * 7
            flat[offset + 0] = s.timestamp  // t
            flat[offset + 1] = s.x          // px
            flat[offset + 2] = s.y          // py
            flat[offset + 3] = s.heading    // theta
            flat[offset + 4] = s.vx         // vx
            flat[offset + 5] = s.vy         // vy
            flat[offset + 6] = s.omega      // omega
        }
        numWindows = MecanumLTVBridge.nativeLoadTrajectory(handle, flat, samples.size, dtSeconds)
        numVars = MecanumLTVBridge.nativeNumVars(handle)
    }

    /**
     * Load precomputed windows from a .bin file (v2 format).
     * Can be called on an already-configured client as an alternative to [loadTrajectory].
     * Updates dt from the file header.
     */
    fun loadWindows(filepath: String) {
        numWindows = MecanumLTVBridge.nativeLoadWindows(handle, filepath)
        require(numWindows > 0) { "Failed to load precomputed windows from $filepath" }
        numVars = MecanumLTVBridge.nativeNumVars(handle)
        dtSeconds = MecanumLTVBridge.nativeDt(handle)
    }

    /**
     * Solve the LTV OCP for the current state and elapsed time.
     *
     * @param state Current mecanum state (pos + vel as Pose2d)
     * @param elapsedSeconds Time since trajectory start
     * @return 4 wheel duty cycles in SigmaIO order: [FL, BL, BR, FR]
     */
    fun solve(state: MecanumState, elapsedSeconds: Duration): DoubleArray {
        val windowIdx = min(floor(elapsedSeconds.toDouble(DurationUnit.SECONDS) / dtSeconds).toInt(), numWindows - 1)

        // Pack state as [px, py, theta, vx, vy, omega] (LTV native order)
        val x0 = doubleArrayOf(
            state.pos.v.x, state.pos.v.y, state.pos.rot,
            state.vel.v.x, state.vel.v.y, state.vel.rot,
        )

        val uOut = DoubleArray(numVars)
        MecanumLTVBridge.nativeSolve(handle, windowIdx, x0, uOut)

        // First 4 values are the controls for the first timestep
        // JNI order: [FL, FR, RL, RR] → SigmaIO order: [FL, BL, BR, FR]
        val fl = uOut[0]
        val fr = uOut[1]
        val rl = uOut[2]
        val rr = uOut[3]
        return doubleArrayOf(fl, rl, rr, fr) // [FL, BL, BR, FR]
    }

    fun isComplete(elapsedSeconds: Duration): Boolean {
        return floor(elapsedSeconds.toDouble(DurationUnit.SECONDS) / dtSeconds).toInt() >= numWindows
    }

    fun numWindows(): Int = numWindows

    override fun close() {
        MecanumLTVBridge.nativeDestroy(handle)
    }
}
