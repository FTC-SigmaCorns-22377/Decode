package sigmacorns.control.ltv

import kotlinx.coroutines.yield
import sigmacorns.control.trajopt.TrajoptTrajectory
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumParameters
import sigmacorns.sim.MecanumState
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

/**
 * Tuning parameters for cost-based window selection.
 *
 * @param posWeight      Weight on squared position+heading distance in the cost function.
 * @param timeWeight     Weight on squared time-schedule deviation in the cost function.
 * @param headingWeight  Scale applied to heading error (radians) in the distance metric.
 * @param searchRadius   Max windows to search forward from the current window each solve.
 * @param maxJump        Hard cap on window advance per solve call.
 * @param holdRadius     XY distance (metres) beyond which the window is frozen until the
 *                       robot re-approaches. Accumulated time is reset on re-entry.
 */
data class WindowSelConfig(
    val posWeight: Double     = 1.0,
    val timeWeight: Double    = 0.5,
    val headingWeight: Double = 0.3,
    val searchRadius: Int     = 10,
    val maxJump: Int          = 5,
    val holdRadius: Double    = 0.5,
)

enum class QpSolverType(val nativeId: Int) {
    FISTA(0),
    HPIPM_OCP(1),
    NEON_IPM(2);

    val isAvailable: Boolean get() = MecanumLTVBridge.nativeIsSolverAvailable(nativeId)
}

class LTVClient private constructor(
    private val handle: Long,
    private var dtSeconds: Double,
    private var solverType: QpSolverType = QpSolverType.NEON_IPM,
    val qDiag: DoubleArray,
    val qfDiag: DoubleArray,
    val rDiag: DoubleArray
) : AutoCloseable {

    private var numWindows: Int = 0
    private var numVars: Int = 0

    // Tracks the elapsed time of the previous solve call for dt_since_last computation.
    // Null before the first solve after a trajectory load.
    private var prevCallElapsed: Double? = null

    val dt: Duration get() = dtSeconds.seconds


    /**
     * Primary constructor: configure model params + MPC tuning, then call [loadTrajectory].
     */
    constructor(
        parameters: MecanumParameters,
        horizon: Int = 30,
        dt: Duration = 20.milliseconds,
        qDiag: DoubleArray = doubleArrayOf(100.0, 100.0, 400.0, 1.0, 1.0, 1.0),
        rDiag: DoubleArray = doubleArrayOf(0.05, 0.05, 0.05, 0.05),
        qfDiag: DoubleArray = doubleArrayOf(100.0, 100.0, 400.0, 2.0, 2.0, 2.0),
        solverType: QpSolverType = QpSolverType.NEON_IPM,
        windowSelConfig: WindowSelConfig = WindowSelConfig(),
        /** Anti-tip X acceleration limit (m/s²) in robot body frame. 0 = disabled.
         *  Typical value: g * (half fore-aft wheelbase) / h_com */
        aTipX: Double = 0.0,
        /** Anti-tip Y acceleration limit (m/s²) in robot body frame. 0 = disabled.
         *  Typical value: g * (half lateral track width) / h_com */
        aTipY: Double = 0.0,
        /** Low-pass time constant (s) for the sustained-acceleration tip constraint.
         *  0 (default) → per-step instantaneous barrier (original).
         *  Values ~half the robot's pitch period (e.g. 0.15–0.4 s) allow brief
         *  transient accelerations while preventing sustained tip-inducing deceleration,
         *  eliminating mid-path chattering without sacrificing end-of-path protection. */
        aTipTau: Double = 0.0,
    ) : this(MecanumLTVBridge.nativeCreate(), dt.toDouble(DurationUnit.SECONDS),qDiag=qDiag, qfDiag=qfDiag, rDiag=rDiag) {
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
        MecanumLTVBridge.nativeSetConfig(handle, horizon, qDiag, rDiag, qfDiag, -1.0, 1.0, aTipX, aTipY, aTipTau)
        this.solverType = solverType
        MecanumLTVBridge.nativeSetSolverType(handle, solverType.nativeId)
        setWindowSelConfig(windowSelConfig)
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
        prevCallElapsed = null
    }

    /** Update window selection tuning. Takes effect on the next [solve] call. */
    fun setWindowSelConfig(cfg: WindowSelConfig) {
        MecanumLTVBridge.nativeSetWindowSelConfig(
            handle,
            cfg.posWeight, cfg.timeWeight, cfg.headingWeight,
            cfg.searchRadius, cfg.maxJump, cfg.holdRadius,
        )
    }

    /**
     * Save precomputed windows to a .bin file (v2 format).
     */
    fun saveWindows(filepath: String) {
        val result = MecanumLTVBridge.nativeSaveWindows(handle, filepath)
        require(result == 0) { "Failed to save precomputed windows to $filepath" }
    }

    /**
     * Solve the LTV OCP for the current state and elapsed time.
     *
     * On the first call after loading a trajectory, pass total time since trajectory start.
     * On subsequent calls, the time since the previous call is computed automatically.
     *
     * @param state Current mecanum state (pos + vel as Pose2d)
     * @param elapsedSeconds Time since trajectory start
     * @return 4 wheel duty cycles in SigmaIO order: [FL, BL, BR, FR]
     */
    fun solve(state: MecanumState, elapsedSeconds: Duration): DoubleArray {
        val elapsedSec = elapsedSeconds.toDouble(DurationUnit.SECONDS)
        val prev = prevCallElapsed
        val dtSinceLast = if (prev == null) elapsedSec else (elapsedSec - prev)
        prevCallElapsed = elapsedSec

        // Pack state as [px, py, theta, vx, vy, omega] (LTV native order)
        val x0 = doubleArrayOf(
            state.pos.v.x, state.pos.v.y, state.pos.rot,
            state.vel.v.x, state.vel.v.y, state.vel.rot,
        )

        val uOut = DoubleArray(numVars)
        MecanumLTVBridge.nativeSolve(handle, dtSinceLast, x0, uOut)

        // First 4 values are the controls for the first timestep
        // JNI order: [FL, FR, RL, RR] → SigmaIO order: [FL, BL, BR, FR]
        val fl = uOut[0]
        val fr = uOut[1]
        val rl = uOut[2]
        val rr = uOut[3]
        return doubleArrayOf(fl, rl, rr, fr) // [FL, BL, BR, FR]
    }

    /**
     * Solve to a waypoint without a preloaded trajectory.
     *
     * Generates a Hermite-interpolated reference from the current state to [target]
     * over [tRemaining] and solves via NEON_IPM. No precomputation — target can
     * change every loop with zero overhead.
     *
     * Requires the constructor to have been called (model params + config), but NOT
     * [loadTrajectory]. Can be used alongside or instead of trajectory tracking.
     *
     * @param state      Current robot state
     * @param target     Desired state when [tRemaining] reaches zero
     * @param tRemaining Time until the waypoint should be reached
     * @param lqrRef     true  → constant reference = [target] for all steps. The
     *                           Riccati sweep solves the exact discrete LQR problem,
     *                           finding the dynamically-optimal path. Best for
     *                           zero-velocity arrival (stop at a point).
     *                   false → Hermite-interpolated reference that matches position
     *                           and velocity at both endpoints. Better when arriving
     *                           with nonzero velocity. (default)
     *                   Both modes shorten the horizon to ceil(tRemaining/dt) so
     *                   Qf lands exactly on the deadline.
     * @return           [FL, BL, BR, FR] duty cycles
     */
    fun solveWaypoint(
        state: MecanumState,
        target: MecanumState,
        tRemaining: Duration,
        lqrRef: Boolean = false,
        r: Double = 0.020,
    ): DoubleArray {
        val x0 = doubleArrayOf(
            state.pos.v.x, state.pos.v.y, state.pos.rot,
            state.vel.v.x, state.vel.v.y, state.vel.rot,
        )
        val xTarget = doubleArrayOf(
            target.pos.v.x, target.pos.v.y, target.pos.rot,
            target.vel.v.x, target.vel.v.y, target.vel.rot,
        )
        val uOut = DoubleArray(4)
        MecanumLTVBridge.nativeSolveWaypoint(
            handle, dtSeconds, x0, xTarget,
            maxOf(0.0, tRemaining.toDouble(DurationUnit.SECONDS)),
            lqrRef, qDiag, r, uOut,
        )
        // JNI order: [FL, FR, RL, RR] → SigmaIO order: [FL, BL, BR, FR]
        return doubleArrayOf(uOut[0], uOut[2], uOut[3], uOut[1])
    }

    fun holdPos(
        io: SigmaIO,
        p: Pose2d,
    ) {
        val u = solveWaypoint(
            MecanumState(io.velocity(),io.position()),
            MecanumState(Pose2d(),p),
            1.seconds,
            lqrRef = true
        )

        val voltage = io.voltage()
        io.driveFL = u[0] * 12.0 / voltage
        io.driveBL = u[1] * 12.0 / voltage
        io.driveBR = u[2] * 12.0 / voltage
        io.driveFR = u[3] * 12.0 / voltage
    }


    /** Returns the window index selected by the most recent solve() call. */
    fun prevWindowIdx(): Int = MecanumLTVBridge.nativeGetPrevIdx(handle)

    /**
     * Returns the ETA from the most recent [solveWaypoint] call.
     *
     * Computed by forward-simulating the solved control sequence and finding
     * the step at which the predicted trajectory is closest to the target in
     * XY. Use this as [tRemaining] on the next [solveWaypoint] call for a
     * self-consistent, model-derived horizon estimate.
     *
     * Returns [Duration.ZERO] before the first [solveWaypoint] call.
     */
    fun prevWaypointEta(): Duration = MecanumLTVBridge.nativeGetWaypointEta(handle).seconds

    /**
     * Returns the reference state [px, py, theta, vx, vy, omega] for the given window index,
     * or null if the index is out of range.
     */
    fun getWindowRef(windowIdx: Int): DoubleArray? {
        val out = DoubleArray(6)
        return if (MecanumLTVBridge.nativeGetWindowRef(handle, windowIdx, out)) out else null
    }

    /**
     * Returns true when the solver has reached the last window.
     * Uses the selected window index rather than elapsed time so that trials
     * spending time in hold mode (approaching the path) are not prematurely
     * considered complete. The [elapsedSeconds] safety cutoff (default 3× trajectory
     * duration) guards against infinite loops if a robot never converges.
     */
    fun isComplete(elapsedSeconds: Duration, safetyMultiplier: Double = 3.0): Boolean {
        val windowDone = prevWindowIdx() >= numWindows - 1
        val safetyCutoff = elapsedSeconds.toDouble(DurationUnit.SECONDS) >= dtSeconds * numWindows * safetyMultiplier
        return windowDone || safetyCutoff
    }

    fun numWindows(): Int = numWindows

    suspend fun runPathToCompletion(traj: TrajoptTrajectory, io: SigmaIO) {
        loadTrajectory(traj)
        val legStart = io.time()

        while (true) {
            val currentState = MecanumState(io.velocity(), io.position())
            val elapsedTime = (io.time() - legStart)

            val u = solve(currentState, elapsedTime)

            val voltage = io.voltage()
            io.driveFL = u[0] * 12.0 / voltage
            io.driveBL = u[1] * 12.0 / voltage
            io.driveBR = u[2] * 12.0 / voltage
            io.driveFR = u[3] * 12.0 / voltage

            if (isComplete(elapsedTime)) {
                return
            }

            yield()
        }
    }

    /**
     * Suspending function that runs a waypoint to completion.
     *
     * Repeatedly calls [solveWaypoint], applies controls via [applyControls], and checks
     * for arrival based on position and velocity tolerances. Returns when the robot arrives
     * or hits the timeout.
     *
     * @param state Current robot state
     * @param target Desired state (typically with zero velocity)
     * @param initialTRemaining Initial time horizon for the first solve call
     * @param applyControls Callback to apply wheel duty cycles and advance the simulation
     * @param posTol Position arrival tolerance (metres)
     * @param velTol Velocity arrival tolerance (m/s XY magnitude)
     * @param minTRemaining Minimum horizon to prevent collapse to a single step
     * @param timeout Hard per-waypoint timeout — forces return if the robot never converges
     */
    suspend fun runWaypointToCompletion(
        target: MecanumState,
        initialTRemaining: Duration,
        io: SigmaIO,
        posTol: Double = 0.05,
        headingTol: Double = 0.05,
        velTol: Double = 0.05,
        minTRemaining: Duration = 40.milliseconds,
        timeout: Duration = 10.seconds,
    ) {
        val legStart = io.time()
        val legDeadlineNs = legStart + timeout

        var tRemaining = initialTRemaining

        while (true) {
            val currentState = MecanumState(io.velocity(), io.position())
            val u = solveWaypoint(currentState, target, tRemaining, lqrRef = true)

            val voltage = io.voltage()
            io.driveFL = u[0] * 12.0 / voltage
            io.driveBL = u[1] * 12.0 / voltage
            io.driveBR = u[2] * 12.0 / voltage
            io.driveFR = u[3] * 12.0 / voltage

            // Update tRemaining from solver's forward-simulated ETA,
            // but floor it with a distance-based estimate so a single bad
            // native ETA can't collapse the horizon to 2 steps permanently.
            val posErr = hypot(
                currentState.pos.v.x - target.pos.v.x,
                currentState.pos.v.y - target.pos.v.y,
            )
            val distBasedMin = (posErr / 1.5).seconds // 1.5 m/s reference speed
            tRemaining = maxOf(prevWaypointEta(), distBasedMin, minTRemaining)
            val headingErr = abs(currentState.pos.rot - target.pos.rot)
            val velMag = hypot(currentState.vel.v.x, currentState.vel.v.y)

            if (posErr < posTol && headingErr < headingTol && velMag < velTol) {
                return  // Arrived
            }

            if (io.time() > legDeadlineNs) {
                return  // Timed out
            }

            yield()  // Suspend to allow other coroutines to run
        }
    }

    override fun close() {
        MecanumLTVBridge.nativeDestroy(handle)
    }
}
