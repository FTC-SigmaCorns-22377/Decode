package sigmacorns.control.mpc

import org.joml.Vector2d
import sigmacorns.constants.drivetrainCenter
import sigmacorns.control.SlewRateLimiter
import sigmacorns.math.Pose2d
import sigmacorns.sim.MECANUM_DT
import sigmacorns.sim.MecanumDynamics
import sigmacorns.sim.MecanumParameters
import sigmacorns.sim.MecanumState
import java.net.InetSocketAddress
import java.net.SocketAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.channels.ClosedByInterruptException
import java.nio.channels.DatagramChannel
import java.nio.channels.SelectionKey
import java.nio.channels.Selector
import kotlinx.coroutines.delay
import kotlinx.coroutines.yield
import sigmacorns.io.rotate
import kotlin.math.PI
import kotlin.math.min
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.nanoseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit
import kotlin.time.times

/**
 * Immutable snapshot of trajectory state for thread-safe access.
 */
data class TrajectorySnapshot(
    val trajectory: TrajoptTrajectory?,
    val path: List<LinearContour>,
    val referencePositions: List<Vector2d>,
    val timestamps: List<Double>,
) {
    companion object {
        val EMPTY = TrajectorySnapshot(null, emptyList(), emptyList(), emptyList())
    }
}

class MPCClient(
    val parameters: MecanumParameters,
    val SOLVER_IP: String = "172.29.0.1",
    SOLVER_PORT: Int = 5000,
    ROBOT_PORT: Int = 22377,
    val sampleLookahead: Int = 0,
    val preIntegrate: Duration = 0.milliseconds,
    var contourSelectionMode: ContourSelectionMode = ContourSelectionMode.TIME,
    var tuning: MPCTuning = MPCTuning(),
    /**
     * Optional client-side slew rate limiting as a safety fallback.
     * Set to null to disable. Recommended: 3.0-5.0 (units/second) for smooth operation.
     * This is applied AFTER MPC output as a last-resort smoothing filter.
     * The MPC's wDU parameter should handle most smoothing; use this for additional safety.
     */
    var clientSlewRate: Double? = null,
): AutoCloseable {

    companion object {
        const val N: Int = 18
        const val K: Int = N + 1  // Number of knot points
        /** Squared distance threshold (meters) for trajectory completion check. */
        const val COMPLETION_DISTANCE_SQ = 0.2 * 0.2
        const val NX: Int = 6
        const val NU: Int = 3  // drive, strafe, turn
        const val NP: Int = 9
        const val NTUNING: Int = 7  // [w_normal, w_lag, w_heading, w_u, w_du, w_vel, w_omega]
        const val N_CONTOUR_PER_KNOT: Int = 8
        const val N_CONTOUR_PARAMS: Int = K * N_CONTOUR_PER_KNOT  // 152 doubles

        // Variable timesteps: first N_FAST use DT_FAST, next N_MED use DT_MED, rest use DT_SLOW
        const val N_FAST: Int = 4      // steps 0-3: 15ms
        const val N_MED: Int = 11      // steps 4-14: 40ms
        val DT_FAST: Duration = 15.milliseconds
        val DT_MED: Duration = 40.milliseconds
        val DT_SLOW: Duration = 75.milliseconds

        /** Get the timestep for a given stage index (0 to N-1). */
        fun getDtForStage(k: Int): Duration = when {
            k < N_FAST -> DT_FAST
            k < N_FAST + N_MED -> DT_MED
            else -> DT_SLOW
        }

        /** Get cumulative time offsets for each knot point (0 to K-1). */
        val timeOffsets: List<Duration> = buildList {
            add(0.milliseconds)
            for (k in 0 until K - 1) {
                add(last() + getDtForStage(k))
            }
        }

        val horizon: Duration = timeOffsets.last()

        /**
         * Convert a TrajoptTrajectory to a list of LinearContours.
         */
        fun load(traj: TrajoptTrajectory): List<LinearContour> {
            if (traj.samples.isEmpty()) return emptyList()

            val contours = mutableListOf<LinearContour>()
            var lastTheta = traj.samples.first().heading

            for ((i, sample) in traj.samples.withIndex()) {
                val nextSample = traj.samples.getOrNull(i + 1)

                // Unwrap heading so it's continuous
                var theta = sample.heading
                while (theta - lastTheta > PI) theta -= PI * 2.0
                while (theta - lastTheta < -PI) theta += PI * 2.0
                lastTheta = theta

                val unwrappedSample = TrajoptSample(
                    timestamp = sample.timestamp,
                    vx = sample.vx,
                    vy = sample.vy,
                    omega = sample.omega,
                    x = sample.x,
                    y = sample.y,
                    heading = theta,
                )

                contours.add(LinearContour.fromSample(unwrappedSample, nextSample))
            }

            return contours
        }
    }

    val model = MecanumDynamics(parameters)

    @Volatile
    var lastTargetContour: LinearContour? = null
        private set

    @Volatile
    var predictedEvolution: List<MecanumState> = emptyList()
        private set

    /** The last set of K contours sent to the MPC solver (interpolated at variable time intervals). */
    @Volatile
    var lastSentContours: List<LinearContour> = emptyList()
        private set

    /** Thread-safe trajectory state - swap atomically via volatile reference */
    @Volatile
    var trajSnapshot: TrajectorySnapshot = TrajectorySnapshot.EMPTY

    @Volatile
    private var trajectoryStartTime: Duration? = null

    /** True until the MPC solver responds for the current trajectory. */
    @Volatile
    private var awaitingFirstResponse = true

    // Convenience accessors for the snapshot
    private val trajectory: TrajoptTrajectory? get() = trajSnapshot.trajectory
    val path: List<LinearContour> get() = trajSnapshot.path
    private val referencePositions: List<Vector2d> get() = trajSnapshot.referencePositions
    private val timestamps: List<Double> get() = trajSnapshot.timestamps

    private var sentTargetContours: MutableList<Pair<Long, LinearContour?>> = mutableListOf()
    private var curTime = 0.seconds

    private val HEADER_SIZE: Int = 4 + 2 + 2 + 4 + 8  // 20 bytes
    private val DOUBLE_SIZE: Int = 8
    private val REQUEST_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (N_CONTOUR_PARAMS + NX + NU + NP + NTUNING)
    private val RESPONSE_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (1 + N * NU)

    private var x0: MecanumState? = null
    private var V: Double = parameters.motor.vRef

    @Volatile
    private var latestU: DoubleArray = DoubleArray(N * NU)
    @Volatile
    private var latestUTime = 0L

    // Client-side slew rate limiters (one per control channel)
    private val slewLimiters = arrayOf(
        SlewRateLimiter(5.0),  // drive
        SlewRateLimiter(5.0),  // strafe
        SlewRateLimiter(5.0),  // turn
    )
    private var lastGetUTime: Duration? = null

    @Volatile
    var latestSampleI = 0
        private set

    private val channel = DatagramChannel.open()
    private val solverAddr = InetSocketAddress(SOLVER_IP, SOLVER_PORT)
    private val selector = Selector.open()

    init {
        channel.configureBlocking(false)
        channel.bind(InetSocketAddress(ROBOT_PORT))
        channel.register(selector, SelectionKey.OP_READ)
    }

    private data class TargetSelection(
        val closestIndex: Int,
        val targetIndex: Int,
    )

    private fun selectTarget(state: DoubleArray, time: Duration): TargetSelection? {
        if (referencePositions.isEmpty()) {
            return null
        }

        // Fix C: Don't start trajectory timer until solver has responded.
        // This prevents elapsed time from growing while the robot is stationary
        // waiting for the first MPC solution.
        val elapsed = if (trajectoryStartTime == null) {
            if (!awaitingFirstResponse) {
                trajectoryStartTime = time
            }
            0.0
        } else {
            (time - trajectoryStartTime!!).toDouble(DurationUnit.SECONDS)
        }

        val closestIndex = when {
            contourSelectionMode == ContourSelectionMode.TIME && timestamps.isNotEmpty() -> {
                timestamps.withIndex().minBy { (_, t) ->
                    kotlin.math.abs(t - elapsed)
                }.index
            }
            else -> referencePositions.withIndex().minBy { (i, ref) ->
                val dx = ref.x() - state[3]
                val dy = ref.y() - state[4]
                val dt = kotlin.math.abs((timestamps.getOrNull(i) ?: elapsed) - elapsed)
                dx * dx + dy * dy + dt*0.5
            }.index
        }

        latestSampleI = closestIndex

        val targetIndex = min(closestIndex + sampleLookahead, referencePositions.size - 1)

        return TargetSelection(
            closestIndex = closestIndex,
            targetIndex = targetIndex,
        )
    }

    var seq: Int = 0

    private fun unnormalizeHeading(heading: Double, target: Double): Double {
        var h = heading
        while (h - target > PI) h -= PI * 2.0
        while (h - target < -PI) h += PI * 2.0
        return h
    }

    /**
     * Interpolate a LinearContour at a given time using the trajectory.
     * Falls back to the nearest contour from path if trajectory is unavailable.
     */
    private fun interpolateContourAt(time: Double): LinearContour {
        val traj = trajectory
        val contours = path
        if (contours.isEmpty()) return LinearContour(Vector2d(), Vector2d(1.0, 0.0), 0.0, 0.0, 0.0, 0.0)

        // Use trajectory's sampleAt for interpolation if available
        if (traj != null) {
            val sample = traj.sampleAt(time) ?: return contours.last()
            return LinearContour.fromSample(sample, null)
        }

        // Fallback: find nearest by timestamp
        if (timestamps.isEmpty()) return contours.last()
        val idx = timestamps.withIndex().minBy { (_, t) -> kotlin.math.abs(t - time) }.index
        return contours[idx]
    }

    /**
     * Build the contour parameters array for all K knots.
     * Each knot has N_CONTOUR_PER_KNOT (8) values.
     * Contours are interpolated at variable time intervals (first N_FAST at DT_FAST, rest at DT_SLOW).
     */
    private fun buildContourParams(startTime: Double): DoubleArray {
        val params = DoubleArray(N_CONTOUR_PARAMS)
        val contours = path
        if (contours.isEmpty()) return params

        val interpolatedContours = mutableListOf<LinearContour>()

        // Unwrap headings to ensure continuity
        var lastTheta = interpolateContourAt(startTime).targetTheta

        for (k in 0 until K) {
            val time = startTime + timeOffsets[k].toDouble(DurationUnit.SECONDS)
            val contour = interpolateContourAt(time)

            // Unwrap heading
            var theta = contour.targetTheta
            while (theta - lastTheta > PI) theta -= PI * 2.0
            while (theta - lastTheta < -PI) theta += PI * 2.0
            lastTheta = theta

            val unwrappedContour = LinearContour(
                lineP = contour.lineP,
                lineD = contour.lineD,
                targetTheta = theta,
                targetOmega = contour.targetOmega,
                targetVx = contour.targetVx,
                targetVy = contour.targetVy,
            )
            interpolatedContours.add(unwrappedContour)

            val arr = unwrappedContour.toArray()
            for (j in 0 until N_CONTOUR_PER_KNOT) {
                params[k * N_CONTOUR_PER_KNOT + j] = arr[j]
            }
        }

        lastSentContours = interpolatedContours
        return params
    }

    private var lastTargetSelection: TargetSelection? = null

    private fun sendRequest(time: Long) {
        if (x0 == null) {
            println("MPC sendRequest: SKIPPED (x0 is null)")
            return
        }
        if (referencePositions.isEmpty()) {
            println("MPC sendRequest: SKIPPED (referencePositions is empty, no trajectory set)")
            return
        }

        // Predicted state
        val predictedX = predictState(x0!!, time.nanoseconds + preIntegrate)
        val predictedU = getU(time.nanoseconds + preIntegrate)

        val selection = selectTarget(predictedX.toDoubleArray(), time.nanoseconds) ?: return
        lastTargetSelection = selection
        val targetContour = path.getOrNull(selection.targetIndex) ?: return

        // Compute the start time for contour interpolation
        val startTime = if (timestamps.isNotEmpty() && selection.targetIndex < timestamps.size) {
            timestamps[selection.targetIndex]
        } else {
            // Fallback: estimate time from index (using DT_MED as approximate sample rate)
            selection.targetIndex * DT_MED.toDouble(DurationUnit.SECONDS)
        }

        val buf: ByteBuffer = ByteBuffer.allocate(REQUEST_MSG_SIZE)
        buf.order(ByteOrder.LITTLE_ENDIAN)

        // Header
        buf.putInt(0x564C4F53) // magic 'SOLV'
        buf.putShort(1.toShort()) // version
        buf.putShort(1.toShort()) // type = contour request
        buf.putInt(seq) // sequence number
        seq += 1
        buf.putLong(time) // timestamp

        val p = parameters.toArray()
        p[0] *= V / parameters.motor.vRef
        p[1] *= V / parameters.motor.vRef

        // Contour params (K * 8 doubles) - interpolated at variable time intervals
        putArray(buf, buildContourParams(startTime))

        // Unnormalize heading so MPC turns in shortest direction
        val stateArray = predictedX.toDoubleArray()
        stateArray[5] = unnormalizeHeading(stateArray[5], targetContour.targetTheta)

        putArray(buf, stateArray)  // x0 (6 doubles)
        putArray(buf, predictedU)  // u_last (4 doubles)
        putArray(buf, p)           // p (9 doubles)
        putArray(buf, tuning.toArray())  // tuning (7 doubles)

        buf.flip()

        sentTargetContours += time to targetContour

        try {
            if (channel.isOpen) {
                val bytesSent = channel.send(buf, solverAddr)
                println("MPC sendRequest: SENT $bytesSent bytes to $solverAddr (seq=$seq)")
            }
        } catch (e: ClosedByInterruptException) {
            channel.close()
        }
    }

    private fun receiveResponse() {
        if (selector.selectNow() > 0) {
            for (key in selector.selectedKeys()) {
                selector.selectedKeys().remove(key)
                if (key.isReadable) {
                    val buf: ByteBuffer = ByteBuffer.allocate(RESPONSE_MSG_SIZE)
                    buf.order(ByteOrder.LITTLE_ENDIAN)
                    var addr: SocketAddress? = null
                    try {
                        if (channel.isOpen) addr = channel.receive(buf)
                    } catch (_: ClosedByInterruptException) {
                        channel.close()
                    }
                    if (addr != null) {
                        buf.flip()
                        val magic = buf.getInt()
                        require(magic == 0x564C4F53)
                        val version = buf.getShort()
                        val type = buf.getShort()
                        val seq = buf.getInt()
                        val t = buf.getLong()

                        val t0 = buf.getLong()

                        println("SOLVED t0=$t0")

                        if (t0 >= latestUTime) {
                            println("UPDATED LATEST U")
                            latestUTime = t0
                            awaitingFirstResponse = false
                            getArray(buf, latestU)
                            println("MPC: UPDATED U = ${latestU.toList().chunked(3).map { "(${it[0]},${it[1]},${it[2]}), " }.reduce(String::plus)}")
                            val matched = sentTargetContours.find { it.first == t0 }
                            lastTargetContour = matched?.second
                            sentTargetContours.removeAll { it.first <= t0 }

                            repredictEvolution()
                        }
                    }
                }
            }
        }
    }

    private fun putArray(buf: ByteBuffer, arr: DoubleArray) {
        for (v in arr) buf.putDouble(v)
    }

    private fun getArray(buf: ByteBuffer, arr: DoubleArray) {
        for (i in arr.indices) arr[i] = buf.getDouble()
    }

    /**
     * Set the target trajectory from a TrajoptTrajectory.
     * Thread-safe: can be called from any thread.
     */
    fun setTarget(traj: TrajoptTrajectory) {
        val contours = load(traj)
        // Atomically swap the entire trajectory state
        trajSnapshot = TrajectorySnapshot(
            trajectory = traj,
            path = contours,
            referencePositions = contours.map { it.lineP },
            timestamps = traj.samples.map { it.timestamp }
        )
        // Reset other state
        trajectoryStartTime = null
        awaitingFirstResponse = true
        latestSampleI = 0
        lastTargetContour = null
        lastSentContours = emptyList()
        latestU = DoubleArray(N * NU)
    }

    /**
     * Set the target contours directly.
     * Thread-safe: can be called from any thread.
     */
    fun setTargetContours(path: List<LinearContour>, timestamps: List<Double> = emptyList()) {
        trajSnapshot = TrajectorySnapshot(
            trajectory = null,
            path = path,
            referencePositions = path.map { it.lineP },
            timestamps = timestamps
        )
        trajectoryStartTime = null
        awaitingFirstResponse = true
        latestSampleI = 0
        lastTargetContour = null
        lastSentContours = emptyList()
        latestU = DoubleArray(N * NU)
    }

    fun startPose(): Pose2d {
        val first = path.firstOrNull() ?: return Pose2d()
        return Pose2d(first.lineP, first.targetTheta)
    }

    fun resetTrajectoryStartTime() {
        trajectoryStartTime = null
    }

    fun update(state: MecanumState, v: Number, time: Duration) {
        V = v.toDouble()
        x0 = state.copy().also {

            // go from odometry center to drivetrain center
            it.pos.v.add(drivetrainCenter.rotate(it.pos.rot))
        }

        println("sending state=$state")
        println("sending $x0")

        println("Sending with ${time.inWholeNanoseconds}")
        sendRequest(time.inWholeNanoseconds)
    }

    fun getU(time: Duration): DoubleArray {
        receiveResponse()

        val delay = time - latestUTime.nanoseconds

        println("MPC DELAY: ${delay.inWholeMilliseconds}ms (true) ${(delay - preIntegrate).inWholeMilliseconds}ms (effective)")

        // Find the stage index using variable timesteps
        val effectiveDelay = delay - preIntegrate - 5.milliseconds
        val i = findStageIndex(effectiveDelay).coerceIn(0, N - 1)

        var res = latestU.copyOfRange(i * NU, (i + 1) * NU)

        // Apply optional client-side slew rate limiting
        val slewRate = clientSlewRate
        if (slewRate != null) {
            val dt = lastGetUTime?.let { time - it } ?: DT_FAST
            lastGetUTime = time

            slewLimiters.forEachIndexed { idx, limiter ->
                limiter.maxRate = slewRate
                res[idx] = limiter.calculate(res[idx], dt)
            }
        }

        return res
    }

    /** Find the stage index for a given elapsed time using variable timesteps. */
    private fun findStageIndex(elapsed: Duration): Int {
        for (i in 0 until N) {
            if (elapsed < timeOffsets[i + 1]) return i
        }
        return N - 1
    }

    private fun repredictEvolution() {
        val xs = mutableListOf(x0!!)
        for (i in 0 until N) {
            val u = latestU.sliceArray(i * NU..(i + 1) * NU - 1).let {
                val v = model.mecanumInversePowerKinematics(Pose2d(it[0],it[1],it[2]))
                doubleArrayOf(v[0],v[1],v[2],v[3])
            }
            val dt = getDtForStage(i).toDouble(DurationUnit.SECONDS)
            xs += model.integrate(dt, 0.001, u, xs.last())
        }

        predictedEvolution = xs
    }

    fun predictState(x0: MecanumState, time: Duration): MecanumState {
        val t = time - latestUTime.nanoseconds
        var state = x0
        val i = findStageIndex(t).coerceIn(0, N - 1)
        if (predictedEvolution.isNotEmpty()) {
            state = predictedEvolution[i]
        }
        val u = latestU.sliceArray(i * NU..(i + 1) * NU - 1).let {
            val v = model.mecanumInversePowerKinematics(Pose2d(it[0],it[1],it[2]))
            doubleArrayOf(v[0],v[1],v[2],v[3])
        }
        return model.integrate((t - timeOffsets[i]).toDouble(DurationUnit.SECONDS), MECANUM_DT, u, state)
    }

    fun isTrajectoryComplete(): Boolean {
        val snapshot = trajSnapshot
        if (snapshot.trajectory == null) return false
        if (trajectoryStartTime == null) return false

        // Check if we're at the last sample
        val pathSize = snapshot.path.size
        if (pathSize <= 0 || latestSampleI < pathSize - 1) return false

        val state = x0 ?: return false
        val lastPos = snapshot.path.last().lineP
        val dx = state.pos.v.x() - lastPos.x()
        val dy = state.pos.v.y() - lastPos.y()
        return dx * dx + dy * dy < COMPLETION_DISTANCE_SQ
    }

    /**
     * Creates a suspend function that runs MPC control on the given trajectory.
     * The function updates the drivetrain until the trajectory is complete.
     *
     * Usage:
     * ```
     * val mpc = MPCClient(...)
     * scope.launch {
     *     mpc.runTrajectory(traj, io)()
     * }
     * ```
     *
     * @param traj The trajectory to follow
     * @param updateDelay Optional delay between loop iterations (use for simulation)
     * @return A suspend function that runs the MPC and completes when trajectory is finished
     */
    fun runTrajectory(
        traj: TrajoptTrajectory,
        updateDelay: Duration = 0.milliseconds
    ): suspend () -> Unit = suspend {
        setTarget(traj)

        while (!isTrajectoryComplete()) {
            if (updateDelay > 0.milliseconds) {
                delay(updateDelay.inWholeMilliseconds)
            } else {
                yield()
            }
        }
    }

    override fun close() {
        if (channel.isOpen) {
            channel.close()
        }
    }
}
