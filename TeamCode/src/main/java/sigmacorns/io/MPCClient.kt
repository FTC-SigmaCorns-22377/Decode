package sigmacorns.io

import org.joml.Vector2d
import sigmacorns.constants.drivetrainCenter
import sigmacorns.control.DriveController
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
import sigmacorns.control.SlewRateLimiter
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sin
import kotlinx.coroutines.delay
import kotlinx.coroutines.yield
import sigmacorns.State
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.nanoseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit
import kotlin.time.times

/**
 * Linear contour parameters for a single knot point.
 * Each knot defines a target point on the path and the path direction.
 */
data class LinearContour(
    /** Target position on the path */
    val lineP: Vector2d,
    /** Path direction (unit vector) */
    val lineD: Vector2d,
    /** Target heading angle (radians) */
    val targetTheta: Double,
    /** Target angular velocity (rad/s) */
    val targetOmega: Double,
) {
    fun toArray(): DoubleArray = doubleArrayOf(
        lineP.x, lineP.y,
        lineD.x, lineD.y,
        targetTheta, targetOmega
    )

    companion object {
        const val SIZE = 6

        fun fromSample(sample: TrajoptSample, nextSample: TrajoptSample?): LinearContour {
            // Compute path direction from velocity or position difference
            val dir = if (sample.vx != 0.0 || sample.vy != 0.0) {
                val speed = kotlin.math.hypot(sample.vx, sample.vy)
                if (speed > 1e-6) {
                    Vector2d(sample.vx / speed, sample.vy / speed)
                } else {
                    Vector2d(1.0, 0.0)
                }
            } else if (nextSample != null) {
                val dx = nextSample.x - sample.x
                val dy = nextSample.y - sample.y
                val d = kotlin.math.hypot(dx, dy)
                if (d > 1e-6) {
                    Vector2d(dx / d, dy / d)
                } else {
                    Vector2d(1.0, 0.0)
                }
            } else {
                Vector2d(cos(sample.heading), sin(sample.heading))
            }

            return LinearContour(
                lineP = Vector2d(sample.x, sample.y),
                lineD = dir,
                targetTheta = sample.heading,
                targetOmega = sample.omega,
            )
        }
    }
}

enum class ContourSelectionMode {
    POSITION,
    TIME,
}

/**
 * MPC tuning parameters matching the mecanum_mpc formulation.
 * See gen_mpc_native.py for cost function details.
 *
 * Cost = w_normal * normal_err² + w_lag * lag_err² + w_heading * heading_err²
 *      + w_u * ||u||² + w_du * ||u - u_last||²
 *
 * Tuning guidelines:
 * - wNormal: High values (1000-10000) keep robot close to path. Start high, reduce if too aggressive.
 * - wLag: Moderate values (100-1000) control how tightly robot follows timing. Lower = more relaxed.
 * - wHeading: Lower values (50-500) for heading. Too high causes oscillation when path changes direction.
 * - wU: Control effort penalty (1-50). Higher = slower, smoother movements. Prevents motor saturation.
 * - wDU: Control rate penalty (10-200). CRITICAL for smoothness. Higher = less jitter but slower response.
 *
 * For jitter issues: Increase wDU first (try 50-100), then wU (try 10-30).
 */
data class MPCTuning(
    /** Weight for normal error (perpendicular to path) */
    var wNormal: Double = 500.0,
    /** Weight for lag error (along path direction) */
    var wLag: Double = 500.0,
    /** Weight for heading error */
    var wHeading: Double = 200.0,
    /** Weight for control effort (penalizes torque) */
    var wU: Double = 0.2,
    /** Weight for control rate (penalizes rapid changes - reduces jitter) */
    var wDU: Double = 10.0,
) {
    fun toArray(): DoubleArray = doubleArrayOf(
        wNormal, wLag, wHeading, wU, wDU
    )
}

class MPCClient(
    val parameters: MecanumParameters,
    SOLVER_IP: String = "172.29.0.1",
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
        const val N: Int = 15
        const val K: Int = N + 1  // Number of knot points
        const val NX: Int = 6
        const val NU: Int = 3  // drive, strafe, turn
        const val NP: Int = 7
        const val NTUNING: Int = 5  // [w_normal, w_lag, w_heading, w_u, w_du]
        const val N_CONTOUR_PER_KNOT: Int = 6
        const val N_CONTOUR_PARAMS: Int = K * N_CONTOUR_PER_KNOT  // 96 doubles
        val DT: Duration = 40.milliseconds

        val horizon = DT * N

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

    var lastTargetContour: LinearContour? = null
        private set

    var predictedEvolution: List<MecanumState> = emptyList()
        private set

    var path: List<LinearContour>? = null
        private set

    /** The last set of K contours sent to the MPC solver (interpolated at DT intervals). */
    var lastSentContours: List<LinearContour> = emptyList()
        private set

    private var trajectory: TrajoptTrajectory? = null
    private var referencePositions: List<Vector2d> = emptyList()
    private var timestamps: List<Double> = emptyList()
    private var trajectoryStartTime: Duration? = null

    private var sentTargetContours: MutableList<Pair<Long, LinearContour?>> = mutableListOf()
    private var curTime = 0.seconds

    private val HEADER_SIZE: Int = 4 + 2 + 2 + 4 + 8  // 20 bytes
    private val DOUBLE_SIZE: Int = 8
    private val REQUEST_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (N_CONTOUR_PARAMS + NX + NU + NP + NTUNING)
    private val RESPONSE_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (1 + N * NU)

    private var x0: MecanumState? = null
    private var V: Double = parameters.motor.vRef

    private var latestU: DoubleArray = DoubleArray(N * NU)
    private var latestUTime = 0L

    // Client-side slew rate limiters (one per control channel)
    private val slewLimiters = arrayOf(
        SlewRateLimiter(5.0),  // drive
        SlewRateLimiter(5.0),  // strafe
        SlewRateLimiter(5.0),  // turn
    )
    private var lastGetUTime: Duration? = null

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


        if (trajectoryStartTime == null) {
            trajectoryStartTime = time
        }

        val elapsed = (time - trajectoryStartTime!!).toDouble(DurationUnit.SECONDS)

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
                dx * dx + dy * dy + dt*2.0
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
        val contours = path ?: return LinearContour(Vector2d(), Vector2d(1.0, 0.0), 0.0, 0.0)
        if (contours.isEmpty()) return LinearContour(Vector2d(), Vector2d(1.0, 0.0), 0.0, 0.0)

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
     * Each knot has N_CONTOUR_PER_KNOT (6) values.
     * Contours are interpolated at DT time intervals from the start time.
     */
    private fun buildContourParams(startTime: Double): DoubleArray {
        val params = DoubleArray(N_CONTOUR_PARAMS)
        val contours = path ?: return params
        if (contours.isEmpty()) return params

        val dtSeconds = DT.toDouble(DurationUnit.SECONDS)
        val interpolatedContours = mutableListOf<LinearContour>()

        // Unwrap headings to ensure continuity
        var lastTheta = interpolateContourAt(startTime).targetTheta

        for (k in 0 until K) {
            val time = startTime + k * dtSeconds
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
            return
        }
        if (referencePositions.isEmpty()) {
            return
        }

        // Predicted state
        val predictedX = predictState(x0!!, time.nanoseconds + preIntegrate)
        val predictedU = getU(time.nanoseconds + preIntegrate)

        val selection = selectTarget(predictedX.toDoubleArray(), time.nanoseconds) ?: return
        lastTargetSelection = selection
        val targetContour = path?.getOrNull(selection.targetIndex) ?: return

        // Compute the start time for contour interpolation
        val startTime = if (timestamps.isNotEmpty() && selection.targetIndex < timestamps.size) {
            timestamps[selection.targetIndex]
        } else {
            // Fallback: estimate time from index
            selection.targetIndex * DT.toDouble(DurationUnit.SECONDS)
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

        // Contour params (K * 6 doubles) - interpolated at DT intervals
        putArray(buf, buildContourParams(startTime))

        // Unnormalize heading so MPC turns in shortest direction
        val stateArray = predictedX.toDoubleArray()
        stateArray[5] = unnormalizeHeading(stateArray[5], targetContour.targetTheta)

        putArray(buf, stateArray)  // x0 (6 doubles)
        putArray(buf, predictedU)  // u_last (4 doubles)
        putArray(buf, p)           // p (7 doubles)
        putArray(buf, tuning.toArray())  // tuning (6 doubles)

        buf.flip()

        sentTargetContours += time to targetContour

        try {
            if (channel.isOpen) channel.send(buf, solverAddr)
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
     */
    fun setTarget(traj: TrajoptTrajectory) {
        this.trajectory = traj
        setTargetContours(load(traj), traj.samples.map { it.timestamp })
    }

    /**
     * Set the target contours directly.
     */
    fun setTargetContours(path: List<LinearContour>, timestamps: List<Double> = emptyList()) {
        this.path = path
        referencePositions = path.map { it.lineP }
        this.timestamps = timestamps
        trajectoryStartTime = null
        latestSampleI = 0
        lastTargetContour = null
        lastSentContours = emptyList()
        latestU = DoubleArray(N * NU)
    }

    fun startPose(): Pose2d {
        val first = path?.firstOrNull() ?: return Pose2d()
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

        val i = ((delay - preIntegrate - 5.milliseconds) / DT).toInt().coerceIn(0, N - 1)

        var res = latestU.copyOfRange(i * NU, (i + 1) * NU)

        // Apply optional client-side slew rate limiting
        val slewRate = clientSlewRate
        if (slewRate != null) {
            val dt = lastGetUTime?.let { time - it } ?: DT
            lastGetUTime = time

            slewLimiters.forEachIndexed { idx, limiter ->
                limiter.maxRate = slewRate
                res[idx] = limiter.calculate(res[idx], dt)
            }
        }

        return res
    }

    private fun repredictEvolution() {
        val xs = mutableListOf(x0!!)
        for (i in 0 until N) {
            val u = latestU.sliceArray(i * NU..(i + 1) * NU - 1).let {
                val v = model.mecanumInversePowerKinematics(Pose2d(it[0],it[1],it[2]))
                doubleArrayOf(v[0],v[1],v[2],v[3])
            }
            xs += model.integrate(DT.toDouble(DurationUnit.SECONDS), 0.001, u, xs.last())
        }

        predictedEvolution = xs
    }

    fun predictState(x0: MecanumState, time: Duration): MecanumState {
        val t = time - latestUTime.nanoseconds
        var state = x0
        val i = (t.inWholeMilliseconds / DT.inWholeMilliseconds).toInt().coerceIn(0, N - 1)
        if (predictedEvolution.isNotEmpty()) {
            state = predictedEvolution[i]
        }
        val u = latestU.sliceArray(i * NU..(i + 1) * NU - 1).let {
            val v = model.mecanumInversePowerKinematics(Pose2d(it[0],it[1],it[2]))
            doubleArrayOf(v[0],v[1],v[2],v[3])
        }
        return model.integrate((t - i * DT).toDouble(DurationUnit.SECONDS), MECANUM_DT, u, state)
    }

    fun isTrajectoryComplete(): Boolean {
        val traj = trajectory ?: return true
        val startTime = trajectoryStartTime ?: return false

        // Also check if we're at the last sample
        val pathSize = path?.size ?: 0
        if (pathSize > 0 && latestSampleI >= pathSize - 1) {
            return true
        }

        return false
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
     * @param io The IO interface for reading state and writing motor powers
     * @param driveController The drive controller for setting motor powers (optional)
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
