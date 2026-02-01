package sigmacorns.io

import org.joml.Vector2d
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
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin
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
 * MPC tuning parameters matching the new mecanum_mpc formulation.
 * See gen_mpc.py for cost function details.
 */
data class MPCTuning(
    /** Weight for normal error (perpendicular to path) */
    var wNormal: Double = 5000.0,
    /** Weight for lag error (along path direction) */
    var wLag: Double = 500.0,
    /** Weight for heading error */
    var wHeading: Double = 150.0,
    /** Weight for control smoothness (u_diff) */
    var wUDiff: Double = 20.0,
    /** Temporal decay factor for future costs */
    var futureDecay: Double = 0.8,
    /** Scale factor for terminal cost */
    var terminalScale: Double = 2.0,
) {
    fun toArray(): DoubleArray = doubleArrayOf(
        wNormal, wLag, wHeading, wUDiff, futureDecay, terminalScale
    )
}

class MPCClient(
    val parameters: MecanumParameters,
    SOLVER_IP: String = "172.29.0.1",
    SOLVER_PORT: Int = 5000,
    ROBOT_PORT: Int = 22377,
    val sampleLookahead: Int = 0,
    val preIntegrate: Duration = 80.milliseconds,
    var contourSelectionMode: ContourSelectionMode = ContourSelectionMode.POSITION,
    var tuning: MPCTuning = MPCTuning(),
): AutoCloseable {

    companion object {
        const val N: Int = 10
        const val K: Int = N + 1  // Number of knot points
        const val NX: Int = 6
        const val NU: Int = 4
        const val NP: Int = 7
        const val NTUNING: Int = 6
        const val N_CONTOUR_PER_KNOT: Int = 6
        const val N_CONTOUR_PARAMS: Int = K * N_CONTOUR_PER_KNOT  // 66 doubles
        val DT: Duration = 80.milliseconds

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

        val closestIndex = when {
            contourSelectionMode == ContourSelectionMode.TIME && timestamps.isNotEmpty() -> {
                if (trajectoryStartTime == null) {
                    trajectoryStartTime = time
                }
                val elapsed = (time - trajectoryStartTime!!).toDouble(DurationUnit.SECONDS)
                timestamps.withIndex().minBy { (_, t) ->
                    kotlin.math.abs(t - elapsed)
                }.index
            }
            else -> referencePositions.withIndex().minBy { (_, ref) ->
                val dx = ref.x() - state[3]
                val dy = ref.y() - state[4]
                dx * dx + dy * dy
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
     * Build the contour parameters array for all K knots.
     * Each knot has N_CONTOUR_PER_KNOT (6) values.
     */
    private fun buildContourParams(startIndex: Int): DoubleArray {
        val params = DoubleArray(N_CONTOUR_PARAMS)
        val contours = path ?: return params
        if (contours.isEmpty()) return params

        val lastIndex = contours.size - 1
        for (k in 0 until K) {
            val contour = contours[min(startIndex + k, lastIndex)]
            val arr = contour.toArray()
            for (j in 0 until N_CONTOUR_PER_KNOT) {
                params[k * N_CONTOUR_PER_KNOT + j] = arr[j]
            }
        }

        return params
    }

    private fun sendRequest(time: Long) {
        if (x0 == null) {
            return
        }
        if (referencePositions.isEmpty()) {
            return
        }

        val selection = selectTarget(x0!!.toDoubleArray(), time.nanoseconds) ?: return
        val targetContour = path?.getOrNull(selection.targetIndex) ?: return

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

        // Contour params (66 doubles)
        putArray(buf, buildContourParams(selection.targetIndex))

        // Predicted state
        val predictedX = predictState(x0!!, time.nanoseconds + preIntegrate)
        val predictedU = getU(time.nanoseconds + preIntegrate)

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
        x0 = state

        println("sending state=$state")
        println("sending $x0")

        println("Sending with ${time.inWholeNanoseconds}")
        sendRequest(time.inWholeNanoseconds)
    }

    fun getU(time: Duration): DoubleArray {
        receiveResponse()

        val delay = time - latestUTime.nanoseconds

        println("MPC DELAY: ${delay.inWholeMilliseconds}ms (true) ${(delay - preIntegrate).inWholeMilliseconds}ms (effective)")

        val i = ((delay - preIntegrate - 10.milliseconds) / DT).toInt().coerceIn(0, N - 1)

        val res = latestU.copyOfRange(i * NU, (i + 1) * NU)

        return res
    }

    private fun repredictEvolution() {
        val xs = mutableListOf(x0!!)
        for (i in 0 until N) {
            xs += model.integrate(DT.toDouble(DurationUnit.SECONDS), 0.001, latestU.sliceArray(i * NU..(i + 1) * NU - 1), xs.last())
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
        val u = latestU.sliceArray(i * NU..(i + 1) * NU - 1)
        return model.integrate((t - i * DT).toDouble(DurationUnit.SECONDS), MECANUM_DT, u, state)
    }

    override fun close() {
        if (channel.isOpen) {
            channel.close()
        }
    }
}
