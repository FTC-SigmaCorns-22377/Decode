package sigmacorns.io

import dev.nullftc.choreolib.sample.MecanumSample
import dev.nullftc.choreolib.trajectory.Trajectory
import org.joml.Vector2d
import org.joml.dot
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
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.withSign
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.nanoseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit
import kotlin.time.times

data class Contour(
    var pos: Pose2d,
    var vel: Pose2d,
    var tangent: Double,
    var r: Double,
    var a: Vector2d
) {
    fun toArray(): DoubleArray = doubleArrayOf(pos.v.x,pos.v.y,pos.rot,vel.v.x,vel.v.y,vel.rot,tangent,r,a.x,a.y)
    fun toStateArray(): DoubleArray = doubleArrayOf(vel.v.x, vel.v.y, vel.rot, pos.v.x, pos.v.y, pos.rot)
}

enum class SolverRequestType(val wireValue: Short) {
    CONTOURING(1),
    TRACKING(3),
}

class MPCClient(
    val parameters: MecanumParameters,
    SOLVER_IP: String = "172.29.0.1",
    SOLVER_PORT: Int = 5000,
    ROBOT_PORT: Int = 22377,
    val sampleLookahead: Int = 0,
    val preIntegrate: Duration = 10.milliseconds
): AutoCloseable {

    companion object {
        val N: Int = 7
        val NX: Int = 6
        val NU: Int = 4
        val NT: Int = 10
        val NP: Int = 7
        val DT: Duration = 40.milliseconds

        val horizon = DT*N

        // Choreo uses bottom-left origin, but this project uses center origin
        // FTC field is 3.6576m (12 feet), so we subtract half to center the coordinates
        private const val FIELD_HALF_SIZE = 1.8288 // meters

        fun load(traj: Trajectory<MecanumSample>): List<Contour> {
            if (traj.samples.isEmpty()) return emptyList()

            val samples = traj.samples

            val wpts = mutableListOf<Contour>()
            var lastTheta = samples.first().heading

            for ((i,s) in samples.withIndex()) {
                // Transform from Choreo's bottom-left origin to center origin
                val p = Vector2d(s.x - FIELD_HALF_SIZE, s.y - FIELD_HALF_SIZE)
                val v = Vector2d(s.vx, s.vy)
                val a = Vector2d(s.ax, s.ay)

                var cross = v[0] * a[1] - v[1] * a[0]
                if (abs(cross) < 0.01) {
                    cross = 0.01.withSign(cross)
                }

                var r = hypot(v[0], v[1]).pow(3) / cross

                if (abs(r) > 1000) {
                    r = 1000.0.withSign(r)
                }
                if (abs(r) < 0.05) {
                    r = 0.05.withSign(r)
                }

                var theta = s.heading

                // Unwrap heading so it's continuous
                while (theta - lastTheta > PI) {
                    theta -= PI * 2.0
                }
                while (theta - lastTheta < -PI) {
                    theta += PI * 2.0
                }

                lastTheta = theta

                // use average acceleration over the time horizon instead of the instantaneous acceleration
                val endHorizonSample = traj.sampleAt(s.timestamp + horizon.toDouble(DurationUnit.SECONDS)) ?: traj.getFinalSample()!!

                var ax = (endHorizonSample.vx - s.vx)/(endHorizonSample.timestamp-s.timestamp)
                var ay = (endHorizonSample.vy - s.vy)/(endHorizonSample.timestamp-s.timestamp)

                if(s == traj.getFinalSample()) {
                    ax = 0.0
                    ay = 0.0
                }

                wpts.add(
                    Contour(
                        Pose2d(
                            p,
                            theta,
                        ),
                        Pose2d(
                            v,
                            s.omega
                        ),
                        atan2(v[1], v[0]),
                        -r,
                        Vector2d(ax,ay)
                    )
                )
            }

            return wpts
        }
    }

    val model = MecanumDynamics(parameters)

    var lastTargetContour: Contour? = null
        private set

    var predictedEvolution: List<MecanumState> = emptyList()
        private set

    var path: List<Contour>? = null
        private set

    private var trackingReferenceStates: List<DoubleArray> = emptyList()
    private var referencePositions: List<Pose2d> = emptyList()

    private var sentTargetContours: MutableList<Pair<Long, Contour?>> = mutableListOf()
    private var curTime = 0.seconds

    private val HEADER_SIZE: Int = 4 + 2 + 2 + 4 + 8
    private val DOUBLE_SIZE: Int = 8
    private val TRACKING_STATE_COUNT: Int = (N + 1) * NX
    private val CONTOURING_REQUEST_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (NT + NX + NU + NP)
    private val TRACKING_REQUEST_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (TRACKING_STATE_COUNT + NX + NU + NP)
    private val RESPONSE_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (1 + N * NU)

    private var x0: MecanumState? = null
    private var V: Double = parameters.motor.vRef

    private var latestU: DoubleArray = DoubleArray(N*NU)
    private var latestUTime = 0L

    private var latestSampleI = 0
    private var lastTrackingHorizonPositions: List<Vector2d> = emptyList()

    // Monotonic progression tracking
    private var confirmedProgressIndex = 0
    private var consecutiveForwardSteps = 0

    // Search window parameters
    private val SEARCH_WINDOW_SIZE = 10
    private val BACKWARDS_TOLERANCE = 5
    private val RECOVERY_THRESHOLD = 1.5  // meters

    // Scoring weights
    private val DISTANCE_WEIGHT = 1.0
    private val HEADING_WEIGHT = 0.0  // Increased to better distinguish at intersections
    private val VELOCITY_WEIGHT = 0.2
    private val CONFIRMATION_THRESHOLD = 2  // Faster confirmation to track position better

    private val channel = DatagramChannel.open()
    private val solverAddr = InetSocketAddress(SOLVER_IP, SOLVER_PORT)
    private val selector = Selector.open()
    private var solverRequestType: SolverRequestType = SolverRequestType.TRACKING

    init {
        channel.configureBlocking(false)
        channel.bind(InetSocketAddress(ROBOT_PORT))
        channel.register(selector, SelectionKey.OP_READ)
    }

    fun contourErr(contour: Contour, p: Vector2d): Pair<Double, Double> {
        val cx = contour.pos.v[0] + contour.r * sin(contour.tangent)
        val cy = contour.pos.v[1] - contour.r * cos(contour.tangent)

        val dx = p[0] - cx
        val dy = p[1] - cy

        val drx = cos(-contour.tangent) * dx - sin(-contour.tangent) * dy
        val dry = sin(-contour.tangent) * dx + cos(-contour.tangent) * dy

        val progress = contour.r * (sign(contour.r) * (Math.PI / 2.0) - atan2(dry, drx))
        val err = abs(contour.r) - hypot(dx, dy)

        return Pair(progress, err)
    }

    private fun updateProgressState(selectedIndex: Int) {
        if (selectedIndex >= confirmedProgressIndex) {
            consecutiveForwardSteps++
            if (consecutiveForwardSteps >= CONFIRMATION_THRESHOLD) {
                confirmedProgressIndex = selectedIndex
            }
        } else {
            consecutiveForwardSteps = 0
        }
        latestSampleI = selectedIndex
    }

    private data class TargetSelection(
        val closestIndex: Int,
        val targetIndex: Int,
        val contour: Contour?,
    )

    private fun selectTarget(state: DoubleArray): TargetSelection? {
        if (referencePositions.isEmpty()) return null

        val robotPos = Vector2d(state[3], state[4])
        val robotHeading = state[5]
        val robotVel = Vector2d(state[0], state[1])

        // Check if we're far from expected position (recovery mode)
        val inRecovery = if (referencePositions.isNotEmpty()) {
            robotPos.distance(referencePositions[confirmedProgressIndex].v) > RECOVERY_THRESHOLD
        } else false

        // Determine search window
        val searchStart: Int
        val searchEnd: Int
        if (inRecovery) {
            searchStart = max(0, confirmedProgressIndex - 20)
            searchEnd = min(referencePositions.size - 1, confirmedProgressIndex + 100)
        } else {
            // Search around latest position (not confirmed, to be more responsive)
            // but don't go backwards past confirmed progress
            val windowCenter = max(confirmedProgressIndex, latestSampleI)
            searchStart = max(confirmedProgressIndex, windowCenter - BACKWARDS_TOLERANCE)
            searchEnd = min(referencePositions.size - 1, searchStart + SEARCH_WINDOW_SIZE)
        }

        // Find best candidate by scoring

        val bestIndex = (searchStart..searchEnd).minBy { idx ->
            val dist = robotPos.distance(referencePositions[idx].v)

            if (inRecovery) {
                // Recovery mode: prioritize distance and heading
                val pathHeading = when (solverRequestType) {
                    SolverRequestType.CONTOURING -> path!![idx].tangent
                    SolverRequestType.TRACKING -> trackingReferenceStates[idx][5]
                }
                var headingDiff = pathHeading - robotHeading
                while (headingDiff > PI) headingDiff -= 2 * PI
                while (headingDiff < -PI) headingDiff += 2 * PI
                val headingScore = abs(headingDiff)

                val score = dist * 3.0 + headingScore
                score
            } else {
                // Normal mode: multi-criteria scoring

                // Heading score
                var headingDiff = referencePositions[idx].rot - robotHeading
                while (headingDiff > PI) headingDiff -= 2 * PI
                while (headingDiff < -PI) headingDiff += 2 * PI
                val headingScore = abs(headingDiff)

                // Velocity score
                val v = when (solverRequestType) {
                    SolverRequestType.CONTOURING -> {
                       path!![idx].vel.v
                    }
                    SolverRequestType.TRACKING -> {
                        Vector2d(trackingReferenceStates[idx][0], trackingReferenceStates[idx][1])
                    }
                }
                val robotSpeed = robotVel.length()
                val pathSpeed = v.length()
                val velocityScore = if (robotSpeed < 0.01 || pathSpeed < 0.01) {
                    0.0
                } else {
                    val similarity = robotVel.dot(v) / (robotSpeed * pathSpeed)
                    1.0 - similarity
                }

                val score = (
                    DISTANCE_WEIGHT * dist +
                    HEADING_WEIGHT * headingScore +
                    VELOCITY_WEIGHT * velocityScore
                )

                score
            }
        }

        val closestIndex = bestIndex

        updateProgressState(closestIndex)

        val targetIndex = min(closestIndex + sampleLookahead, referencePositions.size - 1)

        val contour = if (solverRequestType == SolverRequestType.CONTOURING) {
            path?.getOrNull(targetIndex)
        } else {
            null
        }

        return TargetSelection(closestIndex, targetIndex, contour)
    }

    private fun buildTrackingReferenceStates(traj: Trajectory<MecanumSample>): List<DoubleArray> {
        if (traj.samples.isEmpty()) return emptyList()

        val states = ArrayList<DoubleArray>(traj.samples.size)
        var lastTheta = traj.samples.first().heading

        for (sample in traj.samples) {
            var theta = sample.heading
            while (theta - lastTheta > PI) {
                theta -= PI * 2.0
            }
            while (theta - lastTheta < -PI) {
                theta += PI * 2.0
            }
            lastTheta = theta

            states += doubleArrayOf(
                sample.vx,
                sample.vy,
                sample.omega,
                sample.x - FIELD_HALF_SIZE,  // Transform from Choreo's bottom-left origin to center origin
                sample.y - FIELD_HALF_SIZE,  // Transform from Choreo's bottom-left origin to center origin
                theta,
            )
        }

        return states
    }

    var seq: Int = 0

    private fun sendRequest(time: Long) {
        if (x0 == null) {
            return
        }

        if (referencePositions.isEmpty()) {
            return
        }

        val selection = selectTarget(x0!!.toDoubleArray()) ?: return
        val targetContour = selection.contour

        if (solverRequestType == SolverRequestType.CONTOURING && targetContour == null) {
            return
        }
        if (solverRequestType == SolverRequestType.TRACKING && trackingReferenceStates.isEmpty()) {
            return
        }

        val requestSize = when (solverRequestType) {
            SolverRequestType.CONTOURING -> CONTOURING_REQUEST_MSG_SIZE
            SolverRequestType.TRACKING -> TRACKING_REQUEST_MSG_SIZE
        }

        val buf: ByteBuffer = ByteBuffer.allocate(requestSize)
        buf.order(ByteOrder.LITTLE_ENDIAN)

        // Header
        buf.putInt(0x564C4F53) // magic 'SOLV'
        buf.putShort(1.toShort()) // version
        buf.putShort(solverRequestType.wireValue) // type = request
        buf.putInt(seq) // sequence number
        seq += 1
        buf.putLong(time) // timestamp

        val p = parameters.toArray()
        p[0] *= V/parameters.motor.vRef
        p[1] *= V/parameters.motor.vRef

        val headingAnchor = x0!!.pos.rot

        // Payload
        when (solverRequestType) {
            SolverRequestType.CONTOURING -> {
                val target = targetContour!!.toArray()
                target[2] = unwrapAngleNear(target[2], headingAnchor)
                putArray(buf, target)
                lastTrackingHorizonPositions = emptyList()
            }
            SolverRequestType.TRACKING -> {
                val expected = buildExpectedStateHorizon(selection.targetIndex)
                val adjusted = adjustTrackingHeading(expected, headingAnchor)
                lastTrackingHorizonPositions = buildTrackingHorizonPositions(adjusted)
                putArray(buf, adjusted)
            }
        }

        val predictedX = predictState(x0!!,time.nanoseconds+preIntegrate)
        val predictedU = getU(time.nanoseconds+preIntegrate)

        val predictedXArr = predictedX.toDoubleArray()
        predictedXArr[5] = unwrapAngleNear(predictedXArr[5], headingAnchor)
        putArray(buf, predictedXArr)
        putArray(buf, predictedU)
        putArray(buf, p)

        buf.flip()

        sentTargetContours += time to targetContour

        try {
            if(channel.isOpen) channel.send(buf, solverAddr)
        } catch (e: ClosedByInterruptException) {
            channel.close()
        }
    }

    private fun buildExpectedStateHorizon(startIndex: Int): DoubleArray {
        val expected = DoubleArray(TRACKING_STATE_COUNT)
        if (trackingReferenceStates.isEmpty()) {
            return expected
        }
        val lastIndex = trackingReferenceStates.size - 1
        for (offset in 0..N) {
            val state = trackingReferenceStates[min(startIndex + offset, lastIndex)]
            for (i in state.indices) {
                expected[offset * NX + i] = state[i]
            }
        }

        return expected
    }

    private fun unwrapAngleNear(angle: Double, reference: Double): Double {
        var unwrapped = angle
        val twoPi = 2.0 * PI
        while (unwrapped - reference > PI) {
            unwrapped -= twoPi
        }
        while (unwrapped - reference < -PI) {
            unwrapped += twoPi
        }
        return unwrapped
    }

    private fun adjustTrackingHeading(expected: DoubleArray, anchorHeading: Double): DoubleArray {
        if (expected.isEmpty()) {
            return expected
        }
        val adjusted = expected.copyOf()
        val headingIndex = 5
        var prevHeading = unwrapAngleNear(adjusted[headingIndex], anchorHeading)
        adjusted[headingIndex] = prevHeading
        val stateCount = adjusted.size / NX
        for (i in 1 until stateCount) {
            val idx = i * NX + headingIndex
            val raw = adjusted[idx]
            val unwrapped = unwrapAngleNear(raw, prevHeading)
            adjusted[idx] = unwrapped
            prevHeading = unwrapped
        }
        return adjusted
    }

    private fun buildTrackingHorizonPositions(expected: DoubleArray): List<Vector2d> {
        val points = ArrayList<Vector2d>(N + 1)
        val stateCount = expected.size / NX
        for (i in 0 until stateCount) {
            val idx = i * NX
            points.add(Vector2d(expected[idx + 3], expected[idx + 4]))
        }
        return points
    }

    fun getLastTrackingHorizonPositions(): List<Vector2d> = lastTrackingHorizonPositions

    private fun receiveResponse() {
        if (selector.selectNow() > 0) {
            for (key in selector.selectedKeys()) {
                selector.selectedKeys().remove(key)
                if (key.isReadable) {
                    val buf: ByteBuffer = ByteBuffer.allocate(RESPONSE_MSG_SIZE)
                    buf.order(ByteOrder.LITTLE_ENDIAN)
                    var addr: SocketAddress? = null
                    try {
                        if(channel.isOpen) addr = channel.receive(buf)
                    } catch (_: ClosedByInterruptException) {
                        channel.close()
                    }
                    if (addr != null) {
                        buf.flip()
                        val magic = buf.getInt() // magic 'SOLV'
                        require(magic == 0x564C4F53)
                        val version = buf.getShort() // version
                        val type = buf.getShort() // type = request
                        val seq = buf.getInt() // sequence number
                        val t = buf.getLong() // timestamp

                        val t0 = buf.getLong()

                        println("SOLVED t0=$t0")

                        if (t0>=latestUTime) {
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

    fun setTarget(
        traj: Trajectory<MecanumSample>,
        requestType: SolverRequestType = solverRequestType,
    ) {
        solverRequestType = requestType
        when (requestType) {
            SolverRequestType.CONTOURING -> setTargetContours(load(traj))
            SolverRequestType.TRACKING -> setTargetTracking(buildTrackingReferenceStates(traj))
        }
    }

    fun setTargetContours(path: List<Contour>) {
        this.path = path
        solverRequestType = SolverRequestType.CONTOURING
        trackingReferenceStates = emptyList()
        referencePositions = path.map { it.pos }
        latestSampleI = 0
        confirmedProgressIndex = 0
        consecutiveForwardSteps = 0
        lastTargetContour = null
        latestU = DoubleArray(N * NU)
    }

    fun setTargetTracking(states: List<DoubleArray>) {
        solverRequestType = SolverRequestType.TRACKING
        this.path = null
        trackingReferenceStates = states.map { it.copyOf() }
        referencePositions = trackingReferenceStates.map { Pose2d(it[3], it[4], it[5]) }
        latestSampleI = 0
        confirmedProgressIndex = 0
        consecutiveForwardSteps = 0
        lastTargetContour = null
        latestU = DoubleArray(N * NU)
    }

    fun startPose(): Pose2d {
        return when (solverRequestType) {
            SolverRequestType.CONTOURING -> path!![0].pos
            SolverRequestType.TRACKING -> trackingReferenceStates[0].let {
                Pose2d(it[3],it[4],it[5])
            }
        }
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

        val delay = time-latestUTime.nanoseconds

        println("MPC DELAY: ${delay.inWholeMilliseconds}ms (true) ${(delay-preIntegrate).inWholeMilliseconds}ms (effective)")

        val i = ((delay-preIntegrate)/DT).toInt().coerceIn(0, N-1)

        val res =  latestU.copyOfRange(i*NU, (i+1)*NU)

        return res
    }

    private fun repredictEvolution() {
        val xs = mutableListOf(x0!!)
        for(i in 0..N-1) {
            xs += model.integrate(DT.toDouble(DurationUnit.SECONDS),0.001, latestU.sliceArray(i*NU..(i+1)*NU-1), xs.last())
        }

        predictedEvolution = xs
    }

    fun predictState(x0: MecanumState, time: Duration): MecanumState {
        val t = time-latestUTime.nanoseconds
        var x0 = x0
        val i = (t.inWholeMilliseconds/DT.inWholeMilliseconds).toInt().coerceIn(0,N-1)
        if(!predictedEvolution.isEmpty()) {
            x0 = predictedEvolution[i]
        }
        val u = latestU.sliceArray(i*NU..(i+1)*NU-1)
        return model.integrate((t-i*DT).toDouble(DurationUnit.SECONDS), MECANUM_DT,u,x0)
    }

    override fun close() {
        if(channel.isOpen) {
            channel.close()
        }
    }
}
