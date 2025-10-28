package sigmacorns.io

import dev.nullftc.choreolib.sample.MecanumSample
import dev.nullftc.choreolib.trajectory.Trajectory
import org.joml.Vector2d
import org.joml.minus
import org.joml.plus
import org.joml.times
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
}

class MPCClient(
    val parameters: MecanumParameters,
    SOLVER_IP: String = "172.29.0.1",
    SOLVER_PORT: Int = 5000,
    ROBOT_PORT: Int = 22377,
    val sampleLookahead: Int = 0,
    val preIntegrate: Duration = 30.milliseconds
): AutoCloseable {

    companion object {
        val N: Int = 7
        val NX: Int = 6
        val NU: Int = 4
        val NT: Int = 10
        val NP: Int = 7
        val DT: Duration = 40.milliseconds

        val horizon = DT*N

        fun load(traj: Trajectory<MecanumSample>): List<Contour> {
            if (traj.samples.isEmpty()) return emptyList()

            val samples = traj.samples

            val wpts = mutableListOf<Contour>()
            var lastTheta = samples.first().heading

            for ((i,s) in samples.withIndex()) {
                val p = Vector2d(s.x, s.y)
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

    private var sentTargetContours: MutableList<Pair<Long, Contour>> = mutableListOf()
    private var curTime = 0.seconds

    private val HEADER_SIZE: Int = 4 + 2 + 2 + 4 + 8
    private val DOUBLE_SIZE: Int = 8
    private val REQUEST_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (NT + NX + NU + NP)
    private val RESPONSE_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (1 + N * NU)

    private var x0: MecanumState? = null
    private var V: Double = parameters.motor.vRef

    private var latestU: DoubleArray = DoubleArray(N*NU)
    private var latestUTime = 0L

    private var latestSampleI = 0

    private val channel = DatagramChannel.open()
    private val solverAddr = InetSocketAddress(SOLVER_IP, SOLVER_PORT)
    private val selector = Selector.open()

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

    private fun findContour(path: List<Contour>, state: DoubleArray): Contour {
        val p = Vector2d(state[3],state[4])

        // select the contour with the lowest normal error
//        val i = path.windowed(2, partialWindows = true).withIndex().minBy { (i,it) ->
//            val last = it.size == 1
//            // only consider the part of the arc before the start of the next arc
//            val maxProgress = if(last) Double.MAX_VALUE else contourErr(it[0],it[1].pos.v).first
//            val err = contourErr(it[0], p)
//
//            if((err.first <= maxProgress) && (err.first >= 0 || i == 0) || last) abs(err.second) else Double.MAX_VALUE
//        }.index
//        println("I=$i")

        val i = path.withIndex().minBy { (p-it.value.pos.v).lengthSquared() }.index

        latestSampleI = i

        return path[min(i+sampleLookahead, path.size-1)]
    }

    var seq: Int = 0

    private fun sendRequest(time: Long) {
        if (path == null || x0 == null) {
            return
        }

        val target = findContour(path!!,x0!!.toDoubleArray())

        val buf: ByteBuffer = ByteBuffer.allocate(REQUEST_MSG_SIZE)
        buf.order(ByteOrder.LITTLE_ENDIAN)

        // Header
        buf.putInt(0x564C4F53) // magic 'SOLV'
        buf.putShort(1.toShort()) // version
        buf.putShort(1.toShort()) // type = request
        buf.putInt(seq) // sequence number
        seq += 1
        buf.putLong(time) // timestamp

        val p = parameters.toArray()
        p[0] *= V/parameters.motor.vRef
        p[1] *= V/parameters.motor.vRef

        // Payload
        putArray(buf, target.toArray())

        val predictedX = predictState(x0!!,time.nanoseconds+preIntegrate)
        val predictedU = getU(time.nanoseconds+preIntegrate)

        putArray(buf, predictedX.toDoubleArray())
        putArray(buf, predictedU)
        putArray(buf, p)

        buf.flip()

        sentTargetContours += time to target

        try {
            if(channel.isOpen) channel.send(buf, solverAddr)
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
                            lastTargetContour = sentTargetContours.find { it.first == t0 }!!.second
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

    fun setTarget(traj: Trajectory<MecanumSample>) = setTarget(load(traj))

    fun setTarget(path: List<Contour>) {
        this.path = path
        latestSampleI = 0
        lastTargetContour = null
        latestU = DoubleArray(N*NU)
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
