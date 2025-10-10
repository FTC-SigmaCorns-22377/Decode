package sigmacorns.io

import dev.nullftc.choreolib.sample.MecanumSample
import dev.nullftc.choreolib.trajectory.Trajectory
import org.joml.Vector2d
import org.joml.minus
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumDynamics
import sigmacorns.sim.MecanumParameters
import sigmacorns.sim.MecanumState
import java.net.InetSocketAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.channels.DatagramChannel
import java.nio.channels.SelectionKey
import java.nio.channels.Selector
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.withSign
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.nanoseconds
import kotlin.time.DurationUnit

data class Contour(
    var pos: Pose2d,
    var vel: Pose2d,
    var tangent: Double,
    var r: Double,
) {
    fun toArray(): DoubleArray = doubleArrayOf(pos.v.x,pos.v.y,pos.rot,vel.v.x,vel.v.y,vel.rot,tangent,r)
}

object ContourLoader {
    fun load(traj: Trajectory<MecanumSample>): List<Contour> {
        val samples = traj.samples

        val wpts = mutableListOf<Contour>()
        var lastTheta = samples.first().heading

        for (s in samples) {
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
                    -r
                )
            )
        }

        return wpts
    }
}

class MPCClient(
    val parameters: MecanumParameters,
    SOLVER_IP: String = "172.29.0.1",
    SOLVER_PORT: Int = 5000,
    ROBOT_PORT: Int = 22377,
    val sampleLookahead: Int = 3,
): AutoCloseable {
    val N: Int = 7
    val NX: Int = 6
    val NU: Int = 4
    val NT: Int = 8
    val NP: Int = 7
    val DT: Duration = 40.milliseconds

    private val HEADER_SIZE: Int = 4 + 2 + 2 + 4 + 8
    private val DOUBLE_SIZE: Int = 8
    private val REQUEST_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (NT + NX + NU + NP)
    private val RESPONSE_MSG_SIZE: Int = HEADER_SIZE + DOUBLE_SIZE * (1 + N * NU)

    private var x0: DoubleArray? = null
    private var V: Double = parameters.motor.vRef
    private var path: List<Contour>? = null

    private var lastU: DoubleArray = DoubleArray(NU)
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

    private fun findContour(path: List<Contour>, state: DoubleArray): Contour {
        val p = Vector2d(state[3],state[4])

        val i = path.withIndex().minBy { (p-it.value.pos.v).lengthSquared() }.index

        latestSampleI = i

        return path[min(i+sampleLookahead, path.size-1)]
    }

    var seq: Int = 0

    private fun sendRequest(time: Long) {
        if (path == null || x0 == null) {
            return
        }

        val target = findContour(path!!,x0!!)

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
        putArray(buf, x0!!)
        putArray(buf, lastU)
        putArray(buf, p)

        buf.flip()
        channel.send(buf, solverAddr)
    }

    private fun receiveResponse() {
        if (selector.selectNow() > 0) {
            for (key in selector.selectedKeys()) {
                selector.selectedKeys().remove(key)
                if (key.isReadable()) {
                    val buf: ByteBuffer = ByteBuffer.allocate(RESPONSE_MSG_SIZE)
                    buf.order(ByteOrder.LITTLE_ENDIAN)
                    val addr = channel.receive(buf)
                    if (addr != null) {
                        buf.flip()
                        val magic = buf.getInt() // magic 'SOLV'
                        val version = buf.getShort() // version
                        val type = buf.getShort() // type = request
                        val seq = buf.getInt() // sequence number
                        val t = buf.getLong() // timestamp

                        val solveTime = buf.getLong()

                        println("SOLVED solveTime=$solveTime")

                        if (solveTime>=latestUTime) {
                            println("UPDATED LATEST U")
                            latestUTime = solveTime
                            getArray(buf, latestU)
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

    fun setTarget(path: List<Contour>) {
        this.path = path
        latestSampleI = 0
    }

    fun update(state: MecanumState, v: Number, time: Duration) {
        V = v.toDouble()
        x0 = state.toDoubleArray()

        println("sending state=$state")
        println("sending ${x0.contentToString()}")

        println("Sending with ${time.inWholeNanoseconds}")
        sendRequest(time.inWholeNanoseconds)
    }

    fun getU(time: Duration): DoubleArray {
        receiveResponse()

        val delay = time-latestUTime.nanoseconds

        println("MPC DELAY: ${delay.inWholeMilliseconds}ms")

        val i = ((time-latestUTime.nanoseconds)/DT).toInt().coerceIn(0, N-1)

        val res =  latestU.copyOfRange(i*NU, (i+1)*NU)
        lastU = res

        return res
    }

    fun getPredictedEvolution(): List<Pose2d> {
        val model = MecanumDynamics(parameters)

        val xs = mutableListOf(MecanumState(x0!!))
        for(i in 0..N-1) {
            xs += model.integrate(DT.toDouble(DurationUnit.SECONDS),0.001, latestU.sliceArray(i*NU..(i+1)*NU-1), xs.last())
        }

        return xs.map { it.pos }
    }

    override fun close() {
        channel.close()
    }
}
