package sigmacorns.test.jolt

import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.io.JoltSimIO
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.auto.Auto1Test
import sigmacorns.opmode.auto.Auto12Ball
import sigmacorns.opmode.auto.Auto12BallMirrored
import sigmacorns.sim.viz.SimVizServer

class JoltAutoTest {
    private lateinit var sim: JoltSimIO
    private lateinit var server: SimVizServer
    @Volatile private var broadcasting = false
    private var broadcastThread: Thread? = null

    @BeforeEach
    fun setUp() {
        SigmaOpMode.SIM = true
        SigmaOpMode.LIMELIGHT_CONNECTED = false
        sim = JoltSimIO(realTime = true)
        sim.spawnFieldBalls()
        SigmaOpMode.simIO = sim

        server = SimVizServer(sim)
        server.start()
        println("Visualizer at http://localhost:8080 — waiting for client to connect...")
        server.awaitClient()
        Thread.sleep(500)
        println("Client connected.")

        // Broadcast sim state in background so opmodes don't need to know about it
        broadcasting = true
        broadcastThread = Thread {
            while (broadcasting) {
                server.broadcastState()
                Thread.sleep(20) // ~50Hz
            }
        }.apply {
            isDaemon = true
            start()
        }
    }

    @AfterEach
    fun tearDown() {
        broadcasting = false
        broadcastThread?.join(500)
        SigmaOpMode.simIO = null
        server.stop()
        sim.close()
    }

    @Test
    fun testAuto1() {
        Auto1Test().runOpMode()
    }

    @Test
    fun test12Ball() {
        Auto12Ball().runOpMode()
    }

    @Test
    fun test12BallMirrored() {
        Auto12BallMirrored().runOpMode()
    }
}
