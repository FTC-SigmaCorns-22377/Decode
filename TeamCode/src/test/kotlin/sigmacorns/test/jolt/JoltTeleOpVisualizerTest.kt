package sigmacorns.test.jolt

import org.junit.jupiter.api.Test
import sigmacorns.io.JoltSimIO
import sigmacorns.opmode.AutoTeleOp
import sigmacorns.opmode.MainTeleOp
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.viz.SimVizServer

/**
 * Runs [MainTeleOp] against the Jolt physics sim with the web visualizer.
 *
 * Flow:
 *   1. Boot JoltSimIO + SimVizServer (serves the Three.js viewer on :8080).
 *   2. Wait for a browser to connect.
 *   3. Launch MainTeleOp on a background thread — its [ioLoop] drives physics
 *      via `io.update()`, so the main thread only needs to broadcast and sync
 *      input.
 *   4. On the main thread, every ~20 ms: copy the latest browser-reported
 *      gamepad state into the opmode's `gamepad1`/`gamepad2`, attach the
 *      opmode's `robotRef` to the server once it's available (so shot
 *      trajectories render), and broadcast sim state to the client.
 *
 * The browser sends both keyboard (WASD etc.) and HTML5-Gamepad-API state in
 * a single WS message at ~60 Hz — see `app.js: sendInput`. Plug a controller
 * into the machine running the browser tab (Chrome must see it via
 * `navigator.getGamepads`) and it'll drive MainTeleOp exactly like the real
 * competition opmode.
 *
 * This test runs until the process is killed — there's no auto-exit, since
 * it's an interactive viz session rather than an assertion-bearing test.
 */
class JoltTeleOpVisualizerTest {

    @Test
    fun testMainTeleOpWithBrowserGamepad() {
        SigmaOpMode.SIM = true
        SigmaOpMode.LIMELIGHT_CONNECTED = false

        val sim = JoltSimIO(realTime = true)
        sim.spawnFieldBalls()
        SigmaOpMode.simIO = sim

        val server = SimVizServer(sim)
        server.start()
        println("Visualizer at http://localhost:8080")
        println("Waiting for a browser client to connect...")
        server.awaitClient()
        Thread.sleep(500)
        println("Client connected. Starting MainTeleOp.")

        val opmode = MainTeleOp()

        // Launch the opmode; its ioLoop will step physics via io.update().
        val opmodeThread = Thread {
            try {
                opmode.runOpMode()
            } catch (t: Throwable) {
                if (t is InterruptedException) return@Thread
                t.printStackTrace()
            }
        }.apply {
            name = "MainTeleOp"
            isDaemon = true
            start()
        }

        // Broadcast + input sync loop on the main thread. Runs until the test
        // process is killed.
        var robotAttached = false
        try {
            while (!Thread.currentThread().isInterrupted) {
                // Push browser gamepads into the opmode's gamepad objects.
                server.syncGamepads(opmode.gamepad1, opmode.gamepad2)

                // Once MainTeleOp has built its Robot, wire it into the server
                // so shot trajectories + the goal marker render.
                if (!robotAttached) {
                    opmode.robotRef?.let {
                        server.setRobot(it)
                        robotAttached = true
                    }
                }

                server.broadcastState()
                Thread.sleep(20)
            }
        } finally {
            opmodeThread.interrupt()
            SigmaOpMode.simIO = null
            server.stop()
            sim.close()
        }
    }
}
