package sigmacorns.sim.viz

import com.google.gson.Gson
import io.javalin.Javalin
import io.javalin.websocket.WsContext
import sigmacorns.io.JoltSimIO
import java.util.concurrent.ConcurrentHashMap

class SimVizServer(
    private val simIO: JoltSimIO,
    private val port: Int = 8080
) {
    private val app: Javalin = Javalin.create { config ->
        config.staticFiles.add("/web")
    }
    private val clients = ConcurrentHashMap.newKeySet<WsContext>()
    private val gson = Gson()

    fun start(): SimVizServer {
        app.ws("/sim") { ws ->
            ws.onConnect { ctx -> clients.add(ctx) }
            ws.onClose { ctx -> clients.remove(ctx) }
        }
        app.start(port)
        return this
    }

    fun stop() {
        app.stop()
    }

    fun broadcastState() {
        if (clients.isEmpty()) return

        val robotState = FloatArray(6)
        sigmacorns.sim.JoltNative.nativeGetRobotState(getHandle(), robotState)

        val balls = simIO.getBallStates()

        val state = mapOf(
            "t" to simIO.time().inWholeMilliseconds / 1000.0,
            "robot" to mapOf(
                "x" to robotState[0],
                "y" to robotState[1],
                "theta" to robotState[2]
            ),
            "balls" to balls.map { b ->
                mapOf("x" to b.x, "y" to b.y, "z" to b.z, "color" to b.color.name.lowercase())
            },
            "heldBalls" to simIO.heldBalls.map { it.name.lowercase() }
        )

        val json = gson.toJson(state)
        clients.removeIf { ctx ->
            try {
                ctx.send(json)
                false
            } catch (_: Exception) {
                true
            }
        }
    }

    private fun getHandle(): Long {
        // Access handle via reflection since it's private
        val field = JoltSimIO::class.java.getDeclaredField("handle")
        field.isAccessible = true
        return field.getLong(simIO)
    }
}
