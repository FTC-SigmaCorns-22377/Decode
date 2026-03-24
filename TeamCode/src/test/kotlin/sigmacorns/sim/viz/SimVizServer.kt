package sigmacorns.sim.viz

import com.google.gson.Gson
import io.javalin.Javalin
import io.javalin.websocket.WsContext
import sigmacorns.io.JoltSimIO
import java.util.concurrent.ConcurrentHashMap

data class WasdState(
    val w: Boolean = false, val a: Boolean = false, val s: Boolean = false, val d: Boolean = false,
    val q: Boolean = false, val e: Boolean = false, val r: Boolean = false, val f: Boolean = false,
    val n1: Boolean = false, val n2: Boolean = false, val n3: Boolean = false, val n4: Boolean = false,
    val space: Boolean = false, val shift: Boolean = false,
)

class SimVizServer(
    private val simIO: JoltSimIO,
    private val port: Int = 8080
) {
    private val app: Javalin = Javalin.create { config ->
        config.staticFiles.add("/web")
    }
    private val clients = ConcurrentHashMap.newKeySet<WsContext>()
    private val gson = Gson()

    @Volatile var wasdState: WasdState = WasdState()
        private set

    fun start(): SimVizServer {
        app.ws("/sim") { ws ->
            ws.onConnect { ctx -> clients.add(ctx) }
            ws.onClose { ctx -> clients.remove(ctx) }
            ws.onMessage { ctx ->
                try { wasdState = gson.fromJson(ctx.message(), WasdState::class.java) } catch (_: Exception) {}
            }
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

        val goalState = simIO.getGoalState()

        val state = mapOf(
            "t" to simIO.time().inWholeMilliseconds / 1000.0,
            "robot" to mapOf(
                "x" to robotState[0],
                "y" to robotState[1],
                "theta" to robotState[2],
                "turretAngle" to simIO.turretPosition(),
                "intakeAngle" to simIO.intakeAngle(),
                "intakeRollerRPM" to simIO.intakeRollerVelocity() * 60.0 / (2.0 * Math.PI),
                "hoodAngle" to simIO.hoodPosition(),
                "flywheelRPM" to simIO.flywheelVelocity() * 60.0 / (2.0 * Math.PI)
            ),
            "balls" to balls.map { b ->
                mapOf("x" to b.x, "y" to b.y, "z" to b.z, "color" to b.color.name.lowercase())
            },
            "heldBalls" to simIO.heldBalls.map { it.name.lowercase() },
            "goals" to mapOf(
                "redScore" to goalState.redScore,
                "blueScore" to goalState.blueScore,
                "redGateOpen" to goalState.redGateOpen,
                "blueGateOpen" to goalState.blueGateOpen,
                "redLeverAngle" to goalState.redLeverAngle,
                "blueLeverAngle" to goalState.blueLeverAngle
            )
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
