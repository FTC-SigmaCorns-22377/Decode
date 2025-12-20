package sigmacorns.sim.viz

import io.javalin.Javalin
import io.javalin.websocket.WsContext
import java.util.concurrent.ConcurrentHashMap
import java.io.File
import com.fasterxml.jackson.databind.ObjectMapper

class SimServer(private val port: Int = 8000) {
    private var app: Javalin? = null
    private val sockets = ConcurrentHashMap.newKeySet<WsContext>()
    private val mapper = ObjectMapper()
    private val history = java.util.Collections.synchronizedList(mutableListOf<SimState>())

    fun start() {
        val path1 = "TeamCode/src/main/resources/web"
        val path2 = "src/main/resources/web"
        val staticDir = if (File(path1).exists()) path1 else path2
        println("Serving static files from: ${File(staticDir).absolutePath}")

        app = Javalin.create { config ->
            config.staticFiles.add { staticFiles ->
                staticFiles.hostedPath = "/"
                staticFiles.directory = staticDir
                staticFiles.location = io.javalin.http.staticfiles.Location.EXTERNAL
            }
            config.router.mount {
                it.before { ctx ->
                    if (ctx.path() != "/sim") {
                        println("HTTP Request: ${ctx.method()} ${ctx.path()}")
                    }
                }
            }
        }

        app?.get("/history") { ctx ->
            println("History requested from ${ctx.ip()}, sending ${history.size} states")
            ctx.json(history)
        }

        app?.get("/assets/{filename}") { ctx ->
            val filename = ctx.pathParam("filename")
            println("Asset requested: $filename")
            val p1 = "TeamCode/src/main/assets/$filename"
            val p2 = "src/main/assets/$filename"
            val file = if (File(p1).exists()) File(p1) else File(p2)
            if (file.exists()) {
                ctx.result(file.inputStream())
            } else {
                ctx.status(404).result("Asset not found")
            }
        }

        app?.get("/robotMeshes/{filename}") { ctx ->
            val filename = ctx.pathParam("filename")
            println("Mesh requested: $filename")
            val p1 = "TeamCode/src/main/assets/robotMeshes/$filename"
            val p2 = "src/main/assets/robotMeshes/$filename"
            val file = if (File(p1).exists()) File(p1) else File(p2)
            if (file.exists()) {
                ctx.result(file.inputStream())
            } else {
                println("Mesh not found: $filename")
                ctx.status(404).result("Mesh not found")
            }
        }

        app?.get("/robot.urdf") { ctx ->
            println("URDF requested from ${ctx.ip()}")
            val uPath1 = "TeamCode/src/main/assets/robot.urdf"
            val uPath2 = "src/main/assets/robot.urdf"
            val file = if (File(uPath1).exists()) File(uPath1) else File(uPath2)
            if (file.exists()) {
                ctx.contentType("text/xml")
                ctx.result(file.inputStream())
            } else {
                println("URDF not found!")
                ctx.status(404).result("URDF not found")
            }
        }

        app?.ws("/sim") { ws ->
            ws.onConnect { ctx -> 
                println("WebSocket connected: ${ctx.session.remoteAddress}")
                sockets.add(ctx) 
            }
            ws.onClose { ctx -> 
                println("WebSocket closed: ${ctx.session.remoteAddress}, reason: ${ctx.reason()}")
                sockets.remove(ctx) 
            }
            ws.onError { ctx ->
                println("WebSocket error: ${ctx.error()}")
            }
            ws.onMessage { ctx ->
                // println("Received WS message: ${ctx.message()}")
            }
        }

        app?.start(port)
        println("SimServer started on http://localhost:$port")
    }

    fun stop() {
        app?.stop()
    }

    private var lastBroadcastLog = 0L
    fun broadcast(state: SimState) {
        history.add(state)
        val now = System.currentTimeMillis()
        if (now - lastBroadcastLog > 5000) {
            println("Broadcasting state, t=${state.t}, connected clients: ${sockets.size}, history size: ${history.size}")
            lastBroadcastLog = now
        }
        val json = mapper.writeValueAsString(state)
        sockets.forEach { 
            if (it.session.isOpen) {
                it.send(json) 
            }
        }
    }
}

data class SimState(
    val t: Double,
    val base: BaseState,
    val joints: Map<String, Double>,
    val telemetry: TelemetryState,
    val balls: List<BallState> = emptyList()
)

data class BallState(val x: Double, val y: Double, val z: Double)

data class BaseState(
    val x: Double,
    val y: Double,
    val z: Double,
    val roll: Double,
    val pitch: Double,
    val yaw: Double
)

data class TelemetryState(
    val fl: Double,
    val fr: Double,
    val bl: Double,
    val br: Double,
    val flywheel: Double,
    val turret: Double
)
