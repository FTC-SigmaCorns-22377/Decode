package sigmacorns.sim.viz

import io.javalin.Javalin
import io.javalin.websocket.WsContext
import java.util.concurrent.ConcurrentHashMap
import java.io.File
import com.fasterxml.jackson.databind.ObjectMapper

/**
 * Real SimServer implementation for desktop simulation tests.
 * This uses Javalin and Jackson which are test-only dependencies.
 */
class RealSimServer(private val port: Int = 8000) {
    private var app: Javalin? = null
    private val sockets = ConcurrentHashMap.newKeySet<WsContext>()
    private val mapper = ObjectMapper()
    private val history = java.util.Collections.synchronizedList(mutableListOf<SimState>())
    @Volatile private var choreoPath: List<PathPoint> = emptyList()
    @Volatile private var gamepad1State: GamepadState = GamepadState()
    @Volatile private var gamepad2State: GamepadState = GamepadState()

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
        app?.get("/path") { ctx ->
            ctx.json(choreoPath)
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
                try {
                    val message = ctx.message()
                    val node = mapper.readTree(message)
                    if (node.has("type") && node.get("type").asText() == "gamepad") {
                        val gamepadId = node.get("gamepadId").asInt()
                        val input = node.get("input")
                        val state = GamepadState(
                            left_stick_x = input.get("left_stick_x").asDouble(),
                            left_stick_y = input.get("left_stick_y").asDouble(),
                            right_stick_x = input.get("right_stick_x").asDouble(),
                            right_stick_y = input.get("right_stick_y").asDouble(),
                            left_trigger = input.get("left_trigger").asDouble(),
                            right_trigger = input.get("right_trigger").asDouble(),
                            a = input.get("a").asBoolean(),
                            b = input.get("b").asBoolean(),
                            x = input.get("x").asBoolean(),
                            y = input.get("y").asBoolean(),
                            back = input.get("back").asBoolean(),
                            start = input.get("start").asBoolean(),
                            left_bumper = input.get("left_bumper").asBoolean(),
                            right_bumper = input.get("right_bumper").asBoolean(),
                            left_stick_button = input.get("left_stick_button").asBoolean(),
                            right_stick_button = input.get("right_stick_button").asBoolean(),
                            dpad_up = input.get("dpad_up").asBoolean(),
                            dpad_down = input.get("dpad_down").asBoolean(),
                            dpad_left = input.get("dpad_left").asBoolean(),
                            dpad_right = input.get("dpad_right").asBoolean()
                        )
                        if (gamepadId == 1) {
                            gamepad1State = state
                        } else if (gamepadId == 2) {
                            gamepad2State = state
                        }
                    }
                } catch (e: Exception) {
                    println("Error parsing gamepad message: ${e.message}")
                }
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

    fun setChoreoPath(path: List<PathPoint>) {
        choreoPath = path
    }

    fun getGamepad1(): GamepadState = gamepad1State
    fun getGamepad2(): GamepadState = gamepad2State
}
