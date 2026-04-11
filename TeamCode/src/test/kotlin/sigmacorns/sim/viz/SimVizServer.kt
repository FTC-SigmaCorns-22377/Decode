package sigmacorns.sim.viz

import com.google.gson.Gson
import com.qualcomm.robotcore.hardware.Gamepad
import io.javalin.Javalin
import io.javalin.websocket.WsContext
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.Robot
import sigmacorns.constants.ballExitRadius
import sigmacorns.constants.turretPos
import sigmacorns.control.aim.Ballistics
import sigmacorns.control.aim.Ballistics.ShotState
import sigmacorns.io.JoltSimIO
import sigmacorns.logic.AimConfig
import sigmacorns.subsystem.ShooterConfig
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.CountDownLatch
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

data class WasdState(
    val w: Boolean = false, val a: Boolean = false, val s: Boolean = false, val d: Boolean = false,
    val q: Boolean = false, val e: Boolean = false, val r: Boolean = false, val f: Boolean = false,
    val o: Boolean = false, val p: Boolean = false,
    val n1: Boolean = false, val n2: Boolean = false, val n3: Boolean = false, val n4: Boolean = false,
    val space: Boolean = false, val shift: Boolean = false,
)

/**
 * Browser-side gamepad snapshot (HTML5 Gamepad API in "standard" layout).
 *
 * Sticks and triggers are already in [-1, 1] / [0, 1]; the server does not
 * apply deadzone so opmodes can use their own tuning.
 */
data class BrowserGamepadState(
    val connected: Boolean = false,
    val leftStickX: Float = 0f,
    val leftStickY: Float = 0f,
    val rightStickX: Float = 0f,
    val rightStickY: Float = 0f,
    val leftTrigger: Float = 0f,
    val rightTrigger: Float = 0f,
    val a: Boolean = false, val b: Boolean = false,
    val x: Boolean = false, val y: Boolean = false,
    val leftBumper: Boolean = false, val rightBumper: Boolean = false,
    val back: Boolean = false, val start: Boolean = false,
    val leftStickButton: Boolean = false, val rightStickButton: Boolean = false,
    val dpadUp: Boolean = false, val dpadDown: Boolean = false,
    val dpadLeft: Boolean = false, val dpadRight: Boolean = false,
    val guide: Boolean = false,
)

/** Full input message from the browser: keyboard + up to two gamepads. */
data class BrowserInputState(
    val keys: WasdState = WasdState(),
    val gamepad1: BrowserGamepadState = BrowserGamepadState(),
    val gamepad2: BrowserGamepadState = BrowserGamepadState(),
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
    private val firstClientLatch = CountDownLatch(1)

    @Volatile private var robot: Robot? = null

    private val vizBallistics = Ballistics(
        rH = ballExitRadius,
        vMax = AimConfig.vMax,
        phiMin = Math.toRadians(ShooterConfig.minAngleDeg),
        phiMax = Math.toRadians(ShooterConfig.maxAngleDeg),
        g = AimConfig.g,
    )

    @Volatile var wasdState: WasdState = WasdState()
        private set
    @Volatile var gamepad1State: BrowserGamepadState = BrowserGamepadState()
        private set
    @Volatile var gamepad2State: BrowserGamepadState = BrowserGamepadState()
        private set

    /** Attach a [Robot] so [broadcastState] can publish shot trajectories. */
    fun setRobot(robot: Robot) {
        this.robot = robot
    }

    /**
     * Copy the latest browser-side gamepad state into the given FTC [Gamepad]
     * objects. Call once per loop on the opmode thread before reading gamepads.
     */
    fun syncGamepads(gp1: Gamepad, gp2: Gamepad) {
        applyToGamepad(gp1, gamepad1State)
        applyToGamepad(gp2, gamepad2State)
    }

    private fun applyToGamepad(target: Gamepad, state: BrowserGamepadState) {
        target.left_stick_x  = state.leftStickX
        target.left_stick_y  = state.leftStickY
        target.right_stick_x = state.rightStickX
        target.right_stick_y = state.rightStickY
        target.left_trigger  = state.leftTrigger
        target.right_trigger = state.rightTrigger
        target.a = state.a; target.b = state.b
        target.x = state.x; target.y = state.y
        target.left_bumper  = state.leftBumper
        target.right_bumper = state.rightBumper
        target.back  = state.back
        target.start = state.start
        target.left_stick_button  = state.leftStickButton
        target.right_stick_button = state.rightStickButton
        target.dpad_up    = state.dpadUp
        target.dpad_down  = state.dpadDown
        target.dpad_left  = state.dpadLeft
        target.dpad_right = state.dpadRight
        target.guide      = state.guide
    }

    fun start(): SimVizServer {
        app.ws("/sim") { ws ->
            ws.onConnect { ctx -> clients.add(ctx) }
            ws.onClose { ctx -> clients.remove(ctx) }
            ws.onMessage { ctx ->
                try {
                    firstClientLatch.countDown()
                    val input = gson.fromJson(ctx.message(), BrowserInputState::class.java)
                    wasdState = input.keys
                    gamepad1State = input.gamepad1
                    gamepad2State = input.gamepad2
                } catch (_: Exception) {}
            }
        }
        app.start(port)
        return this
    }

    fun awaitClient() {
        firstClientLatch.await()
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

        val shotViz = buildShotViz()

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
            ),
            "shotViz" to shotViz,
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

    /**
     * Build the shot visualization payload for the current frame.
     *
     * Returns null when there's no attached robot or the flywheel is idle.
     * Otherwise includes the goal marker and up to four parabolic trajectories:
     *   - current: from actual turret/hood + current flywheel ω via omegaInv
     *   - target: solver's primaryShotState applied at the current turret pose
     *   - secondary: solver's secondaryShotState (only when robust is active)
     *   - preposition: primary target applied at the horizon-end predicted pose
     */
    private fun buildShotViz(): Map<String, Any>? {
        val r = robot ?: return null
        val omegaNow = simIO.flywheelVelocity()
        if (abs(omegaNow) < 5.0) return null

        val pose = simIO.position()
        val turretYaw = simIO.turretPosition() + pose.rot
        val hoodAngle = simIO.hoodPosition()

        // Shooter pivot position in field frame (matches JoltSimIO.shootBall).
        val cosRot = cos(pose.rot)
        val sinRot = sin(pose.rot)
        val pivot = Vector3d(
            pose.v.x + turretPos.x * cosRot,
            pose.v.y + turretPos.x * sinRot,
            turretPos.z,
        )

        // simIO.velocity().v is already in field frame (see JoltSimIO/HardwareIO).
        val vField = Vector2d(simIO.velocity().v)
        val stopZ = AimConfig.goalHeight

        val trajectories = mutableListOf<Map<String, Any>>()

        // 1. Current-state trajectory — what a ball would do if fired NOW.
        val vExitNow = AimConfig.omegaInv(omegaNow, hoodAngle)
        val currentShot = ShotState(
            theta = turretYaw,
            phi = hoodAngle,
            vExit = vExitNow,
        )
        trajectories.add(trajToMap(
            kind = "current",
            dashed = false,
            points = vizBallistics.sampleTrajectory(pivot, vField, currentShot, stopZ),
        ))

        // 2. Solver-target trajectory.
        val target = r.aim.primaryShotState
        if (target != null) {
            trajectories.add(trajToMap(
                kind = "target",
                dashed = false,
                points = vizBallistics.sampleTrajectory(pivot, vField, target, stopZ),
            ))
        }

        // 3. Robust secondary (dashed).
        if (r.aim.isRobustActive) {
            val s2 = r.aim.secondaryShotState
            if (s2 != null) {
                trajectories.add(trajToMap(
                    kind = "secondary",
                    dashed = true,
                    points = vizBallistics.sampleTrajectory(pivot, vField, s2, stopZ),
                ))
            }
        }

        // 4. Preposition horizon-end.
        if (r.aim.isPrepositionActive && target != null) {
            val h = AimConfig.prepositionHorizon
            val futurePivot = Vector3d(
                pivot.x + vField.x * h,
                pivot.y + vField.y * h,
                pivot.z,
            )
            trajectories.add(trajToMap(
                kind = "preposition",
                dashed = false,
                points = vizBallistics.sampleTrajectory(futurePivot, vField, target, stopZ),
            ))
        }

        val goal = mapOf(
            "x" to r.aim.goalPosition.x,
            "y" to r.aim.goalPosition.y,
            "z" to AimConfig.goalHeight,
        )

        return mapOf(
            "goal" to goal,
            "trajectories" to trajectories,
        )
    }

    private fun trajToMap(
        kind: String,
        dashed: Boolean,
        points: List<Vector3d>,
    ): Map<String, Any> = mapOf(
        "kind" to kind,
        "dashed" to dashed,
        "points" to points.map { listOf(it.x, it.y, it.z) },
    )

    private fun getHandle(): Long {
        // Access handle via reflection since it's private
        val field = JoltSimIO::class.java.getDeclaredField("handle")
        field.isAccessible = true
        return field.getLong(simIO)
    }
}
