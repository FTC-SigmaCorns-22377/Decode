package sigmacorns.io

import org.joml.Vector2d
import sigmacorns.math.Pose2d
import sigmacorns.sim.Balls
import sigmacorns.sim.BallInteractionSimulator
import sigmacorns.sim.DrakeRobotModel
import sigmacorns.sim.viz.SimServer
import sigmacorns.sim.viz.SimState
import sigmacorns.sim.viz.BaseState
import sigmacorns.sim.viz.TelemetryState
import sigmacorns.sim.viz.BallState
import sigmacorns.sim.viz.ErrorState
import sigmacorns.sim.viz.ForceState
import sigmacorns.sim.viz.PathPoint
import com.qualcomm.robotcore.hardware.Gamepad
import sigmacorns.sim.viz.GamepadState
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

class DrakeSimIO(urdfPath: String) : SigmaIO {
    val model = DrakeRobotModel(urdfPath)
    val server: Any
    private val ballSim = BallInteractionSimulator(model)
    private var trackingError: ErrorState? = null
    private var trackingTarget: List<PathPoint> = emptyList()

    private var gamepad1: Gamepad? = null
    private var gamepad2: Gamepad? = null

    private var t = 0.seconds
    private val SIM_UPDATE_TIME = 30.milliseconds

    init {
        // Try to use RealSimServer if available (test environment), otherwise use stub
        server = try {
            val realServerClass = Class.forName("sigmacorns.sim.viz.RealSimServer")
            val constructor = realServerClass.getConstructor(Int::class.java)
            val instance = constructor.newInstance(8080)
            val startMethod = realServerClass.getMethod("start")
            startMethod.invoke(instance)
            println("DrakeSimIO: Using RealSimServer (test environment)")
            instance
        } catch (e: ClassNotFoundException) {
            println("DrakeSimIO: RealSimServer not available, using stub SimServer (robot deployment)")
            val stubServer = SimServer(8080)
            stubServer.start()
            stubServer
        }

        // Spawn initial field balls at artifact pickup locations
        // Based on DECODE field positions (converted to meters from Pedro inches)
        spawnInitialBalls()
    }

    private fun spawnInitialBalls() {
        // Convert Pedro coordinates (inches) to meters (Drake uses meters)
        val inchesToMeters = 0.0254
        val tile = inchesToMeters * 24.0
        val ballOffset = BALL_RADIUS*2.1

        listOf(false,true).forEach { blue ->
            listOf(0.5,-0.5,-1.5).forEachIndexed { green, y ->
                var offsets = listOf(-1.0 * ballOffset, 0.0, 1.0*ballOffset)
                if(!blue) offsets = offsets.reversed()
                offsets.forEachIndexed { i,it ->
                    val x = it + if(blue) -2.0*tile else 2.0*tile
                    val color = if(i==green) Balls.Green else Balls.Purple
                    model.spawnBall(x,y*tile,BALL_RADIUS, color)
                }
            }
        }
    }

    override var driveFL: Double = 0.0
    override var driveBL: Double = 0.0
    override var driveFR: Double = 0.0
    override var driveBR: Double = 0.0
    override var shooter: Double = 0.0
    override var intake: Double = 0.0
    override var turret: Double = 0.0
    override var spindexer: Double = 0.0
    override var turretAngle: Double = 0.0
    override var breakPower: Double = 0.0
    override var transfer: Double = 0.0

    override fun position(): Pose2d = model.drivetrainState.pos
    override fun velocity(): Pose2d = model.drivetrainState.vel
    override fun flywheelVelocity(): Double = model.flywheelState.omega

    override fun turretPosition(): Double =
        model.jointPositions["turret_joint"] ?: 0.0

    override fun spindexerPosition(): Double =
        model.jointPositions["spindexer_joint"] ?: 0.0

    override fun distance(): Double {
        return if(ballSim.colorSensorDetectsBall()) 0.0 else Double.MAX_VALUE
    }

    override fun update() {
        // Measure start time for real-time sync
        val startTime = System.nanoTime()

        // Update gamepads from web interface
        updateGamepadsFromServer()

        // Step Simulation
        model.advanceSim(SIM_UPDATE_TIME.toDouble(DurationUnit.SECONDS), this)
        t += SIM_UPDATE_TIME

        // Update ball interaction simulation
        ballSim.update(
            robotPose = model.drivetrainState.pos,
            robotVelocity = model.drivetrainState.vel,
            intakePower = intake,
            transferPower = transfer,
            spindexerAngle = model.jointPositions["spindexer_joint"] ?: 0.0,
            turretYaw = model.jointPositions["turret_joint"] ?: 0.0,
            hoodPitch = model.jointPositions["hood_joint"] ?: 0.0,
            flywheelOmega = model.flywheelState.omega,
            dt = SIM_UPDATE_TIME
        )

        // Broadcast State
        val state = model.drivetrainState
        val fw = model.flywheelState

        // Combine Drake ball positions with spindexer ball positions for visualization
        val spindexerBalls = ballSim.getSpindexerBallPositions(
            model.drivetrainState.pos,
            model.jointPositions["spindexer_joint"] ?: 0.0
        )
        val drakeBalls = model.ballPositions.mapIndexed { i, pos ->
            val color = model.ballColors.getOrElse(i) { Balls.Green }
            BallState(pos.x, pos.y, pos.z, color.toVizColor())
        }
        val spindexerVizBalls = spindexerBalls.map { (pos, color) ->
            BallState(pos.x, pos.y, pos.z, color.toVizColor())
        }
        val allBalls = drakeBalls + spindexerVizBalls

        val vizState = SimState(
            t = t.toDouble(DurationUnit.SECONDS),
            base = BaseState(
                x = state.pos.v.x,
                y = state.pos.v.y,
                z = model.baseZ,
                roll = 0.0,
                pitch = 0.0,
                yaw = state.pos.rot
            ),
            joints = model.jointPositions,
            telemetry = TelemetryState(
                fl = driveFL,
                fr = driveFR,
                bl = driveBL,
                br = driveBR,
                flywheel = fw.omega,
                turret = turret
            ),
            wheelForces = model.wheelForces.map { ForceState(it.x, it.y, it.z) },
            error = trackingError,
            balls = allBalls,
            mpcTarget = trackingTarget
        )
        broadcastState(vizState)

        // Real-time sync: sleep only for remaining time
        val elapsedNanos = System.nanoTime() - startTime
        val elapsedMillis = elapsedNanos / 1_000_000
        val sleepTime = SIM_UPDATE_TIME.inWholeMilliseconds - elapsedMillis

        if (sleepTime > 0) {
            try {
                Thread.sleep(sleepTime)
            } catch (e: InterruptedException) {
                e.printStackTrace()
            }
        }
        // If sleepTime <= 0, simulation is running slower than real-time (no sleep needed)
    }

    fun setGamepads(gp1: Gamepad, gp2: Gamepad) {
        this.gamepad1 = gp1
        this.gamepad2 = gp2
    }

    private fun getGamepad1State(): GamepadState {
        return try {
            val method = server.javaClass.getMethod("getGamepad1")
            method.invoke(server) as GamepadState
        } catch (e: Exception) {
            GamepadState()
        }
    }

    private fun getGamepad2State(): GamepadState {
        return try {
            val method = server.javaClass.getMethod("getGamepad2")
            method.invoke(server) as GamepadState
        } catch (e: Exception) {
            GamepadState()
        }
    }

    private fun broadcastState(vizState: SimState) {
        try {
            val method = server.javaClass.getMethod("broadcast", SimState::class.java)
            method.invoke(server, vizState)
        } catch (e: Exception) {
            // Silently fail on stub server
        }
    }

    private fun updateGamepadsFromServer() {
        gamepad1?.let { gp ->
            val state = getGamepad1State()
            gp.left_stick_x = state.left_stick_x.toFloat()
            gp.left_stick_y = state.left_stick_y.toFloat()
            gp.right_stick_x = state.right_stick_x.toFloat()
            gp.right_stick_y = state.right_stick_y.toFloat()
            gp.left_trigger = state.left_trigger.toFloat()
            gp.right_trigger = state.right_trigger.toFloat()
            gp.a = state.a
            gp.b = state.b
            gp.x = state.x
            gp.y = state.y
            gp.back = state.back
            gp.start = state.start
            gp.left_bumper = state.left_bumper
            gp.right_bumper = state.right_bumper
            gp.left_stick_button = state.left_stick_button
            gp.right_stick_button = state.right_stick_button
            gp.dpad_up = state.dpad_up
            gp.dpad_down = state.dpad_down
            gp.dpad_left = state.dpad_left
            gp.dpad_right = state.dpad_right
        }

        gamepad2?.let { gp ->
            val state = getGamepad2State()
            gp.left_stick_x = state.left_stick_x.toFloat()
            gp.left_stick_y = state.left_stick_y.toFloat()
            gp.right_stick_x = state.right_stick_x.toFloat()
            gp.right_stick_y = state.right_stick_y.toFloat()
            gp.left_trigger = state.left_trigger.toFloat()
            gp.right_trigger = state.right_trigger.toFloat()
            gp.a = state.a
            gp.b = state.b
            gp.x = state.x
            gp.y = state.y
            gp.back = state.back
            gp.start = state.start
            gp.left_bumper = state.left_bumper
            gp.right_bumper = state.right_bumper
            gp.left_stick_button = state.left_stick_button
            gp.right_stick_button = state.right_stick_button
            gp.dpad_up = state.dpad_up
            gp.dpad_down = state.dpad_down
            gp.dpad_left = state.dpad_left
            gp.dpad_right = state.dpad_right
        }
    }

    override fun setPosition(p: Pose2d) {
        model.setPosition(p)
    }

    override fun time(): Duration = t

    override fun configurePinpoint() {}
    override fun voltage(): Double {
        return 12.0
    }

    override fun colorSensorDetectsBall(): Boolean =
        ballSim.colorSensorDetectsBall()

    override fun colorSensorGetBallColor(): Balls? =
        ballSim.colorSensorGetBallColor()

    fun close() {
        model.destroy()
        try {
            val method = server.javaClass.getMethod("stop")
            method.invoke(server)
        } catch (e: Exception) {
            // Silently fail on stub server
        }
    }

    fun setTrackingError(error: ErrorState?) {
        trackingError = error
    }

    fun setChoreoPath(path: List<PathPoint>) {
        try {
            val method = server.javaClass.getMethod("setChoreoPath", List::class.java)
            method.invoke(server, path)
        } catch (e: Exception) {
            // Silently fail on stub server
        }
    }

    fun setTrackingTarget(path: List<PathPoint>) {
        trackingTarget = path
    }

    // Ball spawning API with color
    fun spawnFieldBall(x: Double, y: Double, z: Double, color: Balls = Balls.Green) {
        model.spawnBall(x, y, z, color)
    }

    fun spawnFieldBalls(positions: List<Pair<Vector2d, Balls>>) {
        positions.forEach { (p, color) ->
            model.spawnBall(p.x, p.y, BALL_RADIUS, color)
        }
    }

    companion object {
        private const val BALL_RADIUS = 0.05
    }
}
