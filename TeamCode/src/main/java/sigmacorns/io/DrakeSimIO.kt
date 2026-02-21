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
import sigmacorns.sim.viz.AimingVizState
import sigmacorns.sim.viz.BallState
import sigmacorns.sim.viz.ContourVizState
import sigmacorns.sim.viz.ErrorState
import sigmacorns.sim.viz.ForceState
import sigmacorns.sim.viz.GTSAMVizState
import sigmacorns.sim.viz.MPCHorizonState
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

    // Control visualization state (thread-safe via volatile)
    @Volatile private var mpcPredicted: List<PathPoint> = emptyList()
    @Volatile private var mpcContours: List<ContourVizState> = emptyList()
    @Volatile private var mpcHorizon: MPCHorizonState? = null
    @Volatile private var gtsamViz: GTSAMVizState? = null
    @Volatile private var aimingViz: AimingVizState? = null

    private var gamepad1: Gamepad? = null
    private var gamepad2: Gamepad? = null

    private var t = 0.seconds
    private val SIM_UPDATE_TIME = 20.milliseconds

    // Threading support
    private val simLock = Any()
    private var running = true
    private val drakeThread: Thread
    private val vizThread: Thread

    // Step synchronization: update() requests a step, Drake thread executes it
    private val stepLock = Object()
    @Volatile private var stepRequested = false
    @Volatile private var stepComplete = false
    @Volatile
    private var currentDriveFL = 0.0
    @Volatile
    private var currentDriveBL = 0.0
    @Volatile
    private var currentDriveFR = 0.0
    @Volatile
    private var currentDriveBR = 0.0
    @Volatile
    private var currentShooter = 0.0
    @Volatile
    private var currentIntake = 0.0
    @Volatile
    private var currentTurret = 0.0
    @Volatile
    private var currentSpindexer = 0.0
    @Volatile
    private var currentTransfer = 0.0

    // Cached reflection methods (resolved once at init, not per-call)
    private var cachedBroadcastMethod: java.lang.reflect.Method? = null
    private var cachedGetGamepad1Method: java.lang.reflect.Method? = null
    private var cachedGetGamepad2Method: java.lang.reflect.Method? = null
    private var cachedSetChoreoPathMethod: java.lang.reflect.Method? = null
    private var cachedStopMethod: java.lang.reflect.Method? = null

    // Reusable SigmaIO snapshot to avoid per-step allocation
    private val snapshotIO = object : SigmaIO {
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

        override fun position(): Pose2d = Pose2d(0.0, 0.0, 0.0)
        override fun velocity(): Pose2d = Pose2d(0.0, 0.0, 0.0)
        override fun flywheelVelocity(): Double = 0.0
        override fun turretPosition(): Double = 0.0
        override fun spindexerPosition(): Double = 0.0
        override fun distance(): Double = Double.MAX_VALUE
        override fun update() {}
        override fun setPosition(p: Pose2d) {}
        override fun time(): Duration = 0.seconds
        override fun configurePinpoint() {}
        override fun voltage(): Double = 12.0
        override fun colorSensorDetectsBall(): Boolean = false
        override fun colorSensorGetBallColor(): Balls? = null
    }

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

        // Cache reflection methods once at init (avoids per-call Method lookup)
        try {
            cachedBroadcastMethod = server.javaClass.getMethod("broadcast", SimState::class.java)
            cachedGetGamepad1Method = server.javaClass.getMethod("getGamepad1")
            cachedGetGamepad2Method = server.javaClass.getMethod("getGamepad2")
            cachedSetChoreoPathMethod = server.javaClass.getMethod("setChoreoPath", List::class.java)
            cachedStopMethod = server.javaClass.getMethod("stop")
        } catch (e: Exception) {
            // Stub server - methods not available, leave null
        }

        // Spawn initial field balls at artifact pickup locations
        // Based on DECODE field positions (converted to meters from Pedro inches)
        spawnInitialBalls()

        // Start Drake simulation thread (step-on-demand)
        drakeThread = Thread {
            println("DrakeSimIO: Drake simulation thread started")
            while (running) {
                try {
                    // Wait for update() to request a step
                    synchronized(stepLock) {
                        while (!stepRequested && running) {
                            (stepLock as Object).wait()
                        }
                        if (!running) break
                        stepRequested = false
                    }

                    // Update gamepads from server
                    updateGamepadsFromServer()

                    // Create snapshot of motor commands (thread-safe read of volatile fields)
                    val snapshotFL = currentDriveFL
                    val snapshotBL = currentDriveBL
                    val snapshotFR = currentDriveFR
                    val snapshotBR = currentDriveBR
                    val snapshotShooter = currentShooter
                    val snapshotIntake = currentIntake
                    val snapshotTurret = currentTurret
                    val snapshotSpindexer = currentSpindexer
                    val snapshotTransfer = currentTransfer

                    // Update reusable snapshot IO (avoids per-step allocation)
                    snapshotIO.driveFL = snapshotFL
                    snapshotIO.driveBL = snapshotBL
                    snapshotIO.driveFR = snapshotFR
                    snapshotIO.driveBR = snapshotBR
                    snapshotIO.shooter = snapshotShooter
                    snapshotIO.intake = snapshotIntake
                    snapshotIO.turret = snapshotTurret
                    snapshotIO.spindexer = snapshotSpindexer
                    snapshotIO.transfer = snapshotTransfer

                    try {
                        synchronized(simLock) {
                            // Validate motor commands before simulation step
                            if (!snapshotFL.isFinite() || !snapshotBL.isFinite() ||
                                !snapshotFR.isFinite() || !snapshotBR.isFinite()) {
                                println("WARNING: Invalid motor commands detected, skipping step")
                                println("  FL=$snapshotFL BL=$snapshotBL FR=$snapshotFR BR=$snapshotBR")
                            } else {
                                model.advanceSim(SIM_UPDATE_TIME.toDouble(DurationUnit.SECONDS), snapshotIO)
                                t += SIM_UPDATE_TIME

                                // Update ball interaction simulation
                                ballSim.update(
                                    robotPose = model.drivetrainState.pos,
                                    robotVelocity = model.drivetrainState.vel,
                                    intakePower = snapshotIntake,
                                    transferPower = snapshotTransfer,
                                    spindexerAngle = model.jointPositions["spindexer_joint"] ?: 0.0,
                                    turretYaw = model.jointPositions["turret_joint"] ?: 0.0,
                                    hoodPitch = model.jointPositions["hood_joint"] ?: 0.0,
                                    flywheelOmega = model.flywheelState.omega,
                                    dt = SIM_UPDATE_TIME
                                )
                            }
                        }
                    } catch (e: Exception) {
                        System.err.println("ERROR in Drake simulation step: ${e.message}")
                        e.printStackTrace()
                    }

                    // Signal step completion
                    synchronized(stepLock) {
                        stepComplete = true
                        (stepLock as Object).notifyAll()
                    }
                } catch (e: InterruptedException) {
                    break
                } catch (e: Exception) {
                    e.printStackTrace()
                }
            }
            println("DrakeSimIO: Drake simulation thread stopped")
        }
        drakeThread.name = "Drake-Sim-Thread"
        drakeThread.start()

        // Start visualization thread
        vizThread = Thread {
            println("DrakeSimIO: Visualization thread started")
            while (running) {
                try {
                    val startTime = System.nanoTime()

                    // Broadcast state
                    val vizState = synchronized(simLock) {
                        val state = model.drivetrainState
                        val fw = model.flywheelState

                        // Combine Drake ball positions with spindexer ball positions
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

                        SimState(
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
                                fl = currentDriveFL,
                                fr = currentDriveFR,
                                bl = currentDriveBL,
                                br = currentDriveBR,
                                flywheel = fw.omega,
                                turret = currentTurret,
                                spindexerPower = currentSpindexer,
                                spindexerAngle = model.jointPositions["spindexer_joint"] ?: 0.0,
                                intakePower = currentIntake
                            ),
                            wheelForces = model.wheelForces.map { ForceState(it.x, it.y, it.z) },
                            error = trackingError,
                            balls = allBalls,
                            mpcTarget = trackingTarget,
                            mpcPredicted = mpcPredicted,
                            mpcContours = mpcContours,
                            mpcHorizon = mpcHorizon,
                            gtsam = gtsamViz,
                            aiming = aimingViz
                        )
                    }

                    broadcastState(vizState)

                    // Visualization updates at 20Hz (50ms)
                    val elapsedNanos = System.nanoTime() - startTime
                    val elapsedMillis = elapsedNanos / 1_000_000
                    val sleepTime = 50 - elapsedMillis

                    if (sleepTime > 0) {
                        Thread.sleep(sleepTime)
                    }
                } catch (e: InterruptedException) {
                    break
                } catch (e: Exception) {
                    e.printStackTrace()
                }
            }
            println("DrakeSimIO: Visualization thread stopped")
        }
        vizThread.name = "Drake-Viz-Thread"
        vizThread.start()
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

    override var driveFL: Double
        get() = currentDriveFL
        set(value) { currentDriveFL = value.sanitize() }
    override var driveBL: Double
        get() = currentDriveBL
        set(value) { currentDriveBL = value.sanitize() }
    override var driveFR: Double
        get() = currentDriveFR
        set(value) { currentDriveFR = value.sanitize() }
    override var driveBR: Double
        get() = currentDriveBR
        set(value) { currentDriveBR = value.sanitize() }
    override var shooter: Double
        get() = currentShooter
        set(value) { currentShooter = value.sanitize() }
    override var intake: Double
        get() = currentIntake
        set(value) { currentIntake = value.sanitize() }
    override var turret: Double
        get() = currentTurret
        set(value) { currentTurret = value.sanitize() }
    override var spindexer: Double
        get() = currentSpindexer
        set(value) { currentSpindexer = if (value.isNaN() || value.isInfinite()) 0.0 else value }
    override var turretAngle: Double = 0.0
    override var breakPower: Double = 0.0
    override var transfer: Double
        get() = currentTransfer
        set(value) { currentTransfer = if (value.isNaN() || value.isInfinite()) 0.0 else value.coerceIn(0.0, 1.0) }

    private fun Double.sanitize(): Double {
        return when {
            this.isNaN() || this.isInfinite() -> 0.0
            this > 1.0 -> 1.0
            this < -1.0 -> -1.0
            else -> this
        }
    }

    override fun position(): Pose2d = synchronized(simLock) { model.drivetrainState.pos }
    override fun velocity(): Pose2d = synchronized(simLock) { model.drivetrainState.vel }
    // Returns flywheel angular velocity in rad/s (raw output shaft velocity from Drake)
    override fun flywheelVelocity(): Double = synchronized(simLock) { model.flywheelState.omega }

    override fun turretPosition(): Double = synchronized(simLock) {
        // Drake returns output shaft radians; control expects motor encoder ticks
        (model.jointPositions["turret_joint"] ?: 0.0) * TURRET_TICKS_PER_RAD
    }

    override fun spindexerPosition(): Double = synchronized(simLock) {
        // Drake returns output shaft radians; control expects motor encoder ticks
        (model.jointPositions["spindexer_joint"] ?: 0.0) * SPINDEXER_TICKS_PER_RAD
    }

    override fun distance(): Double = synchronized(simLock) {
        if(ballSim.colorSensorDetectsBall()) 0.0 else Double.MAX_VALUE
    }

    override fun update() {
        // Request a sim step and wait for completion.
        // This synchronizes the control loop with the Drake simulation:
        // both run in separate threads but advance together one timestep at a time.
        synchronized(stepLock) {
            stepComplete = false
            stepRequested = true
            (stepLock as Object).notifyAll()

            while (!stepComplete) {
                try {
                    (stepLock as Object).wait()
                } catch (e: InterruptedException) {
                    Thread.currentThread().interrupt()
                    return
                }
            }
        }
    }

    fun setGamepads(gp1: Gamepad, gp2: Gamepad) {
        this.gamepad1 = gp1
        this.gamepad2 = gp2
    }

    private fun getGamepad1State(): GamepadState {
        return try {
            cachedGetGamepad1Method?.invoke(server) as? GamepadState ?: GamepadState()
        } catch (e: Exception) {
            GamepadState()
        }
    }

    private fun getGamepad2State(): GamepadState {
        return try {
            cachedGetGamepad2Method?.invoke(server) as? GamepadState ?: GamepadState()
        } catch (e: Exception) {
            GamepadState()
        }
    }

    private fun broadcastState(vizState: SimState) {
        try {
            cachedBroadcastMethod?.invoke(server, vizState)
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

    override fun setPosition(p: Pose2d) = synchronized(simLock) {
        model.setPosition(p)
    }

    override fun time(): Duration = synchronized(simLock) { t }

    override fun configurePinpoint() {}
    override fun voltage(): Double = 12.0

    override fun colorSensorDetectsBall(): Boolean = synchronized(simLock) {
        ballSim.colorSensorDetectsBall()
    }

    override fun colorSensorGetBallColor(): Balls? = synchronized(simLock) {
        ballSim.colorSensorGetBallColor()
    }

    fun close() {
        println("DrakeSimIO: Shutting down...")
        running = false

        // Wake up Drake thread if it's waiting for a step request
        synchronized(stepLock) {
            (stepLock as Object).notifyAll()
        }

        // Wait for threads to finish
        try {
            drakeThread.join(1000)
            vizThread.join(1000)
        } catch (e: InterruptedException) {
            e.printStackTrace()
        }

        model.destroy()
        try {
            cachedStopMethod?.invoke(server)
        } catch (e: Exception) {
            // Silently fail on stub server
        }
        println("DrakeSimIO: Shutdown complete")
    }

    fun setTrackingError(error: ErrorState?) {
        trackingError = error
    }

    fun setChoreoPath(path: List<PathPoint>) {
        try {
            cachedSetChoreoPathMethod?.invoke(server, path)
        } catch (e: Exception) {
            // Silently fail on stub server
        }
    }

    fun setTrackingTarget(path: List<PathPoint>) {
        trackingTarget = path
    }

    fun setMPCPredicted(points: List<PathPoint>) {
        mpcPredicted = points
    }

    fun setMPCContours(contours: List<ContourVizState>) {
        mpcContours = contours
    }

    fun setMPCHorizon(horizon: MPCHorizonState?) {
        mpcHorizon = horizon
    }

    fun setGTSAMViz(viz: GTSAMVizState?) {
        gtsamViz = viz
    }

    fun setAimingViz(viz: AimingVizState?) {
        aimingViz = viz
    }

    // Ball spawning API with color
    fun spawnFieldBall(x: Double, y: Double, z: Double, color: Balls = Balls.Green) = synchronized(simLock) {
        model.spawnBall(x, y, z, color)
    }

    fun spawnFieldBalls(positions: List<Pair<Vector2d, Balls>>) = synchronized(simLock) {
        positions.forEach { (p, color) ->
            model.spawnBall(p.x, p.y, BALL_RADIUS, color)
        }
    }

    companion object {
        private const val BALL_RADIUS = 0.0635

        // Motor encoder ticks per radian of output shaft rotation.
        // Matches the gear ratios used in MotorRangeMapper for each mechanism.

        // Spindexer: (1+(46/17)) * (1+(46/11)) * 28 ticks per revolution
        private val SPINDEXER_TICKS_PER_RAD =
            ((1.0 + (46.0 / 17.0)) * (1.0 + (46.0 / 11.0)) * 28.0) / (2 * Math.PI)

        // Turret: (1+(46/11)) * 28 / (2π) * (76/19) ticks per radian
        private val TURRET_TICKS_PER_RAD =
            (1.0 + (46.0 / 11.0)) * 28.0 / (2 * Math.PI) * 76.0 / 19.0
    }
}
