package sigmacorns.io

import sigmacorns.math.Pose2d
import sigmacorns.sim.DrakeRobotModel
import sigmacorns.sim.viz.SimServer
import sigmacorns.sim.viz.SimState
import sigmacorns.sim.viz.BaseState
import sigmacorns.sim.viz.TelemetryState
import sigmacorns.sim.viz.BallState
import sigmacorns.sim.viz.ErrorState
import sigmacorns.sim.viz.ForceState
import sigmacorns.sim.viz.PathPoint
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

class DrakeSimIO(urdfPath: String) : SigmaIO {
    val model = DrakeRobotModel(urdfPath)
    val server = SimServer(8080)
    private var trackingError: ErrorState? = null
    private var trackingTarget: List<PathPoint> = emptyList()
    
    private var t = 0.seconds
    private val SIM_UPDATE_TIME = 10.milliseconds

    init {
        server.start()
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

    override fun position(): Pose2d = model.drivetrainState.pos
    override fun velocity(): Pose2d = model.drivetrainState.vel
    override fun flywheelVelocity(): Double = model.flywheelState.omega

    override fun update() {
        // Step Simulation
        model.advanceSim(SIM_UPDATE_TIME.toDouble(DurationUnit.SECONDS), this)
        t += SIM_UPDATE_TIME

        // Broadcast State
        val state = model.drivetrainState
        val fw = model.flywheelState
        
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
            balls = model.ballPositions.map { BallState(it.x, it.y, it.z) },
            mpcTarget = trackingTarget
        )
        server.broadcast(vizState)
        
        // Sleep to match real-time? 
        // Or just let it run fast. Usually for viz we want roughly real time.
        try {
             Thread.sleep(SIM_UPDATE_TIME.inWholeMilliseconds)
        } catch (e: InterruptedException) {
             e.printStackTrace()
        }
    }

    override fun setPosition(p: Pose2d) {
        model.setPosition(p)
    }

    override fun time(): Duration = t

    override fun configurePinpoint() {}
    
    fun close() {
        model.destroy()
        server.stop()
    }

    fun setTrackingError(error: ErrorState?) {
        trackingError = error
    }

    fun setChoreoPath(path: List<PathPoint>) {
        server.setChoreoPath(path)
    }

    fun setTrackingTarget(path: List<PathPoint>) {
        trackingTarget = path
    }
}
