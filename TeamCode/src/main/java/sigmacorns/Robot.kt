package sigmacorns

import kotlinx.coroutines.CoroutineScope
import sigmacorns.constants.Limelight
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.flywheelMotor
import sigmacorns.constants.flywheelParameters
import sigmacorns.control.PollableDispatcher
import sigmacorns.control.mpc.ContourSelectionMode
import sigmacorns.control.mpc.MPCClient
import sigmacorns.control.mpc.MPCRunner
import sigmacorns.subsystem.AimingSystem
import sigmacorns.subsystem.DriveController
import sigmacorns.subsystem.Flywheel
import sigmacorns.control.mpc.TrajoptTrajectory
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumState
import java.lang.AutoCloseable
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

class Robot(val io: SigmaIO, blue: Boolean, shotDataPath: String? = null): AutoCloseable {
    val aim = AimingSystem(io, blue, shotDataPath)
    val flywheel = Flywheel(flywheelMotor, flywheelParameters.inertia, io)
    val drive = DriveController()

    val dispatcher = PollableDispatcher(io)
    val scope = CoroutineScope(dispatcher)

    private val limelight = (io as? HardwareIO)?.limelight
    var mpc: MPCClient? = null
    var runner: MPCRunner? = null

    var aimTurret = true
    var aimFlywheel = true

    fun init(pos: Pose2d, apriltagTracking: Boolean) {
        limelight?.stop()
        io.configurePinpoint()
        io.setPosition(pos)
        aim.init(io.position(),apriltagTracking)
        limelight?.start()
        stopMPCSolver()
    }

    /** Switch limelight to the pipeline that starts the MPC solver process. */
    fun startMPCSolver() {
        limelight?.pipelineSwitch(Limelight.START_MPC_PIPELINE)
    }

    fun pipeline(): Int? = limelight?.status?.pipelineIndex

    /** Switch limelight to the pipeline that stops the MPC solver process. */
    fun stopMPCSolver() {
        limelight?.pipelineSwitch(Limelight.STOP_MPC_PIPELINE)
    }

    /** Switch limelight to the idle pipeline (frees CPU for MPC). */
    fun idleLimelight() {
        limelight?.pipelineSwitch(Limelight.IDLE_PIPELINE)
    }

    /** Create the MPC client without starting the runner thread. */
    fun initMPC() {
        if (mpc != null) mpc?.close()
        mpc = MPCClient(
            drivetrainParameters,
            Network.LIMELIGHT,
            contourSelectionMode = ContourSelectionMode.POSITION,
            preIntegrate = 30.milliseconds,
            sampleLookahead = 0
        )
    }

    /** Start the MPC runner thread (creates client if needed). */
    fun startMPCWorker() {
        if (mpc == null) initMPC()
        runner = MPCRunner(mpc!!, drive)
        runner!!.start()
    }

    /** Start MPC solver, create client, and start runner. */
    fun startMPC() {
        startMPCSolver()
        initMPC()
        startMPCWorker()
    }

    var prewarm: Boolean = false
    var startTime = 0.seconds
    var zero: Boolean = false

    /**
     * Send a single MPC request to pre-warm the solver with the first horizon.
     * Call after [initMPC] and setting a target trajectory.
     */
    fun prewarmMPC(traj: TrajoptTrajectory) {
        prewarm = true
        val m = mpc ?: return
        m.setTarget(traj)
        val state = MecanumState(Pose2d(), io.position())
        m.update(state, 12.0, 0.seconds)
    }

    fun stopMPC() {
        runner?.stop()
        runner = null
        mpc?.close()
        mpc = null
    }

    fun startApriltag() {
        limelight?.pipelineSwitch(Limelight.APRILTAG_PIPELINE)
    }

    private var lastTime = io.time()
    fun update() {
        val t = io.time()
        val dt = t - lastTime
        lastTime = t

        dispatcher.update()

        if(prewarm) {
            startTime = t
        }

        if(!prewarm) {
            runner?.updateState(
                MecanumState(
                    io.velocity(),
                    io.position()
                ), 12.0, t-startTime)
            runner?.driveWithMPC(io, io.voltage())
        }

        if(zero) {
            aimTurret = false
            aim.turret.fieldRelativeMode = false
            aim.turret.targetAngle = 0.0
        }

        aim.update(dt, aimTurret)

        if (aimFlywheel) {
            val recommended = aim.getRecommendedFlywheelVelocity()
            flywheel.target = recommended ?: 0.0
        }
        if (dt > Duration.ZERO) {
            flywheel.update(io.flywheelVelocity(), dt)
        }
    }

    override fun close() {
        aim.close()
        stopMPC()
        limelight?.stop()
    }
}