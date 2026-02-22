package sigmacorns.control

import com.bylazar.gamepad.Gamepad
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import sigmacorns.constants.Limelight
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.flywheelMotor
import sigmacorns.constants.flywheelParameters
import sigmacorns.control.mpc.ContourSelectionMode
import sigmacorns.control.mpc.MPCClient
import sigmacorns.control.mpc.MPCRunner
import sigmacorns.control.subsystem.AimingSystem
import sigmacorns.control.subsystem.DriveController
import sigmacorns.control.subsystem.Flywheel
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.globalFieldState
import sigmacorns.control.mpc.TrajoptTrajectory
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.Balls
import sigmacorns.sim.MecanumState
import java.lang.AutoCloseable
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.use

class Robot(val io: SigmaIO, blue: Boolean): AutoCloseable {
    val aim = AimingSystem(io,blue)
    val flywheel = Flywheel(flywheelMotor, flywheelParameters.inertia, io)
    val logic = SpindexerLogic(io, flywheel)
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
    fun startMPCRunner() {
        if (mpc == null) initMPC()
        runner = MPCRunner(mpc!!, drive)
        runner!!.start()
    }

    /** Start MPC solver, create client, and start runner. */
    fun startMPC() {
        startMPCSolver()
        initMPC()
        startMPCRunner()
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
            prewarm = false
            startTime = t
        }

        runner?.updateState(
            MecanumState(
                io.velocity(),
                io.position()
            ), 12.0, t-startTime)
        runner?.driveWithMPC(io, io.voltage())

        if(zero) {
            aimTurret = false
            aim.turret.fieldRelativeMode = false
            aim.turret.targetAngle = 0.0
        }


        aim.update(dt, aimTurret)
        if(aimFlywheel) aim.getRecommendedFlywheelVelocity()?.let {
            logic.shotVelocity = it
            logic.spinupPower = it}


        logic.update( dt)
    }

    /**
     * Non-blocking poll for motif fiducials (IDs 21/22/23).
     * Call repeatedly during init loop. Returns detected ID or null.
     */
    fun pollMotif(): Int? {
        val result = limelight?.latestResult ?: return null

        // if (!result.isValid) return null

        val fiducials: List<FiducialResult> = result.fiducialResults
        for (fiducial in fiducials) {
            val id = fiducial.fiducialId
            if (id == 21 || id == 22 || id == 23) return id
        }
        return null
    }

    /**
     * Stores detected motif in [logic.motif] and [globalFieldState],
     * aims turret at goal. Call once after init polling is done.
     */
    fun applyDetectedMotif(motifId: Int) {
        globalFieldState.motif = motifId
        logic.motif = motifIdToBalls(motifId)
        println("Robot: Detected motif ID=$motifId -> ${logic.motif}")

        // Aim turret at goal
        val goalPos = aim.goalPosition
        val robotPos = io.position()
        val angleToGoal = atan2(
            goalPos.y - robotPos.v.y,
            goalPos.x - robotPos.v.x
        )
        aim.turret.fieldTargetAngle = angleToGoal
        aim.turret.fieldRelativeMode = true
    }

    companion object {
        fun motifIdToBalls(id: Int): List<Balls?> = when (id) {
            21 -> listOf(Balls.Green, Balls.Purple, Balls.Purple)
            22 -> listOf(Balls.Purple, Balls.Green, Balls.Purple)
            23 -> listOf(Balls.Purple, Balls.Purple, Balls.Green)
            else -> listOf(Balls.Green, Balls.Purple, Balls.Purple)
        }
    }

    override fun close() {
        aim.close()
        stopMPC()
        limelight?.stop()
    }
}