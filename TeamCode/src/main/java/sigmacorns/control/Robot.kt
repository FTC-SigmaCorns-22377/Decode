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
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.Balls
import sigmacorns.sim.MecanumState
import java.lang.AutoCloseable
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration.Companion.milliseconds
import kotlin.use

class Robot(val io: SigmaIO, blue: Boolean): AutoCloseable {
    val aim = AimingSystem(io,blue)
    val flywheel = Flywheel(flywheelMotor, flywheelParameters.inertia, io)
    val logic = SpindexerLogic(io, flywheel)
    val drive = DriveController()

    val dispatcher = PollableDispatcher(io)
    val scope = CoroutineScope(dispatcher)

    val limelight = (io as? HardwareIO)?.limelight
    var mpc: MPCClient? = null
    var runner: MPCRunner? = null

    var aimTurret = true
    var aimFlywheel = true

    fun init(pos: Pose2d, apriltagTracking: Boolean) {
        io.configurePinpoint()
        io.setPosition(pos)
        aim.init(io.position(),apriltagTracking)
    }

    fun startMPC() {
        if(mpc != null) mpc?.close()
        limelight?.pipelineSwitch(Limelight.MPC_PIPELINE)

        mpc = MPCClient(
            drivetrainParameters,
            Network.LIMELIGHT,
            contourSelectionMode = ContourSelectionMode.POSITION,
            preIntegrate = 30.milliseconds,
            sampleLookahead = 0
        )
        runner = MPCRunner(mpc!!,drive)
        runner!!.start()
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

        runner?.updateState(
            MecanumState(
                io.velocity(),
                io.position()
            ), 12.0, t)
        runner?.driveWithMPC(io, io.voltage())

        aim.update(dt, aimTurret)
        if(aimFlywheel) aim.getRecommendedFlywheelVelocity()?.let {
            logic.shotVelocity = it
            logic.spinupPower = it}


        logic.update( dt)
    }

    /** Start limelight on AprilTag pipeline for motif detection during init. */
    fun startMotifDetection() {
        limelight?.pipelineSwitch(Limelight.APRILTAG_PIPELINE)
        limelight?.start()
    }

    /**
     * Non-blocking poll for motif fiducials (IDs 21/22/23).
     * Call repeatedly during init loop. Returns detected ID or null.
     */
    fun pollMotif(): Int? {
        val result = limelight!!.latestResult ?: return null


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
    }
}