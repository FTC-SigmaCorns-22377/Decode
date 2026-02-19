package sigmacorns.control

import com.bylazar.gamepad.Gamepad
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
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumState
import java.lang.AutoCloseable
import kotlin.math.PI
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
        if(aimFlywheel) aim.getRecommendedFlywheelVelocity()?.let { logic.shotVelocity = it }

        logic.update( dt)
    }

    override fun close() {
        aim.close()
        stopMPC()
    }
}