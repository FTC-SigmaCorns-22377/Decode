package sigmacorns

import kotlinx.coroutines.CoroutineScope
import sigmacorns.constants.Limelight
import sigmacorns.constants.antiWheelieConfig
import sigmacorns.constants.antiWheelieFilter
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.AntiWheelieConfig
import sigmacorns.control.AntiWheelieFilter
import sigmacorns.control.PollableDispatcher
import sigmacorns.control.aim.AutoAim
import sigmacorns.control.ltv.LTVClient
import sigmacorns.logic.AimingSystem
import sigmacorns.logic.IntakeCoordinator
import sigmacorns.logic.NativeAutoAim
import sigmacorns.subsystem.Drivetrain
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumDynamics
import sigmacorns.subsystem.BeamBreak
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.Shooter
import sigmacorns.subsystem.Turret
import java.lang.AutoCloseable
import kotlin.math.max
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

class Robot(val io: SigmaIO, val blue: Boolean, useNativeAim: Boolean = false): AutoCloseable {
    // Subsystems
    val shooter = Shooter(this)
    val drive = Drivetrain()
    val beamBreak = BeamBreak(io)
    val intakeTransfer = IntakeTransfer(io)
    val turret = Turret(io)

    // Logic
    val aim: AutoAim = if (useNativeAim) NativeAutoAim(this, blue) else AimingSystem(this, blue)
    val intakeCoordinator = IntakeCoordinator(this)

    var ltv = LTVClient(
        drivetrainParameters,
        aTipX = max(antiWheelieFilter.axLimitBwd, antiWheelieFilter.axLimitFwd)*0.15,
        aTipY = max(antiWheelieFilter.ayLimitLeft, antiWheelieFilter.ayLimitRight)*2,
        aTipTau = 0.1
    )

    val dispatcher = PollableDispatcher(io)
    val scope = CoroutineScope(dispatcher)

    private val limelight = (io as? HardwareIO)?.limelight
    var aimTurret = true
    var aimFlywheel = true

    fun init(pos: Pose2d, apriltagTracking: Boolean) {
        limelight?.stop()
        io.configurePinpoint()
        io.setPosition(pos)
        aim.init(io.position(),apriltagTracking)
        limelight?.start()
    }

    fun pipeline(): Int? = limelight?.status?.pipelineIndex

    var zero: Boolean = false

    fun startApriltag() {
        limelight?.pipelineSwitch(Limelight.APRILTAG_PIPELINE)
    }

    private var lastTime = io.time()
    fun update() {
        val t = io.time()
        val dt = t - lastTime
        lastTime = t

        dispatcher.update()

        if (zero) {
            aimTurret = false
            turret.fieldRelativeMode = false
            turret.targetAngle = 0.0
        }

        // 1. Read sensors
        beamBreak.update()

        // 2. Logic coordination (sets subsystem inputs)
        // Aim runs first: updates vision/fusion so coordinators see fresh data
        aim.update(dt, aimTurret)
        intakeCoordinator.update()

        // 3. Subsystem updates (write IO)
        intakeTransfer.update(dt, t)
        shooter.update(dt)
        turret.update(dt)
    }

    override fun close() {
        aim.close()
        limelight?.stop()
        ltv.close()
    }
}
