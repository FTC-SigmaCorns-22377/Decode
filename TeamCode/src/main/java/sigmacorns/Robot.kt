package sigmacorns

import kotlinx.coroutines.CoroutineScope
import sigmacorns.constants.Limelight
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.PollableDispatcher
import sigmacorns.control.ltv.LTVClient
import sigmacorns.logic.AimingSystem
import sigmacorns.logic.IntakeCoordinator
import sigmacorns.subsystem.Drivetrain
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.BeamBreak
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.Shooter
import sigmacorns.subsystem.Turret
import java.lang.AutoCloseable
import kotlin.time.Duration

class Robot(val io: SigmaIO, blue: Boolean): AutoCloseable {
    // Subsystems
    val shooter = Shooter(io)
    val drive = Drivetrain()
    val beamBreak = BeamBreak(io)
    val intakeTransfer = IntakeTransfer(io)
    val turret = Turret(io)

    // Logic
    val aim = AimingSystem(this, blue)
    val intakeCoordinator = IntakeCoordinator(this)

    val ltv = LTVClient(drivetrainParameters)

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
