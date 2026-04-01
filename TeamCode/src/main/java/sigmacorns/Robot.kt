package sigmacorns

import kotlinx.coroutines.CoroutineScope
import sigmacorns.constants.Limelight
import sigmacorns.constants.flywheelMotor
import sigmacorns.constants.flywheelParameters
import sigmacorns.control.PollableDispatcher
import sigmacorns.subsystem.AimingSystem
import sigmacorns.subsystem.DriveController
import sigmacorns.subsystem.Flywheel
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.BeamBreak
import sigmacorns.subsystem.Hood
import sigmacorns.subsystem.Intake
import sigmacorns.subsystem.Transfer
import sigmacorns.subsystem.Turret
import java.lang.AutoCloseable
import kotlin.time.Duration

class Robot(val io: SigmaIO, blue: Boolean, shotDataPath: String? = null): AutoCloseable {
    val aim = AimingSystem(this, blue, shotDataPath)
    val flywheel = Flywheel(flywheelMotor, flywheelParameters.inertia, io)
    val drive = DriveController()
    val beamBreak = BeamBreak(this)
    val intake = Intake(this)
    val transfer = Transfer(this)
    val turret = Turret(this)
    val hood = Hood(this)

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

        // Update subsystems
        beamBreak.update()
        intake.update(dt)
        transfer.update(dt)

        // Update flywheel controller
        if (aimFlywheel) {
            flywheel.update(io.flywheelVelocity(), dt)
        }

        // Update hood (continuously adjusts launch angle)
        hood.update(dt)

        // Update aiming system (vision + turret)
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
        limelight?.stop()
    }
}
