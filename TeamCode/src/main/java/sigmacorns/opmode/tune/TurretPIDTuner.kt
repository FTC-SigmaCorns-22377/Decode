package sigmacorns.opmode.tune

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.TurretServoConfig
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@Configurable
object TurretPIDConfig {
    @JvmField var kP = 1.5
    @JvmField var kD = 0.08
    @JvmField var kI = 0.0
    @JvmField var kVRobot = 0.0
    @JvmField var maxTargetLead = 1.0
    @JvmField var slewRate = 60000.0
    @JvmField var outputSlewRate = 4000.0
}

/**
 * Turret servo tuner. Tests the dual-servo turret system.
 * Adjusts slew rate and angle limits via Panels dashboard.
 */
@TeleOp(name = "Turret Servo Tuner", group = "Tune")
class TurretPIDTuner : SigmaOpMode() {

    private var targetAngle = 0.0
    private val continuousRate = 1.0 // rad/s

    override fun runOpMode() {
        val robot = Robot(io, blue = false)
        robot.init(Pose2d(), apriltagTracking = false)
        robot.aimTurret = false

        telemetry.addLine("Turret Servo Tuner")
        telemetry.addLine("Right Stick X: Move turret")
        telemetry.addLine("A: Center turret")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Continuous adjustment
            targetAngle += -gamepad1.right_stick_x * continuousRate * dt.inWholeMilliseconds / 1000.0
            targetAngle = targetAngle.coerceIn(TurretServoConfig.minAngle, TurretServoConfig.maxAngle)

            // Reset
            if (gamepad1.a) {
                targetAngle = 0.0
                while (gamepad1.a && opModeIsActive()) { idle() }
            }

            robot.turret.fieldRelativeMode = false
            robot.turret.targetAngle = targetAngle
            robot.turret.update(dt)

            io.update()

            val tel = PanelsTelemetry.telemetry
            tel.addLine("=== Turret Servo Tuner ===")
            tel.addData("Target (deg)", Math.toDegrees(targetAngle))
            tel.addData("Position (deg)", Math.toDegrees(robot.turret.pos))
            tel.addData("Servo Position", robot.turret.currentServoPosition)
            tel.addData("Servo L", io.turretLeft)
            tel.addData("Servo R", io.turretRight)
            tel.addData("Aligned", robot.turret.aligned)
            tel.update()

            false
        }

        robot.close()
    }
}
