package sigmacorns.opmode.tune

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.constants.bareMotorTopSpeed
import sigmacorns.control.MotorRangeMapper
import sigmacorns.control.subsystem.Turret
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@Configurable
object TurretPIDConfig {
    @JvmField var kP = 1.5
    @JvmField var kD = 0.08
    @JvmField var kI = 0.0
    @JvmField var kVRobot = 1.0 / (bareMotorTopSpeed * (1150.0/6000.0) * 19.0 / 76.0) * 1.2
    @JvmField var maxTargetLead = 0.25
    @JvmField var slewRate = 60000.0
    @JvmField var outputSlewRate = 4000.0
}

@TeleOp(name = "Turret PID Tuner", group = "Tune")
class TurretPIDTuner : SigmaOpMode() {

    private val ticksPerRad = (1.0 + (46.0 / 11.0)) * 28.0 / (2*PI) * 76 / 19

    private val turretRange = MotorRangeMapper(
        limits = -PI/2.0..PI/2.0,           // turret can rotate +/- 190 degrees
        limitsTick = -PI/2.0*ticksPerRad..PI/2.0*ticksPerRad,           // turret can rotate +/- 190 degrees
        slowdownDist = 0.3           // slow down within 0.3 rad of limits
    )

    private lateinit var turret: Turret

    // Tuning state
    private var targetAngle = 0.0
    private val discreteStep = PI / 4 // 45 degrees
    private val continuousRate = 1.0 // rad/s

    override fun runOpMode() {
        turret = Turret(turretRange, io)
        turret.fieldRelativeMode = false

        telemetry.addLine("Turret PID Tuner (Direct)")
        telemetry.addLine("Use Panels dashboard to adjust kP, kD, kI, etc.")
        telemetry.addLine("Use gamepad to control target")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Update inputs
            
            // Continuous adjustment
            targetAngle += -gamepad1.right_stick_y * continuousRate * dt.inWholeMilliseconds / 1000.0
            
            // Discrete steps
            if (gamepad1.dpad_up) {
                targetAngle += discreteStep
                while (gamepad1.dpad_up && opModeIsActive()) { idle() }
            }
            if (gamepad1.dpad_down) {
                targetAngle -= discreteStep
                while (gamepad1.dpad_down && opModeIsActive()) { idle() }
            }
            
            // Reset
            if (gamepad1.a) {
                targetAngle = 0.0
                while (gamepad1.a && opModeIsActive()) { idle() }
            }
            
            // Update Turret
            // Set robot velocity to 0 or actual if available (for static tuning it's 0 usually)
            // But if we want to tune kVRobot we might want to simulate it or drive around.
            // Let's read from io if available.
            turret.robotHeading = io.position().rot
            turret.robotAngularVelocity = io.velocity().rot
            
            // For simple tuning, we might want to test field relative mode if the user requested it.
            // But let's stick to robot relative for consistent tuning unless toggled.
            // Actually, if PID logic changes to rely on field relative logic, we verify it works.
            turret.targetAngle = targetAngle
            turret.update(dt)

            // Telemetry
            val tel = PanelsTelemetry.telemetry
            tel.addLine("=== Turret PID Tuner ===")
            tel.addData("Target (rad)", targetAngle)
            tel.addData("Target (deg)", Math.toDegrees(targetAngle))
            tel.addData("Current (rad)", turret.pos)
            tel.addData("Current (deg)", Math.toDegrees(turret.pos))
            tel.addData("Error (rad)", turret.effectiveTargetAngle - turret.pos)
            tel.addData("Motor Power", io.turret) // Turret class sets io.turret
            
            tel.addLine("")
            tel.addLine("Config:")
            tel.addData("kP", TurretPIDConfig.kP)
            
            tel.update()

            false // continue loop
        }
    }
}
