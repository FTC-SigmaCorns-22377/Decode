package sigmacorns.opmode.tune

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.control.PIDController
import sigmacorns.control.SlewRateLimiter
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@Configurable
object SpindexerPIDConfig {
    // Single PID coefficients
    @JvmField var kP = 0.7
    @JvmField var kD = 0.0
    @JvmField var kI = 0.0

    // Motion Profile Limits
    @JvmField var maxVelocity = PI*2.0
    @JvmField var maxAcceleration = 10.0
}

@TeleOp(name = "Spindexer PID Tuner", group = "Test")
class SpindexerPIDTuner : SigmaOpMode() {

    private val ticksPerRev = (((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0
    private val ticksPerRadian = ticksPerRev / (2 * PI)

    private val discreteStep = (2 * PI) / 3  // 120 degrees (spindexer slot)
    private val continuousRate = 2.0

    private val profile = sigmacorns.control.TrapezoidalProfile(
        SpindexerPIDConfig.maxVelocity,
        SpindexerPIDConfig.maxAcceleration
    )

    override fun runOpMode() {
        var targetValue = 0.0

        val pid = PIDController(
            kp = SpindexerPIDConfig.kP,
            kd = SpindexerPIDConfig.kD,
            ki = SpindexerPIDConfig.kI,
            setpoint = targetValue
        )

        telemetry.addLine("Spindexer PID Tuner")
        telemetry.addLine("Use Panels dashboard to adjust kP, kD, kI, maxVel, maxAccel")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Update PID gains and profile limits from configurables
            pid.kp = SpindexerPIDConfig.kP
            pid.kd = SpindexerPIDConfig.kD
            pid.ki = SpindexerPIDConfig.kI
            
            profile.maxVelocity = SpindexerPIDConfig.maxVelocity
            profile.maxAcceleration = SpindexerPIDConfig.maxAcceleration

            // Continuous adjustment with right stick Y
            targetValue += -gamepad1.right_stick_y * continuousRate * dt.inWholeMilliseconds / 1000.0

            // Discrete steps with dpad
            if (gamepad1.dpad_up) {
                targetValue += discreteStep
                while (gamepad1.dpad_up && opModeIsActive()) { idle() }
            }
            if (gamepad1.dpad_down) {
                targetValue -= discreteStep
                while (gamepad1.dpad_down && opModeIsActive()) { idle() }
            }

            // Reset with A button
            if (gamepad1.a) {
                targetValue = 0.0
                pid.reset()
                // Reset profile to current position to avoid jump
                val currentTicks = io.spindexerPosition()
                val currentRad = currentTicks / ticksPerRadian
                profile.reset(currentRad, 0.0)
                
                while (gamepad1.a && opModeIsActive()) { idle() }
            }

            // Get current value
            val currentPositionTicks = io.spindexerPosition()
            val currentValue = currentPositionTicks / ticksPerRadian

            // Calculate profile
            val profileState = profile.calculate(targetValue, dt.toDouble(kotlin.time.DurationUnit.SECONDS))

            // Update PID and compute output
            pid.setpoint = profileState.position
            val motorPower = pid.update(currentValue, dt).coerceIn(-1.0, 1.0)

            // Apply motor power
            io.spindexer = motorPower

            // Calculate error
            val error = targetValue - currentValue

            val telemetry = PanelsTelemetry.telemetry

            // Telemetry
            telemetry.addLine("=== Spindexer PID Tuner ===")
            telemetry.addLine("")
            telemetry.addData("kP", SpindexerPIDConfig.kP)
            telemetry.addData("kD", SpindexerPIDConfig.kD)
            telemetry.addData("kI", SpindexerPIDConfig.kI)
            telemetry.addLine("")
            telemetry.addData("Target (rad)", targetValue)
            telemetry.addData("Target (deg)", Math.toDegrees(targetValue))
            telemetry.addData("Profile Pos", profileState.position)
            telemetry.addData("Profile Vel", profileState.velocity)
            telemetry.addData("Current (rad)", currentValue)
            telemetry.addData("Current (deg)", Math.toDegrees(currentValue))
            telemetry.addLine("")
            telemetry.addData("Error (rad)", error)
            telemetry.addData("Error (deg)", Math.toDegrees(error))
            telemetry.addData("Motor Power", motorPower)
            telemetry.addLine("")
            telemetry.addLine("Controls:")
            telemetry.addLine("  Right Stick Y: Adjust target")
            telemetry.addLine("  D-Pad Up/Down: Step +/- 120°")
            telemetry.addLine("  A: Reset to 0")
            telemetry.addLine("${io.spindexer}")
            telemetry.update()
            telemetry.update()

            false // continue loop
        }
    }
}
