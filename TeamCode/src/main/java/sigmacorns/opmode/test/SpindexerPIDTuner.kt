package sigmacorns.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.control.PIDController
import sigmacorns.control.SlewRateLimiter
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@Configurable
object SpindexerPIDConfig {
    // PID coefficients for 0 balls
    @JvmField var kP_0 = 0.5
    @JvmField var kD_0 = 0.001
    @JvmField var kI_0 = 0.0

    // PID coefficients for 1 ball
    @JvmField var kP_1 = 0.5
    @JvmField var kD_1 = 0.001
    @JvmField var kI_1 = 0.0

    // PID coefficients for 2 balls
    @JvmField var kP_2 = 0.7
    @JvmField var kD_2 = 0.002
    @JvmField var kI_2 = 0.0

    // PID coefficients for 3 balls
    @JvmField var kP_3 = 0.7
    @JvmField var kD_3 = 0.002
    @JvmField var kI_3 = 0.0

    @JvmField var slewRate = 1.7

    fun getKp(ballCount: Int) = when (ballCount) {
        0 -> kP_0
        1 -> kP_1
        2 -> kP_2
        3 -> kP_3
        else -> kP_0
    }

    fun getKd(ballCount: Int) = when (ballCount) {
        0 -> kD_0
        1 -> kD_1
        2 -> kD_2
        3 -> kD_3
        else -> kD_0
    }

    fun getKi(ballCount: Int) = when (ballCount) {
        0 -> kI_0
        1 -> kI_1
        2 -> kI_2
        3 -> kI_3
        else -> kI_0
    }
}

@TeleOp(name = "Spindexer PID Tuner", group = "Test")
class SpindexerPIDTuner : SigmaOpMode() {

    private val ticksPerRev = (1.0 + (46.0 / 11.0)) * 28.0
    private val ticksPerRadian = ticksPerRev / (2 * PI)

    private val discreteStep = (2 * PI) / 3  // 120 degrees (spindexer slot)
    private val continuousRate = 2.0

    private val slewRateLimiter = SlewRateLimiter(SpindexerPIDConfig.slewRate)

    override fun runOpMode() {
        var targetValue = 0.0
        var selectedBallCount = 0

        val pid = PIDController(
            kp = SpindexerPIDConfig.getKp(selectedBallCount),
            kd = SpindexerPIDConfig.getKd(selectedBallCount),
            ki = SpindexerPIDConfig.getKi(selectedBallCount),
            setpoint = targetValue
        )

        telemetry.addLine("Spindexer PID Tuner")
        telemetry.addLine("Use Panels dashboard to adjust kP, kD, kI")
        telemetry.addLine("Use bumpers to select ball count (0-3)")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Change ball count with bumpers
            if (gamepad1.left_bumper) {
                selectedBallCount = (selectedBallCount - 1).coerceIn(0, 3)
                pid.reset()
                slewRateLimiter.reset()
                while (gamepad1.left_bumper && opModeIsActive()) { idle() }
            }
            if (gamepad1.right_bumper) {
                selectedBallCount = (selectedBallCount + 1).coerceIn(0, 3)
                pid.reset()
                slewRateLimiter.reset()
                while (gamepad1.right_bumper && opModeIsActive()) { idle() }
            }

            // Update PID gains from configurables based on selected ball count
            pid.kp = SpindexerPIDConfig.getKp(selectedBallCount)
            pid.kd = SpindexerPIDConfig.getKd(selectedBallCount)
            pid.ki = SpindexerPIDConfig.getKi(selectedBallCount)
            slewRateLimiter.maxRate = SpindexerPIDConfig.slewRate

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
                slewRateLimiter.reset()
                while (gamepad1.a && opModeIsActive()) { idle() }
            }

            // Get current value
            val currentPositionTicks = io.spindexerPosition()
            val currentValue = currentPositionTicks / ticksPerRadian

            val slewLimitedTarget = slewRateLimiter.calculate(targetValue, dt)

            // Update PID and compute output
            pid.setpoint = slewLimitedTarget
            val motorPower = pid.update(currentValue, dt).coerceIn(-1.0, 1.0)

            // Apply motor power
            io.spindexer = motorPower

            // Calculate error
            val error = targetValue - currentValue

            val telemetry = PanelsTelemetry.telemetry

            // Telemetry
            telemetry.addLine("=== Spindexer PID Tuner ===")
            telemetry.addLine("")
            telemetry.addData("*** Ball Count ***", selectedBallCount)
            telemetry.addLine("")
            telemetry.addData("kP_$selectedBallCount", SpindexerPIDConfig.getKp(selectedBallCount))
            telemetry.addData("kD_$selectedBallCount", SpindexerPIDConfig.getKd(selectedBallCount))
            telemetry.addData("kI_$selectedBallCount", SpindexerPIDConfig.getKi(selectedBallCount))
            telemetry.addLine("")
            telemetry.addData("Target (rad)", targetValue)
            telemetry.addData("Target (deg)", Math.toDegrees(targetValue))
            telemetry.addData("Current (rad)", currentValue)
            telemetry.addData("Current (deg)", Math.toDegrees(currentValue))
            telemetry.addLine("")
            telemetry.addData("Error (rad)", error)
            telemetry.addData("Error (deg)", Math.toDegrees(error))
            telemetry.addData("Motor Power", motorPower)
            telemetry.addData("Slew Rate", SpindexerPIDConfig.slewRate)
            telemetry.addLine("")
            telemetry.addLine("Controls:")
            telemetry.addLine("  LB/RB: Change ball count")
            telemetry.addLine("  Right Stick Y: Adjust target")
            telemetry.addLine("  D-Pad Up/Down: Step +/- 120°")
            telemetry.addLine("  A: Reset to 0")
            telemetry.update()

            false // continue loop
        }
    }
}
