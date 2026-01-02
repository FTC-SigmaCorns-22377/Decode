package sigmacorns.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.panels.Panels
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.control.PIDController
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@Configurable
object SpindexerPIDConfig {
    @JvmField var kP = 0.001
    @JvmField var kD = 0.0
    @JvmField var kI = 0.0
}

@TeleOp(name = "Spindexer PID Tuner", group = "Test")
class SpindexerPIDTuner : SigmaOpMode() {

    override fun runOpMode() {
        // Encoder ticks per revolution (goBilda motor with 10:1 gear ratio)
        // Modern Robotics motor: 28 CPR, with 10:1 external = 280 ticks per output revolution
        val ticksPerRev = 28.0 * 10.0
        val ticksPerRadian = ticksPerRev / (2 * PI)

        var targetPosition = 0.0 // in radians

        val pid = PIDController(
            kp = SpindexerPIDConfig.kP,
            kd = SpindexerPIDConfig.kD,
            ki = SpindexerPIDConfig.kI,
            setpoint = targetPosition
        )

        telemetry.addLine("Spindexer PID Tuner")
        telemetry.addLine("Use Panels dashboard to adjust kP, kD, kI")
        telemetry.addLine("Use gamepad to control target position")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Update PID gains from Panels configurables
            pid.kp = SpindexerPIDConfig.kP
            pid.kd = SpindexerPIDConfig.kD
            pid.ki = SpindexerPIDConfig.kI

            // Control target position with gamepad
            // Right stick Y: adjust target position continuously
            targetPosition += -gamepad1.right_stick_y * 2.0 * dt.inWholeMilliseconds / 1000.0

            // D-pad for discrete position steps (120 degrees = 2π/3 radians, like spindexer slots)
            val slotAngle = (2 * PI) / 3
            if (gamepad1.dpad_up) {
                targetPosition += slotAngle
                while (gamepad1.dpad_up && opModeIsActive()) { idle() }
            }
            if (gamepad1.dpad_down) {
                targetPosition -= slotAngle
                while (gamepad1.dpad_down && opModeIsActive()) { idle() }
            }

            // Reset position with A button
            if (gamepad1.a) {
                targetPosition = 0.0
                while (gamepad1.a && opModeIsActive()) { idle() }
            }

            // Get current position in radians
            val currentPositionTicks = io.spindexerPosition()
            val currentPositionRad = currentPositionTicks / ticksPerRadian

            // Update PID setpoint and compute output
            pid.setpoint = targetPosition
            val motorPower = pid.update(currentPositionRad, dt).coerceIn(-1.0, 1.0)

            // Apply motor power
            io.spindexer = motorPower

            // Calculate error
            val error = targetPosition - currentPositionRad

            val telemetry = PanelsTelemetry.telemetry

            // Telemetry
            telemetry.addLine("=== Spindexer PID Tuner ===")
            telemetry.addLine("")
            telemetry.addData("kP", SpindexerPIDConfig.kP)
            telemetry.addData("kD",SpindexerPIDConfig.kD)
            telemetry.addData("kI", SpindexerPIDConfig.kI)
            telemetry.addLine("")
            telemetry.addData("Target (rad)",  targetPosition)
            telemetry.addData("Target (deg)",  Math.toDegrees(targetPosition))
            telemetry.addData("Current (rad)",  currentPositionRad)
            telemetry.addData("Current (deg)",  Math.toDegrees(currentPositionRad))
            telemetry.addData("Current (ticks)",  currentPositionTicks)
            telemetry.addLine("")
            telemetry.addData("Error (rad)", error)
            telemetry.addData("Error (deg)",  Math.toDegrees(error))
            telemetry.addData("Motor Power",  motorPower)
            telemetry.addLine("")
            telemetry.addLine("Controls:")
            telemetry.addLine("  Right Stick Y: Adjust target")
            telemetry.addLine("  D-Pad Up/Down: Step 120°")
            telemetry.addLine("  A: Reset to 0")
            telemetry.update()

            false // continue loop
        }
    }
}
