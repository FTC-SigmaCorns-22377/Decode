package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.control.MotorRangeMapper
import sigmacorns.control.subsystem.Turret
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@TeleOp(name = "Turret Angle Test", group = "Test")
class TurretTest : SigmaOpMode() {

    override fun runOpMode() {
        var angle = 0.0

        val ticksPerRev = (1.0+46.0/11.0)*28.0

        val turret = Turret(MotorRangeMapper(
            -PI/2..PI/2,
            -ticksPerRev*0.5..ticksPerRev*0.5,
            0.1
        ),io)

        waitForStart()

        ioLoop { state, dt ->
            // Control turretAngle (servo) with dpad or sticks
            angle += 0.2 * (gamepad1.right_stick_x) * dt.inWholeMilliseconds / 1000.0

            // Also allow stick control for faster movement
            if (gamepad1.right_stick_y != 0.0f) {
                angle -= gamepad1.right_stick_y * 1.0 * dt.inWholeMilliseconds / 1000.0
            }

            turret.targetAngle = angle
            turret.update(dt)

            //io.turret = io.turret.sign * min(io.turret.absoluteValue,0.15)


            telemetry.addData("Turret Target Angle ", "%.3f", turret.targetAngle)
            telemetry.addData("Turret Angle ", "%.3f", turret.pos)
            telemetry.addData("Turret Power", "%.3f", io.turret)
            telemetry.addData("Turret Pos (direct)", "%.3f", io.turretPosition())
            telemetry.addLine("Use Dpad Up/Down or Right Stick Y to move")
            telemetry.update()
            
            false // continue loop
        }
    }
}