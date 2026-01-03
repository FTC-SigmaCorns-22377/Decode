package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode

@TeleOp(group = "test")
class DistanceSensorTest : SigmaOpMode() {
    override fun runOpMode() {
        telemetry.addLine("Distance Sensor Test ready")
        telemetry.addLine("Distance sensor will read continuously")
        telemetry.update()

        waitForStart()

        ioLoop { state, _ ->
            val distance = io.distance()

            telemetry.addData("Distance (m)", "%.3f", distance)
            telemetry.addData("Distance (cm)", "%.1f", distance * 100)
            telemetry.addData("Distance (in)", "%.1f", distance * 39.3701)
            telemetry.update()

            false
        }
    }
}
