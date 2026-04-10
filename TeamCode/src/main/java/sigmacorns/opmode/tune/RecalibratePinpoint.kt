package sigmacorns.opmode.tune

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.io.HardwareIO
import sigmacorns.opmode.SigmaOpMode

@TeleOp(group = "tune")
class RecalibratePinpoint: SigmaOpMode() {
    override fun runOpMode() {
        waitForStart()
        (io as HardwareIO).pinpoint!!.recalibrateIMU()
        while (opModeIsActive()) {

        }
    }
}