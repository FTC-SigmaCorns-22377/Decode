package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode

@TeleOp(name = "Hood Zero Test", group = "Test")
class HoodZeroTest : SigmaOpMode() {
    override fun runOpMode() {
        waitForStart()
        while (opModeIsActive()) {
            io.hood = 0.0
            io.update()
        }
    }
}
