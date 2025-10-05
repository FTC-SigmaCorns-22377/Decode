package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import sigmacorns.io.SigmaIO
import kotlin.time.Duration.Companion.seconds

abstract class SigmaOpMode(val io: SigmaIO): LinearOpMode() {
    var SIM: Boolean = false
    var LIMELIGHT_CONNECTED: Boolean = true
    var SIM_INIT_TIME = 1.seconds

    override fun waitForStart() {
        return
    }
}