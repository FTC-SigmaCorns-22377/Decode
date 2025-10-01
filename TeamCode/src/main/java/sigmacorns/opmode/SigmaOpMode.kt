package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import sigmacorns.constants.Network
import sigmacorns.io.SigmaIO

abstract class SigmaOpMode(val io: SigmaIO): LinearOpMode() {
    var SIM: Boolean = false
    var LIMELIGHT_CONNECTED: Boolean = true

    fun solverIP() = if(LIMELIGHT_CONNECTED) Network.LIMELIGHT else Network.SIM_MPC
    fun rerunIP() = if(SIM) Network.SIM_RERUN else Network.ROBOT_RERUN

    override fun waitForStart() {
        if(SIM) {
            return
        } else {
            super.waitForStart()
        }
    }
}