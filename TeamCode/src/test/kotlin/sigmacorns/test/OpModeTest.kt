package sigmacorns.test

import org.junit.jupiter.api.Test
import sigmacorns.io.SigmaIO
import sigmacorns.io.SimIO
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.test.DrivetrainModelValidationTest
import sigmacorns.opmode.test.MPCForward
import sigmacorns.opmode.test.MPCReturn


class OpModeTest {
    @Test
    fun opModeTest() {
        SigmaOpMode.SIM = true
        SigmaOpMode.LIMELIGHT_CONNECTED = false
        val opmode = MPCForward()

        opmode.runOpMode()
    }
}