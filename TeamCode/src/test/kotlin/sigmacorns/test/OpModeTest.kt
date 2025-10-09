package sigmacorns.test

import org.junit.jupiter.api.Test
import sigmacorns.io.SigmaIO
import sigmacorns.io.SimIO
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.test.MPCBenchmarkTest
import sigmacorns.opmode.test.MPCSingleContourTest


class OpModeTest {
    @Test
    fun opModeTest() {
        SigmaOpMode.SIM = true
        SigmaOpMode.LIMELIGHT_CONNECTED = false
        val opmode = MPCBenchmarkTest()

        opmode.runOpMode()
    }
}