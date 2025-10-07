package sigmacorns.test

import org.junit.jupiter.api.Test
import sigmacorns.io.SigmaIO
import sigmacorns.io.SimIO
import sigmacorns.opmode.test.FlywheelSpinupTest
import sigmacorns.opmode.test.MPCBenchmarkTest
import sigmacorns.opmode.test.MPCSingleContourTest


class OpModeTest {
    @Test
    fun opModeTest() {
        val opmode = MPCBenchmarkTest(SimIO())
        opmode.LIMELIGHT_CONNECTED = false

        opmode.runOpMode()
    }
}