package sigmacorns.opmode

import org.junit.jupiter.api.Test
import sigmacorns.State
import sigmacorns.opmode.auto.AutoBlueFarFull
import sigmacorns.opmode.teleop.TeleopBlue
import sigmacorns.opmode.test.MPCForward
import sigmacorns.opmode.test.MPCTest
import kotlin.time.Duration

class TestOpMode : SigmaOpMode() {
    override fun runOpMode() {
        waitForStart()
        ioLoop { state, dt ->
            io.driveFL = -0.5
            io.driveFR = 0.5
            io.driveBL = 0.5
            io.driveBR = -0.5
            io.turret = 0.0
            
            // Stop after 10 seconds
            if (io.time().inWholeSeconds > 10) return@ioLoop true
            false
        }
    }
}

class SigmaOpModeTest {
    @Test
    fun testDrakeSimIntegration() {
        SigmaOpMode.SIM = true
        SigmaOpMode.LIMELIGHT_CONNECTED = false
        val opMode = AutoBlueFarFull()
        try {
            opMode.runOpMode()
        } finally {
            opMode.cleanup()
        }
    }
}
