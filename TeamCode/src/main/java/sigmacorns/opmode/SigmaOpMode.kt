package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.io.SimIO
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

abstract class SigmaOpMode(
    private val providedIO: SigmaIO? = null
): LinearOpMode() {
    val io: SigmaIO by lazy {
        providedIO ?: hardwareMap?.let { HardwareIO(it) } ?: SimIO()
    }

    private val internalState by lazy { OpModeReflection(this) }

    fun solverIP() = if (LIMELIGHT_CONNECTED) Network.LIMELIGHT else Network.SIM_MPC
    fun rerunIP() = if (SIM) Network.SIM_RERUN else Network.ROBOT_RERUN

    fun ioLoop(f: (State, Duration) -> Boolean) {
        val state = State(io)
        while (opModeIsActive()) {
            val tOld = state.timestamp
            state.update(io)
            if(f(state, state.timestamp-tOld)) return
            io.update()
        }
    }

    override fun waitForStart() {
        if (!SIM) {
            super.waitForStart()
            return
        }

        if (!internalState.isStarted()) {
            internalState.markStarted()
        }
    }

    // Reflection bridge to access FTC SDK private opmode state.
    private class OpModeReflection(private val target: SigmaOpMode) {
        private val opModeInternalClass = Class.forName("com.qualcomm.robotcore.eventloop.opmode.OpModeInternal")
        private val linearOpModeClass = Class.forName("com.qualcomm.robotcore.eventloop.opmode.LinearOpMode")
        private val isStartedField = opModeInternalClass.getDeclaredField("isStarted").apply { isAccessible = true }
        private val userMonitoredForStartField = linearOpModeClass.getDeclaredField("userMonitoredForStart").apply { isAccessible = true }

        fun isStarted(): Boolean = isStartedField.getBoolean(target)

        fun markStarted() {
            isStartedField.setBoolean(target, true)
            userMonitoredForStartField.setBoolean(target, true)
        }
    }

    companion object {
        private val SIM_DEFAULT_INIT_DURATION: Duration = 1.seconds
        private val SIM_WAIT_INTERVAL: Duration = 10.milliseconds

        var SIM: Boolean = false
        var LIMELIGHT_CONNECTED: Boolean = false
    }
}