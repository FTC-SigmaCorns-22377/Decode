package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.robocol.TelemetryMessage
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.io.HardwareIO
import sigmacorns.io.RerunLogging
import sigmacorns.io.SigmaIO
import sigmacorns.io.SimIO
import java.io.File
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

import sigmacorns.io.DrakeSimIO

abstract class SigmaOpMode(
    private val providedIO: SigmaIO? = null
): LinearOpMode() {

    private var createdIO: SigmaIO? = null

    val io: SigmaIO by lazy {
        if (providedIO != null) return@lazy providedIO
        
        if (hardwareMap != null) {
            val hio = HardwareIO(hardwareMap)
            createdIO = hio
            return@lazy hio
        }
        
        // Sim Mode
        if (tryLoadDrake()) {
             try {
                 val urdfCandidates = listOfNotNull(
                    System.getenv("ROBOT_URDF"),
                    "TeamCode/src/main/assets/robot.urdf",
                    "src/main/assets/robot.urdf"
                 ).map { File(it) }
                 val urdfFile = urdfCandidates.firstOrNull { it.exists() }
                 if (urdfFile != null) {
                     println("Initializing DrakeSimIO with ${urdfFile.absolutePath}")
                     val drakeIO = DrakeSimIO(urdfFile.absolutePath)
                     createdIO = drakeIO
                     return@lazy drakeIO
                 }
             } catch (e: Throwable) {
                 e.printStackTrace()
                 println("Failed to init DrakeSimIO, falling back to Kotlin SimIO")
             }
        }
        
        println("Using Fallback Kotlin SimIO")
        val simIO = SimIO()
        createdIO = simIO
        simIO
    }

    override fun runOpMode() {
        // Implementers override this. 
        // We can't easily hook into the end of runOpMode here without enforcing super.runOpMode() which isn't standard in LinearOpMode 
        // unless we wrap it.
        // But for LinearOpMode, the user code IS runOpMode. 
        // So we can't auto-cleanup unless we use a different structure or users call a cleanup method.
        // HOWEVER, if we are running in a test, we can call a cleanup method.
    }
    
    fun cleanup() {
        if (createdIO is DrakeSimIO) {
            (createdIO as DrakeSimIO).close()
            println("DrakeSimIO closed.")
        }
    }

    private fun tryLoadDrake(): Boolean {
        try {
            System.loadLibrary("native-lib")
            return true
        } catch (e: UnsatisfiedLinkError) {
            // Try manual load
            val libName = "libnative-lib.so"
            val candidates = listOf(
                File("TeamCode/build/native-desktop/lib/$libName"),
                File("build/native-desktop/lib/$libName"),
                File("src/testDebug/jniLibs/$libName") // Also check where we copied it
            )
            val lib = candidates.firstOrNull { it.exists() }
            if (lib != null) {
                try {
                    System.load(lib.absolutePath)
                    return true
                } catch (t: Throwable) {
                    t.printStackTrace()
                }
            }
        } catch (t: Throwable) {
            t.printStackTrace()
        }
        return false
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

    private val internalState = OpModeReflection(this)

    init {
        if (SIM) {
            super.gamepad1 = Gamepad()
            super.gamepad2 = Gamepad()
            super.telemetry = TelemetryImpl(this)

            val services = object : OpModeServices {
                override fun refreshUserTelemetry(
                    msg: TelemetryMessage?,
                    sInterval: Double
                ) {
                    println("TELEMETRY: ${msg?.dataStrings}")
                }

                override fun requestOpModeStop(opModeToStopIfActive: OpMode?) {
                }

            }

            internalState.internalOpModeServicesField.set(this,services)
        }
    }

    fun solverIP() = if (LIMELIGHT_CONNECTED) Network.LIMELIGHT else Network.SIM_MPC
    fun rerunIP() = if (SIM) Network.SIM_RERUN else Network.ROBOT_RERUN

    fun rerunSink(name: String) = if(SIM) RerunLogging.connect(name, Network.SIM_RERUN) else RerunLogging.save(name, "/sdcard/FIRST/$name.rrd")

    fun rerunLocation() = if (SIM) File(System.getProperty("user.dir")!!,"rerun") else File("/sdcard/FIRST/rerun")

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
            while (!opModeIsActive() && !isStopRequested) {
                idle()
            }
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
        val internalOpModeServicesField = opModeInternalClass.getDeclaredField("internalOpModeServices").apply { isAccessible = true }
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
        var LIMELIGHT_CONNECTED: Boolean = true
    }
}
