package sigmacorns.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.control.SlewRateLimiter
import kotlin.math.PI

@Configurable
object TurretPIDConfig {
    @JvmField var kP = 1.7
    @JvmField var kD = 0.005
    @JvmField var kI = 0.0
    @JvmField var slewRate = 2.0
    @JvmField var outputSlewRate = 4.0
}

@TeleOp(name = "Turret PID Tuner", group = "Test")
class TurretPIDTuner : BasePIDTuner() {

    private val ticksPerRev = (1.0 + (46.0 / 11.0)) * 28.0
    private val ticksPerRadian = ticksPerRev / (2 * PI) * 76 / 19

    override val tunerName = "Turret"
    override val unitLabel = "rad"
    override val discreteStep = PI / 4  // 45 degrees
    override val continuousRate = 1.0  // Slower rate for precise turret control

    // Enable slew rate limiting for smoother turret control
    override val slewRateLimiter = SlewRateLimiter(TurretPIDConfig.slewRate)
    override val outputSlewRateLimiter = SlewRateLimiter(TurretPIDConfig.outputSlewRate)

    override fun getKp() = TurretPIDConfig.kP
    override fun getKd() = TurretPIDConfig.kD
    override fun getKi() = TurretPIDConfig.kI
    override fun getSlewRate() = TurretPIDConfig.slewRate
    override fun getOutputSlewRate() = TurretPIDConfig.outputSlewRate

    override fun readCurrentValue(): Double {
        val currentPositionTicks = io.turretPosition()
        return currentPositionTicks / ticksPerRadian
    }

    override fun applyMotorPower(power: Double) {
        io.turret = power
    }
}
