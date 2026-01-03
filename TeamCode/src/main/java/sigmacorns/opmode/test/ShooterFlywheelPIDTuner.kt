package sigmacorns.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Configurable
object ShooterFlywheelPIDConfig {
    @JvmField var kP = 0.001
    @JvmField var kD = 0.0
    @JvmField var kI = 0.0
    /** Max velocity at power 1.0: 6000 RPM = 628.3 rad/s */
    @JvmField var maxVelocity = 6000.0 * 2 * Math.PI / 60.0  // rad/s
}

@TeleOp(name = "Shooter Flywheel PID Tuner", group = "Test")
class ShooterFlywheelPIDTuner : BasePIDTuner() {

    override val tunerName = "Shooter Flywheel"
    override val unitLabel = "rad/s"
    override val showDegrees = false  // Velocity doesn't make sense in degrees
    override val discreteStep = 10.0  // 10 rad/s steps
    override val continuousRate = 20.0  // 20 rad/s per second of stick input

    override fun getKp() = ShooterFlywheelPIDConfig.kP
    override fun getKd() = ShooterFlywheelPIDConfig.kD
    override fun getKi() = ShooterFlywheelPIDConfig.kI

    override fun getFeedforward(target: Double): Double {
        return target / ShooterFlywheelPIDConfig.maxVelocity
    }

    override fun readCurrentValue(): Double {
        return io.flywheelVelocity()
    }

    override fun applyMotorPower(power: Double) {
        io.shooter = power
    }
}
