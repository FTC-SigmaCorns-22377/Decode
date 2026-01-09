package sigmacorns.control

import sigmacorns.io.SigmaIO
import sigmacorns.opmode.tune.SpindexerPIDConfig
import kotlin.ranges.coerceIn
import kotlin.time.Duration

class Spindexer(
    val range: MotorRangeMapper,
    val io: SigmaIO
) {
    val spindexerPID = PIDController(0.25, 10.0, 0.0, 0.0)
    val slewRateLimiter = SlewRateLimiter(maxRate = SpindexerPIDConfig.slewRate)

    /** Whether slew rate limiting is enabled */
    var slewRateLimitingEnabled: Boolean = true

    var target: Double = 0.0
    var curRotation: Double = 0.0

    fun update(dt: Duration, motorTick: Double, count: Int) {
        curRotation = range.tickToPos(motorTick)

        val slewLimitedTarget = if (slewRateLimitingEnabled) {
            slewRateLimiter.maxRate = SpindexerPIDConfig.slewRate
            slewRateLimiter.calculate(target, dt)
        } else {
            target
        }

        val effectiveTargetRotation = slewLimitedTarget

        // Update PID for spindexer motor position
        spindexerPID.kp = SpindexerPIDConfig.getKp(count)
        spindexerPID.kd = SpindexerPIDConfig.getKd(count)
        spindexerPID.ki = SpindexerPIDConfig.getKi(count)

        spindexerPID.setpoint = effectiveTargetRotation
        val power = spindexerPID.update(curRotation, dt).coerceIn(-1.0, 1.0)
        io.spindexer = power.coerceIn(-0.5, 0.5)
    }
}