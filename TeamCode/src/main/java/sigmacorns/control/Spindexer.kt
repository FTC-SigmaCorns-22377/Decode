package sigmacorns.control

import sigmacorns.io.SigmaIO
import sigmacorns.opmode.tune.SpindexerPIDConfig
import kotlin.ranges.coerceIn
import kotlin.time.Duration
import kotlin.time.DurationUnit

class Spindexer(
    val range: MotorRangeMapper,
    val io: SigmaIO
) {
    val spindexerPID = PIDController(
        SpindexerPIDConfig.kP,
        SpindexerPIDConfig.kD,
        SpindexerPIDConfig.kI,
        0.0
    )
    
    val profile = TrapezoidalProfile(
        SpindexerPIDConfig.maxVelocity,
        SpindexerPIDConfig.maxAcceleration
    )

    /** Whether profiling is enabled */
    var profilingEnabled: Boolean = true

    var target: Double = 0.0
    var curRotation: Double = 0.0
    private var prevRotation: Double = 0.0
    
    private var isInitialized = false
    private val lagThreshold = 0.3 // radians

    fun update(dt: Duration) {
        curRotation = range.tickToPos(io.spindexerPosition())
        val dtSeconds = dt.toDouble(DurationUnit.SECONDS)
        
        // Calculate actual velocity
        val actualVelocity = if (dtSeconds > 0) (curRotation - prevRotation) / dtSeconds else 0.0
        prevRotation = curRotation
        
        if (!isInitialized) {
            profile.reset(curRotation, actualVelocity)
            isInitialized = true
        }

        // Update profile configs in case they changed (tuning)
        profile.maxVelocity = SpindexerPIDConfig.maxVelocity
        profile.maxAcceleration = SpindexerPIDConfig.maxAcceleration

        // Check if we are falling behind the profile
        // If profile velocity is non-zero and error is in the direction of motion and exceeds threshold
        if (profilingEnabled && kotlin.math.abs(profile.currentVelocity) > 1e-6) {
            val error = profile.currentPosition - curRotation
            // If moving positive, error > threshold means we are lagging below profile
            // If moving negative, error < -threshold means we are lagging above profile
            // Combined: error * sign(velocity) > threshold
            if (error * kotlin.math.sign(profile.currentVelocity) > lagThreshold) {
                 profile.reset(curRotation, actualVelocity)
            }
        }

        val effectiveTargetRotation = if (profilingEnabled) {
            val profileState = profile.calculate(target, dtSeconds)
            profileState.position
        } else {
            target
        }

        // Update PID for spindexer motor position
        spindexerPID.kp = SpindexerPIDConfig.kP
        spindexerPID.kd = SpindexerPIDConfig.kD
        spindexerPID.ki = SpindexerPIDConfig.kI

        spindexerPID.setpoint = effectiveTargetRotation
        val power = spindexerPID.update(curRotation, dt).coerceIn(-1.0, 1.0)
        io.spindexer = power
    }
}