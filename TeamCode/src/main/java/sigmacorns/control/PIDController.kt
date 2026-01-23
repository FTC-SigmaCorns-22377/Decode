package sigmacorns.control
import kotlin.time.Duration
import kotlin.time.DurationUnit


class PIDController(
    var kp: Double,
    var kd: Double,
    var ki: Double,
    var setpoint: Double,
) {
    private var integral: Double = 0.0
    private var lastMeasurement: Double? = null

    fun update(measured_state: Double, dt: Duration): Double{
        val t = dt.toDouble(DurationUnit.SECONDS)
        val error = setpoint-measured_state
        integral += error*t

        // Derivative on measurement (not error) to prevent derivative kick on setpoint changes
        // This is critical for field-relative control where setpoint changes continuously
        val dMeasurement = if(lastMeasurement!=null && t > 0.0) (measured_state - lastMeasurement!!)/t else 0.0

        lastMeasurement = measured_state

        // Note: -kd*dMeasurement is equivalent to -kd*dError when setpoint is constant,
        // but handles setpoint changes smoothly by ignoring d(setpoint)/dt
        return kp*error - kd*dMeasurement + ki*integral
    }

    fun reset() {
        integral = 0.0
        lastMeasurement = null
    }
}
