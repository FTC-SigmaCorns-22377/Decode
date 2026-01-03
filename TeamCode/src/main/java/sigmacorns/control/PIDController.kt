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
    private var lastError: Double? = null

    fun update(measured_state: Double, dt: Duration): Double{
        val t = dt.toDouble(DurationUnit.SECONDS)
        val error = setpoint-measured_state
        integral += error*t
        val dError = if(lastError!=null) kd*(error - lastError!!)/t else 0.0

        return kp*error - kd*dError + ki*integral
    }

    fun reset() {
        integral = 0.0
        lastError = null
    }
}
