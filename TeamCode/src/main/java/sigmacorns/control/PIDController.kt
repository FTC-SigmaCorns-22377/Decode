package sigmacorns.control
import kotlin.math.abs


class PIDController(
    var kp: Float,
    var kd: Float,
    var ki: Float,
    var integralterm: Float,
    var init_error: Float,
    var setpoint: Float,
    var statepoint:Float
) {

    fun update(measured_state: Float, dt: Float): Float{
        val error = abs(measured_state - setpoint)
        val p = kp*error

        integralterm += error * dt
        val i =  ki  * integralterm

        val d = kd* (error - init_error)/dt

        return p + i + d

    }


}
