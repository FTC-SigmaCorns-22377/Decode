package sigmacorns.control.subsystem

import sigmacorns.io.SigmaIO
import sigmacorns.sim.LinearDcMotor
import kotlin.math.exp
import kotlin.math.ln
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

/**
 * Assumes that the motor power will be updated in fixed timesteps. Calculates the power that results
 * in the flywheel being at the target at the start of the next timestep
 *
 * This is equivalent to a 1st-order deadbeat controller
 *
 * @param motor the motor powering the flywheel
 * @param inertia the inertia of the flywheel system (kg m^2)
 * @param io IO handle
 * @param lag expected delay between sensor measurement and motor power write.
 */
class Flywheel(
    val motor: LinearDcMotor,
    val inertia: Double,
    val io: SigmaIO,
    val lag: Duration = 0.seconds
) {
    /*
    math:
        dv/dt = u tau_max/inertia (1-v/(u v_max))

        a = tau_max/inertia

        dv/dt = u a - a/v_max v

        v(t)= u v_max + (v_0 - u v_max)e^(-a/v_max t)

        -- need to find u such that v(DT) = v_target
        v_target = u v_max + (v_0 - u v_max)e^(-a/v_max DT)
        v_target = u*v_max + v_0*e^(-a/v_max DT) - u*v_max*e^(-a/v_max DT)
        v_target - v_0*e^(-a/v_max DT)= u(v_max - v_max*e^(-a/v_max DT))
        (v_target - v_0*e^(-a/v_max DT))/(v_max - v_max*e^(-a/v_max DT)) = u
     */

    var target: Double = 0.0
    var hold: Boolean = false

    private var lastU = 0.0

    private fun calculate(v0: Double, dt: Duration): Double {
        val e = exp(-motor.stallTorque/inertia/motor.freeSpeed * dt.toDouble(DurationUnit.SECONDS))
        val power = (target - v0*e)/(motor.freeSpeed * (1.0-e))

        // verification: if target=2 & v0=2
        // power = (target - v0*e)/(v_max*(1-e))
        // power = (2-2e)/(v_max*(1-e))
        // power = (2*(1-e))/(v_max*(1-e))
        // power = 2/v_max -> yup, holds target at necessary power

        return power * motor.vRef
    }

    fun update(curV: Double, dt: Duration) {
        val V = if (hold) {
            target/motor.freeSpeed*motor.vRef
        } else {
            val v0 = integrate(curV,lastU,lag)
            calculate(v0, dt)
        }

        io.shooter = (V/io.voltage()).coerceIn(-1.0..1.0)
        lastU = V.coerceIn(-io.voltage()..io.voltage())
    }

    fun integrate(v0: Double, u: Double, t: Duration): Double {
        val a = motor.stallTorque/inertia
        val s = u*motor.freeSpeed
        return s + (v0 - s)*exp(-a/motor.freeSpeed*t.toDouble(DurationUnit.SECONDS))
    }

    fun spinupTime(cur: Double, target: Double, dt: Duration): Duration {
        /*
        If the target cannot be reached in DT, this controller will saturate motor limits
        therefore an upper bound for the spinup time is the time to reach target under full power + DT

        v_taget = v_max + (v_0 - v_max)e^(-a/v_max t)
        -- solve for t when u=1--
        (v_taget - v_max)/(v_0 - v_max) = e^(-a/v_max t)
        ln((v_taget - v_max)/(v_0 - v_max))/(-a/v_max) = t
        */

        val t = ln((target - motor.freeSpeed)/(cur - motor.freeSpeed))/(-motor.stallTorque/inertia/motor.freeSpeed)

        return t.seconds + dt + lag
    }
}