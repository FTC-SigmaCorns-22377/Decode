package sigmacorns.sim

/**
 * Simple ODE-based spindexer inertia model using a linear DC motor torque curve.
 */

//class spindexerDynamics (val p: spindexerParameters) {
//    private fun dx(u:DoubleArray, x:DoubleArray): DoubleArray {
//        val omega = x[0]
//
//        val torque = u.fold(0.0) {acc, power ->
//            acc + p.spinMotor.torque(power,omega)
//        }
//
//        val damping = p.spinViscousFriction * omega
//
//        val alpha = if (p.inertia > 0.0) (torque-damping) /p.inertia else 0.0
//
//        return doubleArrayOf(alpha)
//    }
//
//    fun integrate(tf: Double, dt: Double, u: DoubleArray, x: spindexerState): spindexerState {
//        val x0 = x.toDoubleArray()
//
//        val xf = rk4Integrate(tf, dt, x0) {state ->dx(u, state)}
//
//        return spindexerState(xf)
//
//    }
//
//}

class spindexerDynamics(val p: spindexerParameters) {
    private fun dx(u: DoubleArray, x: DoubleArray): DoubleArray {
        val omega = x[0]

        val torque = u.fold(0.0) { acc, power ->
            acc + p.spinMotor.torque(power, omega)
        }

        val damping = p.spinViscousFriction * omega

        val alpha = if (p.inertia > 0.0) (torque - damping) / p.inertia else 0.0

        return doubleArrayOf(alpha)
    }

    fun integrate(tf: Double, dt: Double, u: DoubleArray, x: spindexerState): spindexerState {
        val x0 = x.toDoubleArray()

        val xf = rk4Integrate(tf, dt, x0) { state -> dx(u, state) }

        return spindexerState(xf)
    }
}