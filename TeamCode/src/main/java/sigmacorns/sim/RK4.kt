package sigmacorns.sim

/**
 * Performs a single step of the 4th-order Runge–Kutta (RK4) numerical integration method.
 *
 * This function advances the state `x0` forward by one timestep `dt` according to the
 * system dynamics defined by the derivative function `dx`.
 *
 * @param dt The timestep size.
 * @param x0 The current state vector.
 * @param dx A function that computes the time derivative of the state given the current state.
 * @return The updated state vector after advancing by `dt` using RK4.
 */
fun rk4Step(dt: Double, x0: DoubleArray, dx: (DoubleArray) -> DoubleArray): DoubleArray {
    val nx = x0.size
    val k1 = dx(x0)
    val k2 = dx(DoubleArray(nx) { x0[it] + k1[it]*dt/2.0 })
    val k3 = dx(DoubleArray(nx) { x0[it] + k2[it]*dt/2.0 })
    val k4 = dx(DoubleArray(nx) { x0[it] + k3[it]*dt })

    return DoubleArray(nx) {
        x0[it] + dt/6.0 * (k1[it] + 2.0*k2[it] + 2.0*k3[it] + k4[it])
    }
}
/**
 * Integrates a system of ordinary differential equations from time 0 to `tf`
 * using fixed-step 4th-order Runge–Kutta (RK4).
 *
 * The integration proceeds in steps of size `dt`, with the last step adjusted to exactly
 * reach `tf`.
 *
 * @param tf The final integration time.
 * @param dt The step size for integration.
 * @param x0 The initial state vector at time `t = 0`.
 * @param dx A function that computes the time derivative of the state given the current state.
 * @return The state vector at time `tf`.
 */
fun rk4Integrate(tf: Double, dt: Double, x0: DoubleArray, dx: (DoubleArray) -> DoubleArray): DoubleArray {
    var t = 0.0
    var x = x0.copyOf()
    while (t+dt<tf) {
        x = rk4Step(dt,x,dx)
        t += dt
    }
    x = rk4Step(tf-t,x,dx)
    return x
}