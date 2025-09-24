package sigmacorns.sim

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