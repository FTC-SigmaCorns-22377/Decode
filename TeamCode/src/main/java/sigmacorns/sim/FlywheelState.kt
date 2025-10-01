package sigmacorns.sim

/**
 * The state of the flywheel, captured as its angular velocity in rad/s.
 */
data class FlywheelState(
    var omega: Double = 0.0,
) {
    constructor(data: DoubleArray) : this(
        omega = data[0],
    )

    fun toDoubleArray(): DoubleArray = doubleArrayOf(omega)
}
