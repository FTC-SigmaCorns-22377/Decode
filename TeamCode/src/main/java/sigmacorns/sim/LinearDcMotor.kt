package sigmacorns.sim

/**
 * Linear DC motor model that maps commanded power and shaft speed to torque output.
 */
class LinearDcMotor(
    val freeSpeed: Double,
    val stallTorque: Double,
    val vRef: Double = 12.0,
) {
    fun torque(power: Double, motorOmega: Double, supplyVoltage: Double = vRef): Double {
        val commandedPower = power.coerceIn(-1.0, 1.0)
        val voltageScale = if (vRef > 0.0) supplyVoltage / vRef else 1.0
        val torque = stallTorque * (commandedPower * voltageScale - motorOmega / freeSpeed)

        return torque
    }
}
