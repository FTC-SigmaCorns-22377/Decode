package sigmacorns.sim


/**
 * Parameters for the flywheel model.
 *
 * @param motor the motor powering the flywheel
 * @param inertia total rotational inertia of the flywheel (kg·m^2)
 * @param viscousFriction viscous damping coefficient applied at the flywheel (N·m·s)
 */
data class FlywheelParameters(
    val motor: LinearDcMotor,
    val inertia: Double,
    val viscousFriction: Double = 0.0,
) {
    /**
     * The mechanical time constant for the flywheel system
     *
     * @return the time (s) it takes for the flywheel system to reach 63% of its final velocity
     */
    fun velTimeConstant() = inertia*motor.freeSpeed/motor.stallTorque
}
