package sigmacorns.sim

/**
 * Parameters for a Mecanum drivetrain model
 *
 * @param freeSpeed top speed of the motor (rad/s)
 * @param stallTorque stall torque of the motor (Nm)
 * @param lx distance to wheel (m)
 * @param ly distance to wheel (m)
 * @param wheelRadius wheel radius (m)
 * @param weight weight of the robot (kg)
 * @param rotInertia rotational inertia of the robot about the wheelbase center (kg m^2)
 */
class MecanumParameters(
    var freeSpeed: Double,
    var stallTorque: Double,
    var lx: Double,
    var ly: Double,
    var wheelRadius: Double,
    var weight: Double,
    var rotInertia: Double,

    val vRef: Number = 12.0,
) {
    fun toArray(): DoubleArray = doubleArrayOf(freeSpeed, stallTorque, lx,ly,wheelRadius,weight,rotInertia)
}
