package sigmacorns.sim

/**
 * Parameters for a Mecanum drivetrain model
 *
 * @param motor the motors powering the drive shafts
 * @param lx distance to wheel (m)
 * @param ly distance to wheel (m)
 * @param wheelRadius wheel radius (m)
 * @param weight weight of the robot (kg)
 * @param rotInertia rotational inertia of the robot about the wheelbase center (kg m^2)
 */
class MecanumParameters(
    var motor: LinearDcMotor,
    var lx: Double,
    var ly: Double,
    var wheelRadius: Double,
    var weight: Double,
    var rotInertia: Double,
) {
    fun toArray(): DoubleArray = doubleArrayOf(motor.freeSpeed, motor.stallTorque, lx,ly,wheelRadius,weight,rotInertia)
}
