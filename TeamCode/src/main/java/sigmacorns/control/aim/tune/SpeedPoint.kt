package sigmacorns.control.aim.tune

/**
 * A single data point mapping distance to flywheel speed and hood angle.
 */
data class SpeedPoint(
    val distance: Double,   // meters
    val speed: Double,      // rad/s
    val hoodAngle: Double = 45.0  // degrees
)
