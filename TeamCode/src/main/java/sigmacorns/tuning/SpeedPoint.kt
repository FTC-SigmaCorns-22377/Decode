package sigmacorns.tuning

/**
 * A single data point mapping distance to flywheel speed.
 */
data class SpeedPoint(
    val distance: Double,  // meters
    val speed: Double      // rad/s
)
