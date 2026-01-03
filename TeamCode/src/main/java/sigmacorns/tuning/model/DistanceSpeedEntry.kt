package sigmacorns.tuning.model

/**
 * A confirmed optimal speed for a distance bucket.
 */
data class DistanceSpeedEntry(
    val distanceMin: Double,       // meters (bucket start)
    val distanceMax: Double,       // meters (bucket end)
    val optimalSpeed: Double,      // rad/s
    val confirmedTrials: Int,      // number of "Good" confirmations
    val lastUpdated: Long          // epoch millis
)
