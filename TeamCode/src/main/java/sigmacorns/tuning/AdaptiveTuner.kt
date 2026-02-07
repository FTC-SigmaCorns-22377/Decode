package sigmacorns.tuning

/**
 * Simple interpolation-based shot tuner.
 * Stores (distance, speed) data points and linearly interpolates between them.
 * No physics-based estimation — user sets the first two points manually,
 * then the tuner interpolates/extrapolates from the nearest neighbors.
 */
class AdaptiveTuner(
    private val dataStore: ShotDataStore
) {
    companion object {
        const val MAX_FLYWHEEL_SPEED = 628.0  // rad/s (~6000 RPM)
        const val MIN_FLYWHEEL_SPEED = 50.0   // rad/s
    }

    /**
     * Get the recommended speed for a given distance by linearly interpolating
     * between the stored data points. Returns null if fewer than 2 points exist.
     */
    fun getRecommendedSpeed(distance: Double): Double? {
        val points = dataStore.getPoints()
        if (points.size < 2) return null

        val sorted = points.sortedBy { it.distance }

        // Find the two nearest bracketing points
        val lower = sorted.lastOrNull { it.distance <= distance }
        val upper = sorted.firstOrNull { it.distance > distance }

        val speed = when {
            lower != null && upper != null -> {
                // Interpolate between lower and upper
                val t = (distance - lower.distance) / (upper.distance - lower.distance)
                lower.speed + t * (upper.speed - lower.speed)
            }
            lower != null -> {
                // Beyond the highest point — extrapolate from last two
                val last = sorted[sorted.size - 1]
                val secondLast = sorted[sorted.size - 2]
                val slope = (last.speed - secondLast.speed) / (last.distance - secondLast.distance)
                last.speed + slope * (distance - last.distance)
            }
            upper != null -> {
                // Below the lowest point — extrapolate from first two
                val first = sorted[0]
                val second = sorted[1]
                val slope = (second.speed - first.speed) / (second.distance - first.distance)
                first.speed + slope * (distance - first.distance)
            }
            else -> return null
        }

        return speed.coerceIn(MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED)
    }

    /**
     * Add a new data point. If a point already exists at this exact distance,
     * it will be a separate entry (use updatePoint to modify existing).
     */
    fun addPoint(distance: Double, speed: Double) {
        dataStore.addPoint(SpeedPoint(distance, speed.coerceIn(MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED)))
    }

    /**
     * Update an existing data point by index.
     */
    fun updatePoint(index: Int, distance: Double, speed: Double) {
        dataStore.updatePoint(index, SpeedPoint(distance, speed.coerceIn(MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED)))
    }

    /**
     * Remove a data point by index.
     */
    fun removePoint(index: Int) {
        dataStore.removePoint(index)
    }

    /**
     * Get all data points sorted by distance.
     */
    fun getPointsSorted(): List<SpeedPoint> {
        return dataStore.getPoints().sortedBy { it.distance }
    }

    /**
     * Number of data points.
     */
    fun pointCount(): Int = dataStore.getPoints().size

    /**
     * Whether we have enough points to interpolate.
     */
    fun canInterpolate(): Boolean = dataStore.getPoints().size >= 2
}
