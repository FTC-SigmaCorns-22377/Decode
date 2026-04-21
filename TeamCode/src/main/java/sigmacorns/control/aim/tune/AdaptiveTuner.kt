package sigmacorns.control.aim.tune

/**
 * Interpolation-based shot tuner.
 * Stores (distance, speed, hoodAngle) data points and linearly interpolates.
 */
class AdaptiveTuner(
    private val dataStore: ShotDataStore
) {
    companion object {
        const val MAX_FLYWHEEL_SPEED = 628.0  // rad/s (~6000 RPM)
        const val MIN_FLYWHEEL_SPEED = 50.0   // rad/s
        const val MIN_HOOD_ANGLE = 15.0       // degrees
        const val MAX_HOOD_ANGLE = 70.0       // degrees
    }

    /**
     * Get the recommended speed for a given distance by linearly interpolating.
     * Returns null if fewer than 2 points exist.
     */
    fun getRecommendedSpeed(distance: Double): Double? {
        return interpolate(distance) { it.speed }?.coerceIn(MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED)
    }

    /**
     * Get the recommended hood angle (degrees) for a given distance.
     * Returns null if fewer than 2 points exist.
     */
    fun getRecommendedHoodAngle(distance: Double): Double? {
        return interpolate(distance) { it.hoodAngle }?.coerceIn(MIN_HOOD_ANGLE, MAX_HOOD_ANGLE)
    }

    private fun interpolate(distance: Double, extract: (SpeedPoint) -> Double): Double? {
        val points = dataStore.getPoints()
        if (points.size < 2) return null

        val sorted = points.sortedBy { it.distance }
        val lower = sorted.lastOrNull { it.distance <= distance }
        val upper = sorted.firstOrNull { it.distance > distance }

        return when {
            lower != null && upper != null -> {
                val t = (distance - lower.distance) / (upper.distance - lower.distance)
                extract(lower) + t * (extract(upper) - extract(lower))
            }
            lower != null -> {
                val last = sorted[sorted.size - 1]
                val secondLast = sorted[sorted.size - 2]
                val slope = (extract(last) - extract(secondLast)) / (last.distance - secondLast.distance)
                extract(last) + slope * (distance - last.distance)
            }
            upper != null -> {
                val first = sorted[0]
                val second = sorted[1]
                val slope = (extract(second) - extract(first)) / (second.distance - first.distance)
                extract(first) + slope * (distance - first.distance)
            }
            else -> null
        }
    }

    fun addPoint(distance: Double, speed: Double, hoodAngle: Double = 45.0) {
        dataStore.addPoint(SpeedPoint(
            distance,
            speed.coerceIn(MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED),
            hoodAngle.coerceIn(MIN_HOOD_ANGLE, MAX_HOOD_ANGLE)
        ))
    }

    fun updatePoint(index: Int, distance: Double, speed: Double, hoodAngle: Double = 45.0) {
        dataStore.updatePoint(index, SpeedPoint(
            distance,
            speed.coerceIn(MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED),
            hoodAngle.coerceIn(MIN_HOOD_ANGLE, MAX_HOOD_ANGLE)
        ))
    }

    fun removePoint(index: Int) {
        dataStore.removePoint(index)
    }

    fun getPointsSorted(): List<SpeedPoint> {
        return dataStore.getPoints().sortedBy { it.distance }
    }

    fun pointCount(): Int = dataStore.getPoints().size

    fun canInterpolate(): Boolean = dataStore.getPoints().size >= 2

    fun save() = dataStore.save()
}
