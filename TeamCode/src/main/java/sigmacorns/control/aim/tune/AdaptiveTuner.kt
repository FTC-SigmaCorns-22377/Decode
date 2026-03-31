package sigmacorns.control.aim.tune

/**
 * Quadratic-fit shot tuner.
 * Stores (distance, speed, hoodAngle) data points and fits a least-squares
 * quadratic curve. Falls back to linear interpolation with only 2 points.
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
     * Get the recommended speed for a given distance via quadratic fit.
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

        // With only 2 points, fall back to linear interpolation
        if (points.size == 2) {
            val sorted = points.sortedBy { it.distance }
            val t = (distance - sorted[0].distance) / (sorted[1].distance - sorted[0].distance)
            return extract(sorted[0]) + t * (extract(sorted[1]) - extract(sorted[0]))
        }

        // Least-squares quadratic fit: y = a*x^2 + b*x + c
        // Solves the 3x3 normal equations directly
        var s0 = 0.0; var s1 = 0.0; var s2 = 0.0; var s3 = 0.0; var s4 = 0.0
        var r0 = 0.0; var r1 = 0.0; var r2 = 0.0
        for (p in points) {
            val x = p.distance; val y = extract(p)
            val x2 = x * x
            s0 += 1.0; s1 += x; s2 += x2; s3 += x2 * x; s4 += x2 * x2
            r0 += y; r1 += x * y; r2 += x2 * y
        }

        // Solve 3x3 system via Cramer's rule:
        // [s4 s3 s2] [a]   [r2]
        // [s3 s2 s1] [b] = [r1]
        // [s2 s1 s0] [c]   [r0]
        val det = s4 * (s2 * s0 - s1 * s1) - s3 * (s3 * s0 - s1 * s2) + s2 * (s3 * s1 - s2 * s2)
        if (kotlin.math.abs(det) < 1e-12) return null

        val a = (r2 * (s2 * s0 - s1 * s1) - s3 * (r1 * s0 - s1 * r0) + s2 * (r1 * s1 - s2 * r0)) / det
        val b = (s4 * (r1 * s0 - s1 * r0) - r2 * (s3 * s0 - s1 * s2) + s2 * (s3 * r0 - r1 * s2)) / det
        val c = (s4 * (s2 * r0 - r1 * s1) - s3 * (s3 * r0 - r1 * s2) + r2 * (s3 * s1 - s2 * s2)) / det

        return a * distance * distance + b * distance + c
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
