package sigmacorns.control

import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.time.Duration
import kotlin.time.DurationUnit

/**
 * Limits the rate of change of a value.
 * Useful for smoothing motor commands to prevent mechanical stress and current spikes.
 *
 * @param maxRate Maximum rate of change per second (e.g., 2.0 means value can change by at most 2.0/sec)
 */
class SlewRateLimiter(
    var maxRate: Double
) {
    private var previousValue: Double? = null

    /**
     * Applies slew rate limiting to the input value.
     *
     * @param input The desired value
     * @param dt Time since last update
     * @return The rate-limited value
     */
    fun calculate(input: Double, dt: Duration): Double {
        val dtSeconds = dt.toDouble(DurationUnit.SECONDS)
        val maxChange = maxRate * dtSeconds

        val prev = previousValue
        val output = if (prev == null) {
            input
        } else {
            val delta = input - prev
            if (delta.absoluteValue <= maxChange) {
                input
            } else {
                prev + maxChange * delta.sign
            }
        }

        previousValue = output
        return output
    }

    /**
     * Resets the limiter state, allowing the next value to pass through unchanged.
     */
    fun reset() {
        previousValue = null
    }
}
