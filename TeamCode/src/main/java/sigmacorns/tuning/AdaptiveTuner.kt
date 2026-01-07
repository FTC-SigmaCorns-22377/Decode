package sigmacorns.tuning

import sigmacorns.tuning.model.FeedbackType
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Smart adaptive tuner that estimates initial flywheel speed from physics
 * and refines based on user feedback using binary-search-like adjustment.
 */
class AdaptiveTuner(
    private val dataStore: ShotDataStore
) {
    companion object {
        // Physics constants
        const val GRAVITY = 9.81                    // m/s^2
        const val LAUNCH_ANGLE_RAD = 0.5236         // 30 degrees in radians
        const val FLYWHEEL_RADIUS = 0.05            // meters (wheel radius)
        const val CORRECTION_FACTOR = 1.3           // empirical adjustment

        // Tuning parameters
        const val INITIAL_STEP_SIZE = 50.0          // rad/s
        const val MIN_STEP_SIZE = 5.0               // rad/s
        const val MAX_FLYWHEEL_SPEED = 628.0        // rad/s (~6000 RPM)
        const val MIN_FLYWHEEL_SPEED = 100.0        // rad/s
    }

    /**
     * Per-bucket tuning state for the adaptive algorithm.
     */
    private data class TuningState(
        var lowerBound: Double = MIN_FLYWHEEL_SPEED,
        var upperBound: Double = MAX_FLYWHEEL_SPEED,
        var currentGuess: Double = 0.0,
        var stepSize: Double = INITIAL_STEP_SIZE,
        var lowerBoundSet: Boolean = false,
        var upperBoundSet: Boolean = false
    )

    private val tuningStates = mutableMapOf<Double, TuningState>()

    /**
     * Get the recommended speed for testing at this distance.
     * Uses lookup table if available, otherwise physics estimate or current tuning state.
     */
    fun getRecommendedSpeed(distance: Double): Double {
        val bucketKey = dataStore.distanceToBucket(distance)

        // First, check if we have a confirmed speed from lookup table
        dataStore.getOptimalSpeed(distance)?.let { return it }

        // Get or create tuning state for this bucket
        val state = tuningStates.getOrPut(bucketKey) {
            TuningState(currentGuess = calculateInitialEstimate(distance))
        }

        // If both bounds are set, use bisection
        if (state.lowerBoundSet && state.upperBoundSet) {
            state.currentGuess = (state.lowerBound + state.upperBound) / 2
        }

        return state.currentGuess.coerceIn(MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED)
    }

    /**
     * Calculate physics-based initial estimate.
     * Based on projectile motion: v = sqrt(distance * g / sin(2 * angle))
     * Then convert linear velocity to flywheel angular velocity.
     */
    fun calculateInitialEstimate(distance: Double): Double {
        // Required launch velocity for projectile to travel 'distance' horizontally
        // Assuming flat trajectory start/end: range = v^2 * sin(2*theta) / g
        // Solving for v: v = sqrt(range * g / sin(2*theta))
        val sin2Theta = sin(2 * LAUNCH_ANGLE_RAD)
        if (sin2Theta <= 0.01) return MIN_FLYWHEEL_SPEED

        val linearVelocity = sqrt(distance * GRAVITY / sin2Theta)

        // Convert linear velocity to flywheel angular velocity
        // Linear velocity at ball contact = omega * radius
        val angularVelocity = (linearVelocity / FLYWHEEL_RADIUS) * CORRECTION_FACTOR

        return angularVelocity.coerceIn(MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED)
    }

    /**
     * Process user feedback and update the tuning state.
     */
    fun processFeedback(distance: Double, feedbackType: FeedbackType, intensity: Int) {
        val bucketKey = dataStore.distanceToBucket(distance)
        val state = tuningStates.getOrPut(bucketKey) {
            TuningState(currentGuess = calculateInitialEstimate(distance))
        }

        when (feedbackType) {
            FeedbackType.UNDERSHOOT -> {
                // Speed too low, need to increase
                state.lowerBound = maxOf(state.lowerBound, state.currentGuess)
                state.lowerBoundSet = true

                val multiplier = if (intensity >= 2) 2.0 else 1.0
                state.currentGuess += state.stepSize * multiplier

                // Shrink step size on slight feedback
                if (intensity == 1) {
                    state.stepSize = maxOf(state.stepSize * 0.7, MIN_STEP_SIZE)
                }
            }

            FeedbackType.OVERSHOOT -> {
                // Speed too high, need to decrease
                state.upperBound = minOf(state.upperBound, state.currentGuess)
                state.upperBoundSet = true

                val multiplier = if (intensity >= 2) 2.0 else 1.0
                state.currentGuess -= state.stepSize * multiplier

                // Shrink step size on slight feedback
                if (intensity == 1) {
                    state.stepSize = maxOf(state.stepSize * 0.7, MIN_STEP_SIZE)
                }
            }

            FeedbackType.GOOD -> {
                // Found optimal speed, reset state for this bucket
                resetBucket(distance)
            }
        }

        // Clamp current guess
        state.currentGuess = state.currentGuess.coerceIn(MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED)

        // If both bounds set, use bisection
        if (state.lowerBoundSet && state.upperBoundSet) {
            state.currentGuess = (state.lowerBound + state.upperBound) / 2
        }
    }

    /**
     * Check if we've converged for this distance (step size very small).
     */
    fun isConverged(distance: Double): Boolean {
        val bucketKey = dataStore.distanceToBucket(distance)
        val state = tuningStates[bucketKey] ?: return false

        if (state.lowerBoundSet && state.upperBoundSet) {
            return (state.upperBound - state.lowerBound) < MIN_STEP_SIZE * 2
        }

        return state.stepSize <= MIN_STEP_SIZE
    }

    /**
     * Reset tuning state for a distance bucket.
     */
    fun resetBucket(distance: Double) {
        val bucketKey = dataStore.distanceToBucket(distance)
        tuningStates.remove(bucketKey)
    }

    /**
     * Get current tuning state info for display.
     */
    fun getTuningStateInfo(distance: Double): String {
        val bucketKey = dataStore.distanceToBucket(distance)
        val state = tuningStates[bucketKey]

        return if (state != null) {
            val bounds = when {
                state.lowerBoundSet && state.upperBoundSet ->
                    "bounds: [${state.lowerBound.toInt()}, ${state.upperBound.toInt()}]"
                state.lowerBoundSet -> "lower: ${state.lowerBound.toInt()}"
                state.upperBoundSet -> "upper: ${state.upperBound.toInt()}"
                else -> "no bounds"
            }
            "step: ${state.stepSize.toInt()}, $bounds"
        } else {
            "initial estimate"
        }
    }
}
