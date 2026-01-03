package sigmacorns.tuning.model

/**
 * Represents a single shot trial at a specific distance.
 */
data class ShotTrial(
    val timestamp: Long,           // epoch millis
    val distance: Double,          // meters from AprilTag
    val flywheelSpeed: Double,     // rad/s
    val feedbackType: FeedbackType,
    val feedbackIntensity: Int     // 0 = good, 1 = slight, 2 = big
)

enum class FeedbackType {
    UNDERSHOOT,
    GOOD,
    OVERSHOOT
}
