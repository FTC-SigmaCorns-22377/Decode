package sigmacorns.vision

import org.opencv.core.Rect

data class DetectedBall(
    val bbox: Rect,
    val color: BallColor,
    val area: Double,
)
