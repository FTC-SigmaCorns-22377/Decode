package sigmacorns.control.vision

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import sigmacorns.sim.Balls

data class DetectedBall(
    val bbox: Rect,
    val color: Balls,
    val area: Double
)

/**
 * VisionProcessor for detecting green and purple balls using
 * dual color-space thresholding (HSV + YCrCb).
 *
 * Designed for global shutter cameras — no rolling shutter
 * compensation needed.
 */
class BallDetectionProcessor : VisionProcessor {

    // Green thresholds (HSV AND YCrCb intersection for robustness)
    private val greenHsvLower = Scalar(65.0, 100.0, 40.0)
    private val greenHsvUpper = Scalar(90.0, 255.0, 220.0)
    private val greenYcrcbLower = Scalar(50.0, 35.0, 60.0)
    private val greenYcrcbUpper = Scalar(160.0, 100.0, 145.0)

    // Purple thresholds (YCrCb only)
    private val purpleYcrcbLower = Scalar(80.0, 135.0, 130.0)
    private val purpleYcrcbUpper = Scalar(170.0, 175.0, 180.0)

    private val minBallArea = 300.0
    private val maxBallArea = 20000.0

    // Reusable Mats to avoid per-frame allocation
    private val hsv = Mat()
    private val ycrcb = Mat()
    private val greenHsvMask = Mat()
    private val greenYcrcbMask = Mat()
    private val greenMask = Mat()
    private val purpleMask = Mat()

    @Volatile
    var detectedBalls: List<DetectedBall> = emptyList()
        private set

    /**
     * Returns the color sequence of detected balls, left-to-right.
     * e.g. "GPG" = Green, Purple, Green
     */
    val ballColorString: String
        get() = detectedBalls.joinToString("") {
            if (it.color == Balls.Green) "G" else "P"
        }

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        // no-op — Mats are lazily sized by OpenCV on first use
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV)
        Imgproc.cvtColor(frame, ycrcb, Imgproc.COLOR_RGB2YCrCb)

        // Green: intersection of HSV and YCrCb masks
        Core.inRange(hsv, greenHsvLower, greenHsvUpper, greenHsvMask)
        Core.inRange(ycrcb, greenYcrcbLower, greenYcrcbUpper, greenYcrcbMask)
        Core.bitwise_and(greenHsvMask, greenYcrcbMask, greenMask)

        // Purple: YCrCb only
        Core.inRange(ycrcb, purpleYcrcbLower, purpleYcrcbUpper, purpleMask)

        val balls = mutableListOf<DetectedBall>()
        extractBalls(greenMask, Balls.Green, balls)
        extractBalls(purpleMask, Balls.Purple, balls)

        // Sort left-to-right by x position
        balls.sortBy { it.bbox.x }
        detectedBalls = balls

        // Draw bounding boxes on the frame for FTC dashboard / driver station preview
        for (ball in balls) {
            val color = if (ball.color == Balls.Green) {
                Scalar(0.0, 255.0, 0.0)
            } else {
                Scalar(255.0, 0.0, 255.0)
            }
            Imgproc.rectangle(frame, ball.bbox, color, 2)
        }

        return balls
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        // Drawing is handled in processFrame via OpenCV on the Mat directly.
        // Android Canvas overlay is not needed.
    }

    private fun extractBalls(
        mask: Mat,
        color: Balls,
        out: MutableList<DetectedBall>
    ) {
        val contours = mutableListOf<MatOfPoint>()
        val hierarchy = Mat()
        Imgproc.findContours(
            mask, contours, hierarchy,
            Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE
        )
        hierarchy.release()

        for (contour in contours) {
            val area = Imgproc.contourArea(contour)
            if (area in minBallArea..maxBallArea) {
                out.add(DetectedBall(
                    bbox = Imgproc.boundingRect(contour),
                    color = color,
                    area = area
                ))
            }
            contour.release()
        }
    }

    fun release() {
        hsv.release()
        ycrcb.release()
        greenHsvMask.release()
        greenYcrcbMask.release()
        greenMask.release()
        purpleMask.release()
    }
}
