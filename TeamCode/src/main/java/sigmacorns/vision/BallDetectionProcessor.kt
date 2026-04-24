package sigmacorns.vision

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

/**
 * Detects green and purple balls via color-space thresholding.
 *
 * Green: intersection of HSV and YCrCb masks (more robust against lighting).
 * Purple: YCrCb mask only.
 *
 * Port of the team's Python SimpleBallTracker — kept intentionally stateless
 * per-frame. Persistent tracking across frames lives in a separate tracker
 * (to be added as the pipeline grows).
 */
class BallDetectionProcessor : VisionProcessor {

    private val greenHsvLower = Scalar(65.0, 100.0, 40.0)
    private val greenHsvUpper = Scalar(90.0, 255.0, 220.0)
    private val greenYcrcbLower = Scalar(50.0, 35.0, 60.0)
    private val greenYcrcbUpper = Scalar(160.0, 100.0, 145.0)

    private val purpleYcrcbLower = Scalar(80.0, 135.0, 130.0)
    private val purpleYcrcbUpper = Scalar(170.0, 175.0, 180.0)

    private val minBallArea = 300.0
    private val maxBallArea = 20000.0

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
     * Capture timestamp (seconds, monotonic since Control Hub boot) of the
     * frame that produced [detectedBalls]. The tracker uses this to pose-
     * interpolate correctly via PoseBuffer, rather than stamping with loop
     * time (which lags frame arrival by ~10-30 ms).
     */
    @Volatile
    var lastCaptureTimeSec: Double = 0.0
        private set

    /** Left-to-right color sequence, e.g. "GPG". */
    val ballColorString: String
        get() = detectedBalls.joinToString("") { it.color.code.toString() }

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(frame: Mat, captureTimeNanos: Long): MutableList<DetectedBall>? {
        if(frame.empty()) {
            detectedBalls = emptyList()
            return mutableListOf()
        }
        lastCaptureTimeSec = captureTimeNanos / 1e9
        // Camera is physically mounted upside-down. Rotate 180° in-place so the
        // rest of the pipeline (detection, bbox coords, projection with cx/cy
        // at image center) sees a right-side-up frame.
        Core.rotate(frame, frame, Core.ROTATE_180)
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV)
        Imgproc.cvtColor(frame, ycrcb, Imgproc.COLOR_RGB2YCrCb)

        Core.inRange(hsv, greenHsvLower, greenHsvUpper, greenHsvMask)
        Core.inRange(ycrcb, greenYcrcbLower, greenYcrcbUpper, greenYcrcbMask)
        Core.bitwise_and(greenHsvMask, greenYcrcbMask, greenMask)

        Core.inRange(ycrcb, purpleYcrcbLower, purpleYcrcbUpper, purpleMask)

        val balls = mutableListOf<DetectedBall>()
        extractBalls(greenMask, BallColor.GREEN, balls)
        extractBalls(purpleMask, BallColor.PURPLE, balls)

        balls.sortBy { it.bbox.x }
        detectedBalls = balls

        for (ball in balls) {
            val color = if (ball.color == BallColor.GREEN) {
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
        userContext: Any?,
    ) {}

    private fun extractBalls(
        mask: Mat,
        color: BallColor,
        out: MutableList<DetectedBall>,
    ) {
        val contours = mutableListOf<MatOfPoint>()
        val hierarchy = Mat()
        Imgproc.findContours(
            mask, contours, hierarchy,
            Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE,
        )
        hierarchy.release()

        for (contour in contours) {
            val area = Imgproc.contourArea(contour)
            if (area in minBallArea..maxBallArea) {
                out.add(
                    DetectedBall(
                        bbox = Imgproc.boundingRect(contour),
                        color = color,
                        area = area,
                    )
                )
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
