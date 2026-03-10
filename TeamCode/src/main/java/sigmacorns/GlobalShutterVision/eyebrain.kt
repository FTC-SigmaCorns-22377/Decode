package sigmacorns.GlobalShutterVision

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.*
import org.opencv.imgproc.Imgproc

/**
 * AprilTag-anchored ROI ball detector for FTC Decode 2025-2026.
 * Detects GREEN and PURPLE balls in the blue goal ramp area.
 *
 * Port of Python OpenCV pipeline to FTC VisionProcessor.
 * AprilTag detection is handled separately (by Limelight or AprilTagProcessor).
 * Call [setTagCorners] each frame with the detected tag corners to anchor the ROI.
 */
class EyeBrain : VisionProcessor {

    // ========================================================================
    // COLOR THRESHOLDS (from vision.py)
    // ========================================================================

    // GREEN (HSV AND YCrCb intersection)
    private val greenHsvLower = Scalar(65.0, 100.0, 40.0)
    private val greenHsvUpper = Scalar(90.0, 255.0, 220.0)
    private val greenYCrCbLower = Scalar(50.0, 35.0, 60.0)
    private val greenYCrCbUpper = Scalar(160.0, 100.0, 145.0)

    // PURPLE (YCrCb only)
    private val purpleYCrCbLower = Scalar(80.0, 135.0, 130.0)
    private val purpleYCrCbUpper = Scalar(170.0, 175.0, 180.0)

    // Detection area bounds
    private val minBallArea = 300.0
    private val maxBallArea = 20000.0

    // ROI multipliers (of tag size) - ramp is LEFT and BELOW the tag
    var leftMult = 4.0
    var rightMult = 1.0
    var upMult = 0.5
    var downMult = 3.0

    // ========================================================================
    // STATE
    // ========================================================================

    /** Latest detection results, read from OpMode. */
    @Volatile
    var detections: List<BallDetection> = emptyList()
        private set

    /** Motif string e.g. "GGPG" */
    @Volatile
    var motif: String = ""
        private set

    // Tag corners fed externally (e.g. from Limelight or AprilTagProcessor)
    // Array of 4 Points: TL, TR, BR, BL
    @Volatile
    private var tagCorners: Array<Point>? = null

    private var lastRoi: MatOfPoint? = null

    // Reusable Mats to avoid per-frame allocation
    private val hsv = Mat()
    private val ycrcb = Mat()
    private val greenHsv = Mat()
    private val greenYCrCb = Mat()
    private val greenMask = Mat()
    private val purpleMask = Mat()
    private val roiMask = Mat()
    private val hierarchy = Mat()

    // ========================================================================
    // PUBLIC API
    // ========================================================================

    /**
     * Feed AprilTag corner data each frame.
     * @param corners 4 points in order: top-left, top-right, bottom-right, bottom-left
     *                (pixel coordinates in the camera frame). Pass null if tag not visible.
     */
    fun setTagCorners(corners: Array<Point>?) {
        tagCorners = corners
    }

    // ========================================================================
    // VisionProcessor interface
    // ========================================================================

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        // nothing needed
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        val corners = tagCorners
        val roi: MatOfPoint

        if (corners != null && corners.size == 4) {
            roi = computeRoi(corners, frame.size())
            lastRoi = roi
        } else if (lastRoi != null) {
            roi = lastRoi!!
        } else {
            detections = emptyList()
            motif = ""
            return null
        }

        // Build ROI mask
        if (roiMask.empty() || roiMask.rows() != frame.rows() || roiMask.cols() != frame.cols()) {
            roiMask.create(frame.size(), CvType.CV_8UC1)
        }
        roiMask.setTo(Scalar(0.0))
        Imgproc.fillPoly(roiMask, listOf(roi), Scalar(255.0))

        // Color conversions
        // Note: FTC VisionPortal delivers frames as RGB, not BGR
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV)
        Imgproc.cvtColor(frame, ycrcb, Imgproc.COLOR_RGB2YCrCb)

        // GREEN: HSV AND YCrCb
        Core.inRange(hsv, greenHsvLower, greenHsvUpper, greenHsv)
        Core.inRange(ycrcb, greenYCrCbLower, greenYCrCbUpper, greenYCrCb)
        Core.bitwise_and(greenHsv, greenYCrCb, greenMask)
        Core.bitwise_and(greenMask, roiMask, greenMask)

        // PURPLE: YCrCb only
        Core.inRange(ycrcb, purpleYCrCbLower, purpleYCrCbUpper, purpleMask)
        Core.bitwise_and(purpleMask, roiMask, purpleMask)

        // Find contours and build detections
        val result = mutableListOf<BallDetection>()

        for ((mask, color) in listOf(greenMask to "G", purpleMask to "P")) {
            val contours = mutableListOf<MatOfPoint>()
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
            for (cnt in contours) {
                val area = Imgproc.contourArea(cnt)
                if (area in minBallArea..maxBallArea) {
                    val rect = Imgproc.boundingRect(cnt)
                    val cx = rect.x + rect.width / 2
                    val cy = rect.y + rect.height / 2
                    result.add(BallDetection(cx, cy, area, color))
                }
                cnt.release()
            }
        }

        // Sort left-to-right
        result.sortBy { it.centerX }

        detections = result
        motif = result.joinToString("") { it.color }

        // Draw ROI and detections on frame for camera stream preview
        Imgproc.polylines(frame, listOf(roi), true, Scalar(255.0, 0.0, 0.0), 2)
        var gCount = 0; var pCount = 0
        for (det in result) {
            val drawColor: Scalar
            val label: String
            if (det.color == "G") {
                gCount++; drawColor = Scalar(0.0, 255.0, 0.0); label = "G$gCount"
            } else {
                pCount++; drawColor = Scalar(255.0, 0.0, 255.0); label = "P$pCount"
            }
            Imgproc.circle(frame, Point(det.centerX.toDouble(), det.centerY.toDouble()), 10, drawColor, 2)
            Imgproc.putText(frame, label, Point(det.centerX - 15.0, det.centerY - 20.0),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, drawColor, 2)
        }
        Imgproc.putText(frame, "Motif: $motif (${result.size})", Point(10.0, 30.0),
            Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255.0, 255.0, 0.0), 2)

        return detections
    }

    override fun onDrawFrame(canvas: Canvas, onscreenWidth: Int, onscreenHeight: Int,
                             scaleBmpPxToCanvasPx: Float, scaleCanvasDensity: Float, userContext: Any?) {
        // Drawing is done in processFrame via OpenCV on the Mat directly
    }

    // ========================================================================
    // ROI COMPUTATION
    // ========================================================================

    private fun computeRoi(tagCorners: Array<Point>, frameSize: Size): MatOfPoint {
        val (tl, tr, br, bl) = tagCorners

        // Direction vectors (averaged across tag edges)
        val rightVecX = ((tr.x - tl.x) + (br.x - bl.x)) / 2
        val rightVecY = ((tr.y - tl.y) + (br.y - bl.y)) / 2
        val downVecX = ((bl.x - tl.x) + (br.x - tr.x)) / 2
        val downVecY = ((bl.y - tl.y) + (br.y - tr.y)) / 2

        val rightLen = Math.sqrt(rightVecX * rightVecX + rightVecY * rightVecY)
        val downLen = Math.sqrt(downVecX * downVecX + downVecY * downVecY)

        val rdx = if (rightLen > 0) rightVecX / rightLen else 1.0
        val rdy = if (rightLen > 0) rightVecY / rightLen else 0.0
        val ddx = if (downLen > 0) downVecX / downLen else 0.0
        val ddy = if (downLen > 0) downVecY / downLen else 1.0

        val tagW = rightLen
        val tagH = downLen

        // Offset vectors
        val leftOx = rdx * tagW * leftMult;  val leftOy = rdy * tagW * leftMult
        val rightOx = rdx * tagW * rightMult; val rightOy = rdy * tagW * rightMult
        val upOx = ddx * tagH * upMult;   val upOy = ddy * tagH * upMult
        val downOx = ddx * tagH * downMult; val downOy = ddy * tagH * downMult

        val w = frameSize.width - 1
        val h = frameSize.height - 1

        fun clamp(p: Point) = Point(p.x.coerceIn(0.0, w), p.y.coerceIn(0.0, h))

        val roiTL = clamp(Point(tl.x - leftOx - upOx, tl.y - leftOy - upOy))
        val roiTR = clamp(Point(tr.x + rightOx - upOx, tr.y + rightOy - upOy))
        val roiBR = clamp(Point(br.x + rightOx + downOx, br.y + rightOy + downOy))
        val roiBL = clamp(Point(bl.x - leftOx + downOx, bl.y - leftOy + downOy))

        return MatOfPoint(roiTL, roiTR, roiBR, roiBL)
    }
}

// ============================================================================
// DATA CLASSES
// ============================================================================

data class BallDetection(
    val centerX: Int,
    val centerY: Int,
    val area: Double,
    val color: String  // "G" or "P"
)
