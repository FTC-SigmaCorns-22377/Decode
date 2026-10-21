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

    // BLUE / RED / YELLOW (HSV)
    private val blueHsvLower = Scalar(100.0, 80.0, 60.0)
    private val blueHsvUpper = Scalar(130.0, 255.0, 255.0)

    private val redHsvLower1 = Scalar(0.0, 80.0, 60.0)
    private val redHsvUpper1 = Scalar(12.0, 255.0, 255.0)
    private val redHsvLower2 = Scalar(170.0, 80.0, 60.0)
    private val redHsvUpper2 = Scalar(180.0, 255.0, 255.0)

    var yellowHsvLower = Scalar(18.0, 80.0, 60.0)
    var yellowHsvUpper = Scalar(38.0, 255.0, 255.0)

    // Detection area bounds
    // 3.6 in red/blue balls and 2.8 in yellow balls; yellow is roughly 60% of the area.
    private val minBallAreaBlueRed = 300.0
    private val minBallAreaYellow = 180.0
    var maxBallArea = 20000.0

    // Shape filters (from yellow_ball.py). 0 = off. Rejects non-round blobs like walls or tables.
    var minCircularity = 0.0
    var minSolidity = 0.0
    var maxAspectRatio = Double.MAX_VALUE

    // ROI multipliers (of tag size) - ramp is LEFT and BELOW the tag
    var leftMult = 4.0
    var rightMult = 1.0
    var upMult = 0.5
    var downMult = 3.0

    // When true and no tag corners have been fed, search the whole frame instead of returning nothing
    var useFullFrame = false

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

    /** Camera frame size in pixels, set when the camera starts. */
    var frameWidth: Int = 0
        private set
    var frameHeight: Int = 0
        private set

    // Tag corners fed externally (e.g. from Limelight or AprilTagProcessor)
    // Array of 4 Points: TL, TR, BR, BL
    @Volatile
    private var tagCorners: Array<Point>? = null

    private var lastRoi: MatOfPoint? = null

    // Reusable Mats to avoid per-frame allocation
    private val hsv = Mat()
    private val blueMask = Mat()
    private val redMask1 = Mat()
    private val redMask2 = Mat()
    private val redMask = Mat()
    private val yellowMask = Mat()
    private val roiMask = Mat()
    private val hierarchy = Mat()
    private val filledMask = Mat()
    private val blobMask = Mat()
    private val dist = Mat()
    private val openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(3.0, 3.0))
    private val closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(7.0, 7.0))

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
        frameWidth = width
        frameHeight = height
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        val corners = tagCorners
        val roi: MatOfPoint

        if (corners != null && corners.size == 4) {
            roi = computeRoi(corners, frame.size())
            lastRoi = roi
        } else if (lastRoi != null) {
            roi = lastRoi!!
        } else if (useFullFrame) {
            val w = frame.cols() - 1.0
            val h = frame.rows() - 1.0
            roi = MatOfPoint(Point(0.0, 0.0), Point(w, 0.0), Point(w, h), Point(0.0, h))
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

        Core.inRange(hsv, blueHsvLower, blueHsvUpper, blueMask)
        Core.inRange(hsv, redHsvLower1, redHsvUpper1, redMask1)
        Core.inRange(hsv, redHsvLower2, redHsvUpper2, redMask2)
        Core.bitwise_or(redMask1, redMask2, redMask)
        Core.inRange(hsv, yellowHsvLower, yellowHsvUpper, yellowMask)

        Core.bitwise_and(blueMask, roiMask, blueMask)
        Core.bitwise_and(redMask, roiMask, redMask)
        Core.bitwise_and(yellowMask, roiMask, yellowMask)

        // Find contours and build detections
        val result = mutableListOf<BallDetection>()
        val candidates = listOf(
            blueMask to ("B" to minBallAreaBlueRed),
            redMask to ("R" to minBallAreaBlueRed),
            yellowMask to ("Y" to minBallAreaYellow)
        )

        val shapeFilter = minCircularity > 0.0 || minSolidity > 0.0 || maxAspectRatio < Double.MAX_VALUE

        for ((mask, pair) in candidates) {
            if (shapeFilter) {
                // Clean speckle and fill highlights so round balls stay one solid contour
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, openKernel)
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel, Point(-1.0, -1.0), 2)
            }
            val contours = mutableListOf<MatOfPoint>()
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
            val (color, minArea) = pair
            if (shapeFilter) {
                // Distance to the nearest background pixel. Its peak inside a blob is the center and
                // radius of the biggest ball in it, so touching balls don't look like one big ball.
                // Blobs are filled solid first so wiffle-ball holes don't shrink the radius.
                filledMask.create(mask.size(), CvType.CV_8UC1)
                filledMask.setTo(Scalar(0.0))
                Imgproc.drawContours(filledMask, contours, -1, Scalar(255.0), -1)
                Imgproc.distanceTransform(filledMask, dist, Imgproc.DIST_L2, 5)
                blobMask.create(mask.size(), CvType.CV_8UC1)
            }
            for (cnt in contours) {
                val area = Imgproc.contourArea(cnt)
                if (area >= minArea && area <= maxBallArea && (!shapeFilter || isRound(cnt, area))) {
                    val rect = Imgproc.boundingRect(cnt)
                    val cx = rect.x + rect.width / 2
                    val cy = rect.y + rect.height / 2
                    if (shapeFilter) {
                        blobMask.setTo(Scalar(0.0))
                        Imgproc.drawContours(blobMask, listOf(cnt), -1, Scalar(255.0), -1)
                        val peak = Core.minMaxLoc(dist, blobMask)
                        result.add(BallDetection(cx, cy, area, color, peak.maxVal, peak.maxLoc.x.toInt(), peak.maxLoc.y.toInt()))
                    } else {
                        result.add(BallDetection(cx, cy, area, color))
                    }
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
        var bCount = 0; var rCount = 0; var yCount = 0
        for (det in result) {
            val drawColor: Scalar
            val label: String
            when (det.color) {
                "B" -> { bCount++; drawColor = Scalar(255.0, 0.0, 0.0); label = "B$bCount" }
                "R" -> { rCount++; drawColor = Scalar(0.0, 0.0, 255.0); label = "R$rCount" }
                else -> { yCount++; drawColor = Scalar(0.0, 255.0, 255.0); label = "Y$yCount" }
            }
            val drawRadius = if (det.radius > 0) det.radius.toInt() else 10
            Imgproc.circle(frame, Point(det.peakX.toDouble(), det.peakY.toDouble()), drawRadius, drawColor, 2)
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

    private fun isRound(cnt: MatOfPoint, area: Double): Boolean {
        val rect = Imgproc.boundingRect(cnt)
        if (rect.width == 0 || rect.height == 0) return false
        val aspect = maxOf(rect.width, rect.height).toDouble() / minOf(rect.width, rect.height)

        val pts = cnt.toArray()
        val cnt2f = MatOfPoint2f(*pts)
        val perimeter = Imgproc.arcLength(cnt2f, true)
        cnt2f.release()
        val circularity = if (perimeter > 0) 4 * Math.PI * area / (perimeter * perimeter) else 0.0

        val hullIdx = MatOfInt()
        Imgproc.convexHull(cnt, hullIdx)
        val hull = MatOfPoint(*hullIdx.toArray().map { pts[it] }.toTypedArray())
        val hullArea = Imgproc.contourArea(hull)
        hull.release()
        hullIdx.release()
        val solidity = if (hullArea > 0) area / hullArea else 0.0

        return circularity >= minCircularity && solidity >= minSolidity && aspect <= maxAspectRatio
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
    val color: String,  // "G" or "P"
    // Biggest single ball inside the blob (only filled in when shape filters are on)
    val radius: Double = 0.0,
    val peakX: Int = centerX,
    val peakY: Int = centerY
)
