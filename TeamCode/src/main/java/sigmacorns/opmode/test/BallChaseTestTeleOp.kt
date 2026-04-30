package sigmacorns.opmode.test

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import sigmacorns.io.HardwareIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.Drivetrain
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.vision.BallDetectionProcessor
import kotlin.math.abs
import kotlin.time.DurationUnit

/**
 * Dumb pixel-based ball chase. No tracker, no pose, no field frame.
 *
 *   RIGHT BUMPER held  ->  AUTO CHASE: steer+drive toward the largest detected ball in the image.
 *   release            ->  Manual drive (robot-centric).
 *
 * Steering law:
 *   yaw   = headingK * (normalizedPixelX)        where normalized x = (bbox.cx - imgCx) / imgCx, in [-1, 1]
 *   drive = baseSpeed * (1 - |normalizedPixelX|) if ball is in view, else 0
 *
 * This is intentionally simple. If this doesn't move the wheels, the problem is
 * either (a) motor wiring, (b) no detections, or (c) drivetrain config. Telemetry
 * below surfaces which one.
 */
@TeleOp(name = "Ball Chase TEST", group = "Test")
class BallChaseTestTeleOp : SigmaOpMode() {

    override fun runOpMode() {
        val processor = BallDetectionProcessor()
        // LiveView off: the camera-feed render on the RC eats CPU on every frame
        // (independent of detection quality) and was starving the WiFi service,
        // causing the DS connectivity indicator to flicker green→yellow. Detection
        // pipeline is unchanged. Use the DS camera-stream view if you need to peek.
        val portal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(processor)
            .enableLiveView(false)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .setCameraResolution(Size(1280, 720))
            .build()

        val hardware = io as? HardwareIO
        hardware?.ballDetectionProvider = {
            processor.detectedBalls to processor.lastCaptureTimeSec
        }

        val drive = Drivetrain().apply { fieldCentric = false }

        val imageWidth = 1280
        val imgCx = imageWidth / 2.0

        // Camera is rotated 180 in processFrame. In the rotated (right-side-up)
        // image, a ball to the robot's LEFT appears at LARGER pixel x, because
        // ROTATE_180 flips x. We account for that: rotatedX = width - rawX.
        // But the processor's bbox is in the ROTATED frame already (rotate is
        // in-place before detection), so bbox.x is already in the right-side-up
        // frame where LEFT-of-robot = SMALLER x. Standard convention: positive
        // yaw = CCW = robot turns LEFT. Ball on the left (small x) -> yaw +.
        // err = (imgCx - bboxCx) / imgCx -> positive when ball is on the left.

        // Grab-and-go control law:
        //   yaw is CAPPED so the wheel-power normalization in Drivetrain (which divides
        //   by max(|fwd ± yaw|)) cannot eat into forward speed. With baseSpeed + maxYaw
        //   ≤ 1.0, no normalization ever triggers and fwd is preserved verbatim.
        //   yaw is also SLEW-RATE-LIMITED so per-frame bbox jitter doesn't snap the
        //   wheels left/right every loop.
        val baseSpeed = 1.0
        val maxYaw = 0.30           // upper bound on yaw command
        val headingK = 0.6          // gain into the cap; saturates near |err| ≈ 0.5, gentler in the middle
        val yawSlewPerSec = 1.5     // max change in yaw command per second (smooths bbox noise)
        val visionGraceSec = 1.5    // hold last detection across vision dropouts
        val lostFwd = 0.35          // when ball is truly lost: keep crawling forward, don't stop or spin
        val telemetryPeriodSec = 0.2

        telemetry.addLine("Ball Chase TEST (dumb pixel chase) — hold RB")
        telemetry.addData("pinpoint", if (hardware?.pinpoint != null) "FOUND" else "MISSING")
        telemetry.update()

        waitForStart()

        // Hold last good detection across short vision dropouts so a single
        // empty frame (motion blur, occlusion, edge of FOV) doesn't snap the
        // robot from "drive forward" into "spin in place" and back.
        var heldErr: Double? = null
        var heldErrTimeSec = -10.0
        var lastTelemetryTimeSec = -10.0
        var yawCmd = 0.0  // slew-rate-limited yaw command

        try {
            ioLoop { state, dt ->
                val tSec = state.timestamp.toDouble(DurationUnit.SECONDS)
                val dtSec = dt.toDouble(DurationUnit.SECONDS).coerceIn(1e-3, 0.1)
                val chaseOn = gamepad1.right_bumper || gamepad2.right_bumper

                val balls = processor.detectedBalls
                val biggest = balls.maxByOrNull { it.bbox.width * it.bbox.height }
                val bboxCx = biggest?.let { it.bbox.x + it.bbox.width / 2.0 }
                val rawErr = bboxCx?.let { (imgCx - it) / imgCx } // +1 full left, -1 full right

                if (rawErr != null) {
                    heldErr = rawErr
                    heldErrTimeSec = tSec
                }
                val effErr = if (rawErr != null) rawErr
                    else if (heldErr != null && tSec - heldErrTimeSec < visionGraceSec) heldErr
                    else null

                val activeGp = if (
                    kotlin.math.hypot(gamepad2.left_stick_x.toDouble(), gamepad2.left_stick_y.toDouble()) >
                    kotlin.math.hypot(gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble())
                ) gamepad2 else gamepad1

                if (chaseOn && effErr != null) {
                    val yawTarget = (headingK * effErr).coerceIn(-maxYaw, maxYaw)
                    val maxStep = yawSlewPerSec * dtSec
                    yawCmd += (yawTarget - yawCmd).coerceIn(-maxStep, maxStep)
                    // Cap fwd so |fwd| + |yaw| ≤ baseSpeed → no Drivetrain normalization.
                    val fwd = baseSpeed - abs(yawCmd)
                    drive.drive(Pose2d(fwd, 0.0, yawCmd), io)
                    io.intake = 1.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                } else if (chaseOn) {
                    // Chase on, ball lost beyond grace -> commit forward at low speed.
                    // No spin-in-place search (that's what made it feel like stopping).
                    val maxStep = yawSlewPerSec * dtSec
                    yawCmd += (0.0 - yawCmd).coerceIn(-maxStep, maxStep)
                    drive.drive(Pose2d(lostFwd, 0.0, yawCmd), io)
                    io.intake = 1.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                } else {
                    yawCmd = 0.0
                    drive.update(activeGp, io)
                    io.intake = 0.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                }
                io.flywheel = 0.0

                // Throttle telemetry — telemetry.update() round-trips to the DS each call
                // and at full loop rate dominates loop time + DS bandwidth.
                if (tSec - lastTelemetryTimeSec > telemetryPeriodSec) {
                    lastTelemetryTimeSec = tSec
                    telemetry.addData("auto-chase", if (chaseOn) "ON" else "OFF")
                    telemetry.addData("pixel dets", balls.size)
                    if (biggest != null) {
                        telemetry.addData("ball bbox cx", "%.0f px (img cx=%.0f)".format(bboxCx, imgCx))
                        telemetry.addData("err raw", "%+.2f".format(rawErr))
                    } else if (effErr != null) {
                        telemetry.addData("err held", "%+.2f (age %.2fs)".format(effErr, tSec - heldErrTimeSec))
                    } else {
                        telemetry.addLine("no ball in frame")
                    }
                    telemetry.addData("yawCmd", "%+.2f".format(yawCmd))
                    telemetry.addData("wheels FL/BL/BR/FR", "%+.2f %+.2f %+.2f %+.2f".format(
                        io.driveFL, io.driveBL, io.driveBR, io.driveFR))
                    telemetry.update()
                }

                // Yield CPU to system services (WiFi, vision thread). Without this,
                // a tight ioLoop can starve the network stack and the DS connectivity
                // indicator flickers green→yellow.
                Thread.yield()

                false
            }
        } finally {
            hardware?.ballDetectionProvider = null
            portal.close()
            processor.release()
        }
    }
}
