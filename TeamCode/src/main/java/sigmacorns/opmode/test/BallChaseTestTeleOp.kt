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
        val portal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(processor)
            .enableLiveView(true)
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

        val baseSpeed = 0.8
        val headingK = 0.9
        val centerDeadZone = 0.08   // |err| below this counts as centered
        val pullInForward = 0.5    // when ball is nearly centered, just drive forward at this power

        telemetry.addLine("Ball Chase TEST (dumb pixel chase) — hold RB")
        telemetry.addData("pinpoint", if (hardware?.pinpoint != null) "FOUND" else "MISSING")
        telemetry.update()

        waitForStart()

        try {
            ioLoop { _, _ ->
                val chaseOn = gamepad1.right_bumper || gamepad2.right_bumper

                val balls = processor.detectedBalls
                val biggest = balls.maxByOrNull { it.bbox.width * it.bbox.height }
                val bboxCx = biggest?.let { it.bbox.x + it.bbox.width / 2.0 }
                val err = bboxCx?.let { (imgCx - it) / imgCx } // +1 full left, -1 full right

                val activeGp = if (
                    kotlin.math.hypot(gamepad2.left_stick_x.toDouble(), gamepad2.left_stick_y.toDouble()) >
                    kotlin.math.hypot(gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble())
                ) gamepad2 else gamepad1

                if (chaseOn && err != null) {
                    val yaw = headingK * err
                    val fwd = if (abs(err) < centerDeadZone) baseSpeed else pullInForward * (1.0 - abs(err))
                    drive.drive(Pose2d(fwd, 0.0, yaw), io)
                    io.intake = 1.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                } else if (chaseOn) {
                    // Chase on but no ball -> gentle search rotation so camera sweeps.
                    drive.drive(Pose2d(0.0, 0.0, 0.6), io)
                    io.intake = 1.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                } else {
                    drive.update(activeGp, io)
                    io.intake = 0.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                }
                io.flywheel = 0.0

                telemetry.addData("auto-chase", if (chaseOn) "ON" else "OFF")
                telemetry.addData("pixel dets", balls.size)
                if (biggest != null) {
                    telemetry.addData("ball bbox cx", "%.0f px (img cx=%.0f)".format(bboxCx, imgCx))
                    telemetry.addData("err", "%+.2f  (+ = ball on LEFT, - = ball on RIGHT)".format(err))
                } else {
                    telemetry.addLine("no ball in frame")
                }
                telemetry.addData("wheels FL/BL/BR/FR", "%+.2f %+.2f %+.2f %+.2f".format(
                    io.driveFL, io.driveBL, io.driveBR, io.driveFR))
                telemetry.update()

                false
            }
        } finally {
            hardware?.ballDetectionProvider = null
            portal.close()
            processor.release()
        }
    }
}
