package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.io.HardwareIO
import sigmacorns.logic.BallChaseAutoFSM
import sigmacorns.logic.BallTrackingSystem
import sigmacorns.math.Pose2d
import sigmacorns.vision.BallDetectionProcessor
import sigmacorns.vision.tracker.TrackerConfig
import kotlin.time.DurationUnit

/**
 * Ball-chase opmode. Vision + tracking + chase all run on the Control Hub
 * off the Global Shutter USB webcam.
 *
 * Hardware:
 *   - Global Shutter webcam registered in the config as "Webcam 1".
 *
 * Pipeline:
 *   - `BallDetectionProcessor` (Kotlin port of SimpleBallTracker.detect)
 *     runs HSV + YCrCb color masks, extracts contour bboxes, publishes
 *     `processor.detectedBalls`. Live annotated stream is visible in the
 *     FTC Dashboard.
 *   - `HardwareIO.ballDetectionProvider` is set to the processor; the
 *     tracker's `getBallDetections` call returns bbox-centers as
 *     `PixelDetection(u, v, t)`.
 *   - `BallTrackingSystem` projects each pixel through the camera
 *     intrinsics + extrinsics (from config/ball_tracker.json) to a
 *     field-frame point with 2x2 covariance, runs the Kalman tracker,
 *     and selects a target.
 *   - `BallChaseAutoFSM` drives the chase / drive-to-shoot / shoot cycle
 *     using the robot's existing Drivetrain.
 *
 * Controls:
 *   - Right bumper HELD = auto-cycle ON. Release = manual drive.
 *   - Gamepad1 left stick + right stick x = manual drive when auto is off.
 *
 * Calibration checklist before trusting this opmode:
 *   - Intrinsics (fx, fy, cx, cy, distortion) in config/ball_tracker.json
 *     match the Global Shutter webcam's actual resolution and calibration.
 *   - CAM_POS_R + CAM_*_RAD match the physical mount. Verify with a ball
 *     at a measured field position — projected (x, y) should match tape
 *     within 3 cm at 1-2 m range.
 */
@TeleOp(name = "Ball Chase TeleOp", group = "Chase")
class BallChaseTeleOp : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = false)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = false)

        // --- Camera pipeline ---
        val processor = BallDetectionProcessor()
        val portal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(processor)
            .enableLiveView(true)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        val hardware = io as? HardwareIO
        hardware?.ballDetectionProvider = {
            processor.detectedBalls to processor.lastCaptureTimeSec
        }

        // --- Tracker + chase ---
        val config = TrackerConfig.loadDefault()
        val tracking = BallTrackingSystem(config = config, io = io)

        // Shooting zone: 1.0 m out from the red goal, 0.6 m to the side,
        // heading pointed at the goal.
        val goal = FieldLandmarks.redGoalPosition
        val zoneX = goal.x - 1.0
        val zoneY = goal.y - 0.6
        val shootingZone = Pose2d(zoneX, zoneY, kotlin.math.atan2(goal.y - zoneY, goal.x - zoneX))

        val fsm = BallChaseAutoFSM(
            tracking = tracking,
            drivetrain = robot.drive,
            io = io,
            shootingZone = shootingZone,
            maxSpeed = 1.0,
            arrivalRadiusM = 0.18,
            chasePullInRadiusM = 0.35,
            flywheelShootSpeed = 1.0,
        )

        telemetry.addLine("Ball Chase ready. Hold RB for auto-cycle; release for manual.")
        telemetry.addLine("Camera pipeline: BallDetectionProcessor (HSV + YCrCb color masks).")
        telemetry.update()

        waitForStart()

        try {
            ioLoop { state, _ ->
                val t = state.timestamp.toDouble(DurationUnit.SECONDS)

                tracking.update(t)

                val wantAuto = gamepad1.right_bumper
                fsm.enabled = wantAuto
                if (wantAuto) {
                    fsm.update()
                } else {
                    robot.drive.update(gamepad1, io)
                }

                telemetry.addData("mode", if (wantAuto) fsm.phase else "MANUAL")
                telemetry.addData("detections", processor.detectedBalls.size)
                telemetry.addData("sequence", processor.ballColorString)
                telemetry.addData("tracks", tracking.tracker.tracks.size)
                val tgt = tracking.target
                if (tgt != null) {
                    val p = tgt.positionAt(t)
                    val pose = io.position()
                    val d = kotlin.math.hypot(p.x - pose.v.x, p.y - pose.v.y)
                    telemetry.addData("target", "#${tgt.id} (hits=${tgt.hits})  d=%.2fm".format(d))
                } else {
                    telemetry.addLine("no target")
                }
                telemetry.addData("held", BallChaseAutoFSM.countHeldFromBeamBreaks(io))
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
