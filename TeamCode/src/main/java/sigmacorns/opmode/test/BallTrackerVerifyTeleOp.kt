package sigmacorns.opmode.test

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import sigmacorns.io.HardwareIO
import sigmacorns.logic.BallTrackingSystem
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.vision.BallDetectionProcessor
import sigmacorns.vision.tracker.TrackerConfig
import kotlin.time.DurationUnit

/**
 * Read-only ball-tracker verification opmode.
 *
 * Purpose: calibrate / sanity-check `CAM_POS_R`, the Euler angles, and the
 * intrinsics in `config/ball_tracker.json`. The robot is not driven. Place a
 * DECODE artifact at a tape-measured field location, start the opmode, and
 * compare the printed field-frame projection to the measured position.
 *
 * Telemetry per frame:
 *   - robot pose from odometry
 *   - number of pixel detections
 *   - for each track: id, hits, field (x, y), distance from robot, velocity
 *     (expected ~0 for stationary balls), position-covariance trace
 *
 * Procedure:
 *   1. Run the opmode with no balls visible — confirm zero tracks, no phantom
 *      detections. If phantoms appear, tune BallDetectionProcessor HSV/YCrCb.
 *   2. Place a single ball at a measured (x_m, y_m) from the robot's odometry
 *      origin. Hold still.
 *   3. Expect one confirmed track. Read its reported (x, y).
 *   4. If the reported position is consistently offset, adjust the
 *      extrinsics (`CAM_POS_R` / `CAM_PITCH_DOWN_RAD` / yaw / roll). An
 *      offset in the direction of the camera usually means the pitch is wrong;
 *      a lateral offset means the yaw (or lateral mount position) is wrong.
 *   5. Repeat at 1 m, 2 m, and on either side of the field — the projection
 *      Jacobian stretches range uncertainty with distance, so the calibration
 *      is most sensitive at the near-field and least sensitive at the far.
 *
 * Target accuracy: projected (x, y) within 3 cm of tape-measure at 1-2 m.
 * If you can't hit that with any reasonable extrinsics, the intrinsics
 * (`K` + `DIST_COEFFS` in `config/ball_tracker.json`) are probably wrong —
 * re-run OpenCV checkerboard calibration.
 */
@TeleOp(name = "Ball Tracker Verify", group = "Test")
class BallTrackerVerifyTeleOp : SigmaOpMode() {

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

        val config = TrackerConfig.loadDefault()
        val tracking = BallTrackingSystem(config = config, io = io)

        telemetry.addLine("Ball Tracker Verify ready. The bot does NOT drive.")
        telemetry.addLine("Place a ball at a measured (x, y) and press START.")
        telemetry.update()

        waitForStart()

        try {
            while (opModeIsActive()) {
                val t = io.time().toDouble(DurationUnit.SECONDS)
                tracking.update(t)

                val pose = io.position()
                telemetry.addData("robot pose", "x=%.3f y=%.3f θ=%.3f rad".format(pose.v.x, pose.v.y, pose.rot))
                telemetry.addData("pixel detections", processor.detectedBalls.size)
                telemetry.addData("sequence L→R", processor.ballColorString)
                telemetry.addData("tracks", tracking.tracker.tracks.size)

                for (tr in tracking.tracker.tracks.sortedBy { it.id }) {
                    val p = tr.positionAt(t)
                    val dx = p.x - pose.v.x; val dy = p.y - pose.v.y
                    val d = kotlin.math.hypot(dx, dy)
                    val covTrace = tr.positionCovTrace()
                    val confirmed = tracking.tracker.isConfirmed(tr)
                    telemetry.addData(
                        "track #${tr.id}${if (confirmed) "*" else ""}",
                        "field=(%.3f, %.3f) d=%.3fm v=(%+.2f,%+.2f) covTr=%.4f hits=%d".format(
                            p.x, p.y, d, tr.state[2], tr.state[3], covTrace, tr.hits,
                        ),
                    )
                }
                telemetry.update()

                io.update()
            }
        } finally {
            hardware?.ballDetectionProvider = null
            portal.close()
            processor.release()
        }
    }
}
