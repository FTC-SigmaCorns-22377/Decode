package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.constants.Limelight
import sigmacorns.io.HardwareIO
import sigmacorns.logic.BallTrackingSystem
import sigmacorns.logic.ChaseCoordinator
import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.TrackerConfig
import kotlin.time.DurationUnit

/**
 * First-light ball-chase opmode.
 *
 * Behavior:
 *   - Initializes the Robot + Limelight (BALL_PIPELINE). If no Limelight is
 *     present (e.g. running against SimIO without a camera), the tracking
 *     pipeline returns empty detections and the robot stays in manual drive.
 *   - Holds right bumper = AUTO-CHASE enabled. Release = manual drive resumes
 *     immediately. This is the manual-override abort the tracker prompt asks
 *     for — the driver can always reclaim the robot by releasing the bumper.
 *   - Drivetrain during manual drive: same gamepad mapping as MainTeleOp
 *     (left stick translate, right stick x rotate).
 *
 * Telemetry: tracks seen, target id, distance to target.
 */
@TeleOp(name = "Ball Chase TeleOp", group = "Chase")
class BallChaseTeleOp : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = false)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = false)

        // Swap Limelight to the ball pipeline. We do this once here rather than
        // every tick so VisionTracker is not fighting us for pipeline ownership.
        (io as? HardwareIO)?.limelight?.pipelineSwitch(Limelight.BALL_PIPELINE)

        val config = TrackerConfig.loadDefault()
        val tracking = BallTrackingSystem(config = config, io = io)
        val chase = ChaseCoordinator(
            drivetrain = robot.drive,
            tracking = tracking,
            io = io,
            maxSpeed = 0.6,
            stopRadiusM = 0.25,
            slowDownRadiusM = 0.8,
            rotateToTarget = true,
        )
        chase.enabled = false

        telemetry.addLine("Ball Chase TeleOp ready. Hold RB to auto-chase; release to regain manual drive.")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            val t = state.timestamp.toDouble(DurationUnit.SECONDS)

            // 1. Run tracker every tick so the estimate stays warm even when
            //    auto-chase is disabled (lets the driver see what the tracker
            //    knows before committing).
            tracking.update(t)

            // 2. Chase is gated behind right bumper. Release = manual.
            val wantChase = gamepad1.right_bumper
            chase.enabled = wantChase
            if (wantChase) {
                chase.update()
            } else {
                robot.drive.update(gamepad1, io)
            }

            telemetry.addData("scenario", if (wantChase) "AUTO-CHASE" else "MANUAL")
            telemetry.addData("tracks", tracking.tracker.tracks.size)
            val tgt = tracking.target
            if (tgt != null) {
                val p = tgt.positionAt(t)
                val pose = io.position()
                val dx = p.x - pose.v.x; val dy = p.y - pose.v.y
                val d = kotlin.math.hypot(dx, dy)
                telemetry.addData("target", "#${tgt.id} (hits=${tgt.hits})")
                telemetry.addData("target pos", "(%.2f, %.2f)".format(p.x, p.y))
                telemetry.addData("distance", "%.2f m".format(d))
            } else {
                telemetry.addLine("no target")
            }
            telemetry.update()

            false
        }

        // Leave the Limelight pipeline back on AprilTags on exit so a later
        // opmode that needs localization isn't surprised.
        (io as? HardwareIO)?.limelight?.pipelineSwitch(Limelight.APRILTAG_PIPELINE)
    }
}
