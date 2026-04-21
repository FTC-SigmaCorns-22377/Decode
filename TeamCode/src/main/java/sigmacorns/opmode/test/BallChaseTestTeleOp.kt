package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import sigmacorns.Robot
import sigmacorns.io.HardwareIO
import sigmacorns.logic.BallTrackingSystem
import sigmacorns.logic.ChaseCoordinator
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.vision.BallDetectionProcessor
import sigmacorns.vision.tracker.TrackerConfig
import kotlin.time.DurationUnit

/**
 * Minimal teleop for testing auto-chase. One button:
 *
 *   RIGHT BUMPER held  →  AUTO CHASE ON (intake running, drive to nearest ball)
 *   release            →  AUTO CHASE OFF, manual drive (gamepad1 sticks)
 *
 * No shoot cycle, no other buttons. If you want the full collect-three-and-shoot
 * behavior, use [sigmacorns.opmode.BallChaseTeleOp] instead.
 */
@TeleOp(name = "Ball Chase TEST", group = "Test")
class BallChaseTestTeleOp : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = false)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = false)

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

        val config = TrackerConfig.loadDefault()
        val tracking = BallTrackingSystem(config = config, io = io)

        val chase = ChaseCoordinator(
            drivetrain = robot.drive,
            tracking = tracking,
            io = io,
            maxSpeed = 0.6,
            stopRadiusM = 0.20,
            slowDownRadiusM = 0.6,
            rotateToTarget = true,
        )
        chase.enabled = false

        telemetry.addLine("Ball Chase TEST — hold right bumper to auto-chase.")
        telemetry.update()

        waitForStart()

        try {
            ioLoop { state, _ ->
                val t = state.timestamp.toDouble(DurationUnit.SECONDS)

                tracking.update(t)

                val chaseOn = gamepad1.right_bumper
                chase.enabled = chaseOn

                if (chaseOn) {
                    chase.update()
                    io.intake = 1.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                } else {
                    robot.drive.update(gamepad1, io)
                    io.intake = 0.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                }
                io.flywheel = 0.0

                telemetry.addData("auto-chase", if (chaseOn) "ON" else "OFF")
                telemetry.addData("pixel dets", processor.detectedBalls.size)
                telemetry.addData("tracks", tracking.tracker.tracks.size)
                val target = tracking.target
                if (target != null) {
                    val p = target.positionAt(t)
                    val pose = io.position()
                    val d = kotlin.math.hypot(p.x - pose.v.x, p.y - pose.v.y)
                    telemetry.addData("target", "#${target.id}  d=%.2fm".format(d))
                } else {
                    telemetry.addLine("no target")
                }
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
