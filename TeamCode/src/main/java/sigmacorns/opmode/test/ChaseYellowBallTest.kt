package sigmacorns.opmode.test

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.opencv.core.Scalar
import sigmacorns.GlobalShutterVision.EyeBrain
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.Drivetrain
import kotlin.math.abs

/**
 * Drives toward the nearest yellow ball seen by the "globalShutter" webcam.
 * Nearest = biggest single yellow ball on screen. Hold gamepad1 A to let the robot drive.
 */
@TeleOp(name = "Chase Yellow Ball", group = "Test")
class ChaseYellowBallTest : SigmaOpMode() {

    // Max forward power while chasing
    private val chaseSpeed = 0.3
    // Turn power per unit of normalized error (-1..1)
    private val turnGain = 0.4
    // Normalized error below which we stop turning
    private val turnDeadzone = 0.05

    override fun runOpMode() {
        val eyeBrain = EyeBrain().apply {
            useFullFrame = true
            maxBallArea = 640.0 * 480.0
            // Tuned on Global Shutter Camera frames: balls are S ~200, the gray mat and beige
            // clothes are S < 110, so a high saturation floor keeps only the balls
            yellowHsvLower = Scalar(20.0, 140.0, 90.0)
            yellowHsvUpper = Scalar(32.0, 255.0, 255.0)
            // Loose shape filter: touching balls merge into lumpy blobs and must still pass
            minCircularity = 0.2
            minSolidity = 0.6
            maxAspectRatio = 3.0
        }
        val drivetrain = Drivetrain()

        val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "globalShutter"))
            .setCameraResolution(Size(640, 480))
            .addProcessor(eyeBrain)
            .build()

        telemetry.addLine("Chase Yellow Ball ready")
        telemetry.addLine("Press START, then hold A to chase")
        telemetry.update()

        waitForStart()
        if (isStopRequested) {
            visionPortal.close()
            return
        }

        ioLoop { _, _ ->
            // Nearest = biggest single ball (largest radius), aimed at that ball's center
            val target = eyeBrain.detections
                .filter { it.color == "Y" }
                .maxByOrNull { it.radius }
            val halfWidth = (if (eyeBrain.frameWidth > 0) eyeBrain.frameWidth else 640) / 2.0

            if (target != null && gamepad1.a) {
                // How far off-center the ball is (-1.0 left to 1.0 right)
                val error = (target.peakX - halfWidth) / halfWidth
                val turn = if (abs(error) > turnDeadzone) -error * turnGain else 0.0
                // Slow down while the ball is far off-center so we turn first
                val forward = chaseSpeed * (1.0 - abs(error)).coerceAtLeast(0.0)

                drivetrain.drive(Pose2d(forward, 0.0, turn), io)
                telemetry.addData("Error", "%.2f".format(error))
            } else {
                drivetrain.drive(Pose2d(0.0, 0.0, 0.0), io)
            }

            telemetry.addData("Target", target?.let { "(${it.peakX}, ${it.peakY}) radius ${it.radius.toInt()}" } ?: "none")
            telemetry.addData("Chasing", target != null && gamepad1.a)
            telemetry.update()

            false
        }

        visionPortal.close()
    }
}
