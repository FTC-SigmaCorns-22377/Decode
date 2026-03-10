package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import sigmacorns.GlobalShutterVision.EyeBrain
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.DriveController

@TeleOp(name = "Vision Drive Test", group = "Test")
class VisionDriveTest : SigmaOpMode() {

    // Drive speed when chasing a ball
    private val chaseSpeed = 0.3
    // How many pixels off-center before we start turning
    private val turnDeadzone = 30

    override fun runOpMode() {
        val eyeBrain = EyeBrain()
        val driveController = DriveController()

        val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "globalShutter"))
            .addProcessor(eyeBrain)
            .build()

        // Frame dimensions (update if your camera resolution differs)
        val frameWidth = 640

        telemetry.addLine("Vision Drive Test ready")
        telemetry.addLine("Press START to begin")
        telemetry.update()

        waitForStart()
        if (isStopRequested) return

        ioLoop { _, dt ->
            val detections = eyeBrain.detections
            val motif = eyeBrain.motif

            if (detections.isNotEmpty()) {
                // Target the first (leftmost) ball
                val target = detections[0]
                val centerX = frameWidth / 2

                // How far off-center the ball is (-1.0 to 1.0)
                val error = (target.centerX - centerX).toDouble() / centerX

                // Turn toward the ball, drive forward
                val turn = if (kotlin.math.abs(target.centerX - centerX) > turnDeadzone) {
                    -error * chaseSpeed
                } else {
                    0.0
                }

                driveController.drive(Pose2d(chaseSpeed, 0.0, turn), io)

                telemetry.addData("Target", "${target.color} at (${target.centerX}, ${target.centerY})")
                telemetry.addData("Error", "%.2f".format(error))
            } else {
                // No balls visible — stop
                driveController.drive(Pose2d(0.0, 0.0, 0.0), io)
                telemetry.addData("Target", "none")
            }

            telemetry.addData("Motif", motif)
            telemetry.addData("Balls", detections.size)
            telemetry.update()

            false
        }

        visionPortal.close()
    }
}
