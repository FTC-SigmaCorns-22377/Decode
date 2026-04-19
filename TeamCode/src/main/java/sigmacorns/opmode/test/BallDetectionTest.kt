package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.vision.BallDetectionProcessor

@TeleOp(name = "Ball Detection Test", group = "Test")
class BallDetectionTest : SigmaOpMode() {

    override fun runOpMode() {
        val processor = BallDetectionProcessor()

        val portal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(processor)
            .enableLiveView(true)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        telemetry.addLine("Ball Detection ready — press START")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val balls = processor.detectedBalls

            telemetry.addData("Balls detected", balls.size)
            telemetry.addData("Sequence (L→R)", processor.ballColorString)

            for ((i, ball) in balls.withIndex()) {
                telemetry.addData(
                    "Ball $i",
                    "%s  x=%d y=%d w=%d h=%d  area=%.0f",
                    ball.color.name,
                    ball.bbox.x, ball.bbox.y,
                    ball.bbox.width, ball.bbox.height,
                    ball.area,
                )
            }

            telemetry.update()
        }

        portal.close()
        processor.release()
    }
}
