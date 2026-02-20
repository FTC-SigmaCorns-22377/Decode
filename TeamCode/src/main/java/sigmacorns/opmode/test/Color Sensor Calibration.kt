package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import sigmacorns.io.HardwareIO
import sigmacorns.opmode.SigmaOpMode

@TeleOp(name = "Color Sensor Calibration", group = "Test")
class ColorSensorCalibration : SigmaOpMode() {
    override fun runOpMode() {
        waitForStart()

        ioLoop { state, dt ->
            // Display raw color values
            telemetry.addData("Ball Detected?", io.colorSensorDetectsBall())

            val hardwareIO = io as? HardwareIO
            val colorSensor = hardwareIO?.colorSensor

            if (colorSensor != null) {
                telemetry.addData("Red", colorSensor.red())
                telemetry.addData("Green", colorSensor.green())
                telemetry.addData("Blue", colorSensor.blue())
                telemetry.addData("Alpha", colorSensor.alpha())

                // If it's also a distance sensor
                if (colorSensor is DistanceSensor) {
                    telemetry.addData("Distance (cm)",
                        colorSensor.getDistance(DistanceUnit.CM))
                }
            }

            telemetry.addData("Detected Color", io.colorSensorGetBallColor())
            telemetry.update()

            false
        }
    }
}
