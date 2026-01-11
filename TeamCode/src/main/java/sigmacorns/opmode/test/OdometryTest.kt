package sigmacorns.opmode.test

import com.bylazar.gamepad.PanelsGamepad
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.RerunLogging
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.control.DriveController

@TeleOp(group = "test")
class OdometryTest : SigmaOpMode() {
    private val driveController = DriveController()

    override fun runOpMode() {
        io.configurePinpoint()

        telemetry.addLine("Odometry test ready")
        telemetry.addLine("Press A to zero pose")
        telemetry.update()

        waitForStart()

//        val gamepad1 = PanelsGamepad.firstManager.asCombinedFTCGamepad(gamepad1)
//        val gamepad2 = PanelsGamepad.secondManager.asCombinedFTCGamepad(gamepad2)

//        RerunLogging.connect("OdometryTest", rerunIP()).use { rerun ->
            var wasResetPressed = false

            ioLoop { state, _ ->
                val resetPressed = gamepad1.a
                if (resetPressed && !wasResetPressed) {
                    io.setPosition(Pose2d())
                }
                wasResetPressed = resetPressed

                val robotPower = Pose2d(-gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble(), -gamepad1.right_stick_x.toDouble())
                driveController.drive(robotPower, io)

                telemetry.addData(
                    "pose (m, m, rad)",
                    "%.3f, %.3f, %.3f",
                    state.driveTrainPosition.v.x,
                    state.driveTrainPosition.v.y,
                    state.driveTrainPosition.rot,
                )
                telemetry.addData(
                    "vel (m/s, m/s, rad/s)",
                    "%.3f, %.3f, %.3f",
                    state.driveTrainVelocity.v.x,
                    state.driveTrainVelocity.v.y,
                    state.driveTrainVelocity.rot,
                )
                telemetry.update()

//                rerun.logState(state)

                false
            }
//        }
    }
}
