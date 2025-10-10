package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.RerunLogging
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumDynamics

@TeleOp(group = "test")
class OdometryTest : SigmaOpMode() {
    private val mecanumDynamics = MecanumDynamics(drivetrainParameters)

    override fun runOpMode() {
        io.configurePinpoint()

        telemetry.addLine("Odometry test ready")
        telemetry.addLine("Press A to zero pose")
        telemetry.update()

        waitForStart()

        RerunLogging.connect("OdometryTest", rerunIP()).use { rerun ->
            var wasResetPressed = false

            ioLoop { state, _ ->
                val resetPressed = gamepad1.a
                if (resetPressed && !wasResetPressed) {
                    io.setPosition(Pose2d())
                }
                wasResetPressed = resetPressed

                val robotPower = Pose2d(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble(),
                )
                val maxSpeed = mecanumDynamics.maxSpeed()
                val desiredVelocity = maxSpeed.componentMul(robotPower)
                var wheelPowers = mecanumDynamics
                    .mecanumInverseVelKinematics(desiredVelocity) * (1.0 / mecanumDynamics.p.motor.freeSpeed)
                val maxComponent = wheelPowers.absolute().maxComponent()
                if (maxComponent > 1.0) {
                    wheelPowers *= (1.0 / maxComponent)
                }

                io.driveFL = wheelPowers[0]
                io.driveBL = wheelPowers[1]
                io.driveBR = wheelPowers[2]
                io.driveFR = wheelPowers[3]

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

                rerun.logState(state)

                false
            }
        }
    }
}
