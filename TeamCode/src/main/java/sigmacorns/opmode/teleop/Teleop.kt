package sigmacorns.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumDynamics

class Teleop(io: SigmaIO): SigmaOpMode(io) {

    val mecanumDynamics = MecanumDynamics(drivetrainParameters)

    override fun runOpMode() {

        telemetry.addLine("SigmusPrime Ready to Go")
        telemetry.update()

        waitForStart()

        //update values

        while (opModeIsActive()) {
            val robotPower = Pose2d(-gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
            val maxSpeed = mecanumDynamics.maxSpeed()
            val robotVelocities = maxSpeed.componentMul(robotPower)
            val wheelVelocities = mecanumDynamics.mecanumInverseVelKinematics(robotVelocities)
            val wheelPowers = wheelVelocities * (1.0/mecanumDynamics.p.freeSpeed)
            io.driveFL = wheelPowers[0]
            io.driveBL = wheelPowers[1]
            io.driveBR = wheelPowers[2]
            io.driveFR = wheelPowers[3]

            io.update()
        }


    }
}