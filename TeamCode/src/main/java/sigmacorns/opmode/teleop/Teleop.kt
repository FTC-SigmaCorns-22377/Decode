package sigmacorns.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumDynamics

class Teleop(io: SigmaIO): SigmaOpMode(io) {

    val mecanumDynamics = MecanumDynamics(drivetrainParameters)

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val flywheel = hardwareMap.get<DcMotor>(DcMotor::class.java, "flyWheel0")
        val intake = hardwareMap.get<DcMotor>(DcMotor::class.java,"intake")


        telemetry.addLine("SigmusPrime Ready to Go")
        telemetry.update()

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

            val isAPressed = gamepad1.a
            val isBPressed = gamepad1.b
            val isXPressed = gamepad1.x
            val isYPressed = gamepad1.y
            if (isAPressed)
                flywheel.setPower(0.5)
            else if (isBPressed)
                flywheel.setPower(0.25)
            else
                flywheel.setPower(0.0)
            if (isXPressed)
                intake.setPower(0.5)
            else if (isYPressed)
                intake.setPower(-1.0)
            else
                intake.setPower(0.0)

            io.update()
        }

    }
}