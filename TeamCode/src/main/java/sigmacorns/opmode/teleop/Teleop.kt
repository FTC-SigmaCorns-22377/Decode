package sigmacorns.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumDynamics

@TeleOp
class Teleop(): SigmaOpMode() {

    val mecanumDynamics = MecanumDynamics(drivetrainParameters)

    @Throws(InterruptedException::class)
    override fun runOpMode() {


        waitForStart()



        while (opModeIsActive()) {

            telemetry.addLine("SigmusPrime Ready to Go")
            telemetry.addLine("DriveFl Power ${io.driveFL}")
            telemetry.addLine("DriveFR Power ${io.driveFR}")
            telemetry.addLine("DriveBL Power ${io.driveBL}")
            telemetry.addLine("DriveBR Power ${io.driveBR}")
            telemetry.update()

            val robotPower = Pose2d(-gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble(), -gamepad1.right_stick_x.toDouble())
            val maxSpeed = mecanumDynamics.maxSpeed()
            val robotVelocities = maxSpeed.componentMul(robotPower)
            val wheelVelocities = mecanumDynamics.mecanumInverseVelKinematics(robotVelocities)
            var wheelPowers = wheelVelocities * (1.0/mecanumDynamics.p.motor.freeSpeed)
            val maxComponents = wheelPowers.absolute().maxComponent()


            if (maxComponents > 1.0) {
                wheelPowers *= (1.0/maxComponents)
            }


            io.driveFL = wheelPowers[0]
            io.driveBL = wheelPowers[1]
            io.driveBR = wheelPowers[2]
            io.driveFR = wheelPowers[3]

            println("Robot power = $robotPower " +
                    "Max speed = $maxSpeed " +
                    "Robot Velocities = $robotVelocities " +
                    "Wheel Velocities = $wheelVelocities " +
                    "Wheel Powers = $wheelPowers")

            val isAPressed = gamepad1.a
            val isBPressed = gamepad1.b
            val isXPressed = gamepad1.x
            val isYPressed = gamepad1.y
            if (isAPressed)
                io.shooter = 1.0
            else if (isBPressed)
                io.shooter = -1.0
            else
                io.shooter = 0.0
            if (isXPressed)
                io.intake = 0.5
            else if (isYPressed)
                io.intake = 0.0
            else
                io.intake = 0.0

            io.update()
        }

    }
}