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

    var spinUpToggle = 0
    val mecanumDynamics = MecanumDynamics(drivetrainParameters)
    var dShooterPower = io.shooter - spinUpToggle
//    var shooterExpectedPower = 0

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        waitForStart()

        io.driveFR = 1.0
        io.update()
        sleep(1000)

        while (opModeIsActive()) {
            io.update()
            telemetry.addLine("SigmusPrime Ready to Go")
            telemetry.addLine("DriveFl Power ${io.driveFL}")
            telemetry.addLine("DriveFR Power ${io.driveFR}")
            telemetry.addLine("DriveBL Power ${io.driveBL}")
            telemetry.addLine("DriveBR Power ${io.driveBR}")

            telemetry.addData(
                "pose (m, m, rad)",
                "%.3f, %.3f, %.3f",
                io.position().v.x,
                io.position().v.y,
                io.position().rot,
            )
            telemetry.addData(
                "vel (m/s, m/s, rad/s)",
                "%.3f, %.3f, %.3f",
                io.velocity().v.x,
                io.velocity().v.y,
                io.velocity().rot,
            )
            telemetry.update()

            val voltage = hardwareMap.voltageSensor.iterator().next().voltage
            val dVoltage = 12 / voltage

            val robotPower = Pose2d(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
            val maxSpeed = mecanumDynamics.maxSpeed()
            val robotVelocities = maxSpeed.componentMul(robotPower)
            val wheelVelocities = mecanumDynamics.mecanumInverseVelKinematics(robotVelocities)
            var wheelPowers = wheelVelocities * (1.0 / mecanumDynamics.p.motor.freeSpeed)
            val maxComponents = wheelPowers[wheelPowers.maxComponent()]

            if (maxComponents > 1.0) {
                wheelPowers *= (1.0 / maxComponents)
            }

            io.driveFL = wheelPowers[0]
            io.driveBL = wheelPowers[1]
            io.driveBR = wheelPowers[2]
            io.driveFR = wheelPowers[3]

            println(
                "Robot power = $robotPower " +
                        "Max speed = $maxSpeed " +
                        "Robot Velocities = $robotVelocities " +
                        "Wheel Velocities = $wheelVelocities " +
                        "Wheel Powers = $wheelPowers"
            )

            io.shooter = -gamepad1.right_trigger.toDouble() * dVoltage
            io.intake = gamepad1.left_trigger.toDouble() * dVoltage

            if (gamepad1.left_bumper) { //ejecting the ball
                io.shooter = 0.4 * dVoltage
                io.intake = 1.0 * dVoltage
            }
            if (gamepad1.left_trigger > 0.1 * dVoltage && gamepad1.right_trigger < 0.1 * dVoltage){ //intaking
                    io.shooter = -1.0 * dVoltage
            }

            //presets
            if (gamepad1.dpad_up) {
                spinUpToggle = if (spinUpToggle == 1) 0 else 1
            } else if (gamepad1.dpad_down){
                spinUpToggle = if (spinUpToggle == 2) 0 else 2
            }
            if (spinUpToggle == 1 ) {
                io.shooter = -0.7 * dVoltage
            } else if (spinUpToggle == 2 ){
                io.shooter = -1.0 * dVoltage
            }

//            if (0.05 > dShooterPower && dShooterPower < -0.05) {
//                //shooter can shoot
//            } else {
//                //shooter can't shoot
//            }

        }
    }
}