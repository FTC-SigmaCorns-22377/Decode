package sigmacorns.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.globalFieldState
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumDynamics

@TeleOp
class Teleop(): SigmaOpMode() {
    val mecanumDynamics = MecanumDynamics(drivetrainParameters)
    var spinUpToggle = 0

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val voltageSensor = hardwareMap.voltageSensor.iterator().next()
        waitForStart()

        var rampArray = globalFieldState.ramp

        while (opModeIsActive()) {
            val voltage = voltageSensor.voltage

            val dVoltage = 12 / voltage

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

            val robotPower = Pose2d(-gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble(), -gamepad1.right_stick_x.toDouble())
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
                io.intake = -1.0 * dVoltage
            }


            if (gamepad1.right_bumper) { //ejecting the ball
                io.intake = 1.0 * dVoltage
            }

            if (gamepad1.left_trigger > 0.1 * dVoltage && gamepad1.right_trigger < 0.1 * dVoltage){ //intaking
                io.shooter = 0.8 * dVoltage
            }

            //presets
            if (gamepad1.dpadUpWasPressed()) {
                spinUpToggle = if (spinUpToggle == 1) 0 else 1
            } else if (gamepad1.dpadDownWasPressed()){
                spinUpToggle = if (spinUpToggle == 2) 0 else 2
            }

            val spinUpClose = 0.6
            val spinUpFar = 0.79

            if (spinUpToggle == 1 ) {
                io.shooter = -spinUpClose * dVoltage
            } else if (spinUpToggle == 2 ){
                io.shooter = -spinUpFar * dVoltage
            }

            if (gamepad2.aWasPressed()) {
                for (i in 0 until 9) {
                    if (rampArray[i] == 0) {
                        rampArray[i] = 1
                        break
                    }
                }
            }
            if (gamepad2.bWasPressed()) {
                for (i in 0 until 9) {
                    if (rampArray[i] == 0) {
                        rampArray[i] = 2
                        break
                    }
                }
            }
            if (gamepad2.xWasPressed()) {
                for (i in 8 downTo 0) {
                    if (rampArray[i] != 0) {
                        rampArray[i] = 0
                        break
                    }
                }
            }
            if (gamepad2.yWasPressed()) {
                for (i in 0 until 9) {
                    if (rampArray[i] != 0) {
                        rampArray[i] = 0
                    }
                }
            }

            telemetry.addData("Ramp",rampArray)
            telemetry.update()

            io.update()
        }
    }
}