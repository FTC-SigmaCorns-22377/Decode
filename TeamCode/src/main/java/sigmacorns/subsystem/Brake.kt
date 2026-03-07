package sigmacorns.subsystem

import sigmacorns.Robot
import sigmacorns.constants.BrakeConstants
import kotlin.math.absoluteValue
import kotlin.time.Duration

class Brake(val robot: Robot) {

    var autoBrake = true

    init {
        disengageBrake()
    }

    fun update(dt: Duration) {

        if (autoBrake) {
            // if power is not being driven to drivetrain engage brakes
            if (
                robot.io.driveFL.absoluteValue <= 0.02 &&
                robot.io.driveFR.absoluteValue <= 0.02 &&
                robot.io.driveBL.absoluteValue <= 0.02 &&
                robot.io.driveBR.absoluteValue <= 0.02
            ) {
                engageBrake()
            } else {
                disengageBrake()
            }
        }

    }

    fun engageBrake() {
        robot.io.frontBrake = BrakeConstants.brakeDown
        robot.io.backBrake = BrakeConstants.brakeDown
    }

    fun disengageBrake() {
        robot.io.frontBrake = BrakeConstants.brakeUp
        robot.io.backBrake = BrakeConstants.brakeUp
    }

}