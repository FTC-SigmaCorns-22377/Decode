package sigmacorns.control

import com.qualcomm.robotcore.hardware.Gamepad
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumDynamics

class DriveController {
    private val mecanumDynamics = MecanumDynamics(drivetrainParameters)
    private var speedMultiplier = 1.0

    /**
     * Process gamepad input and update drivetrain.
     * Handles speed multiplier toggling (dpad_up/down).
     * @return DoubleArray of applied wheel powers [FL, BL, BR, FR]
     */
    fun update(gamepad: Gamepad, io: SigmaIO): DoubleArray {
        // Speed mode toggle
        if (gamepad.dpad_up) {
            speedMultiplier = 1.0  // Full speed
        } else if (gamepad.dpad_down) {
            speedMultiplier = 0.5  // Precision mode
        }

        // Mecanum drive calculation
        val robotPower = Pose2d(
            -gamepad.left_stick_y.toDouble() * speedMultiplier,
            -gamepad.left_stick_x.toDouble() * speedMultiplier,
            -gamepad.right_stick_x.toDouble() * speedMultiplier
        )

        return computeAndSetPower(robotPower, io)
    }

    /**
     * Directly set drive power from a Pose2d input (x, y, turn).
     * Does NOT apply internal speed multiplier.
     * @return DoubleArray of applied wheel powers [FL, BL, BR, FR]
     */
    fun drive(robotPower: Pose2d, io: SigmaIO): DoubleArray {
        return computeAndSetPower(robotPower, io)
    }

    private fun computeAndSetPower(robotPower: Pose2d, io: SigmaIO): DoubleArray {
        val maxSpeed = mecanumDynamics.maxSpeed()
        val robotVelocities = maxSpeed.componentMul(robotPower)
        val wheelVelocities = mecanumDynamics.mecanumInverseVelKinematics(robotVelocities)
        var wheelPowers = wheelVelocities * (1.0 / mecanumDynamics.p.motor.freeSpeed)

        val maxComponent = wheelPowers[wheelPowers.maxComponent()]
        if (maxComponent > 1.0) {
            wheelPowers *= (1.0 / maxComponent)
        }

        io.driveFL = wheelPowers[0]
        io.driveBL = wheelPowers[1]
        io.driveBR = wheelPowers[2]
        io.driveFR = wheelPowers[3]

        return doubleArrayOf(wheelPowers[0], wheelPowers[1], wheelPowers[2], wheelPowers[3])
    }
    
    fun getSpeedMultiplier(): Double = speedMultiplier
}
