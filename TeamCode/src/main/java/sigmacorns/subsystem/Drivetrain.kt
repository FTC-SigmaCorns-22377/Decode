package sigmacorns.subsystem

import com.qualcomm.robotcore.hardware.Gamepad
import org.joml.Matrix2d
import org.joml.Vector4d
import org.joml.times
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.AntiWheelieFilter
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumDynamics

class Drivetrain(val antiWheelieFilter: AntiWheelieFilter? = null) {
    private val mecanumDynamics = MecanumDynamics(drivetrainParameters)
    private var speedMultiplier = 1.0
    var fieldCentric = true
    /** Heading (radians) used for field-centric rotation. Set each loop from fused localization. */
    var fieldCentricHeading: Double? = null

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

        var inputX = -gamepad.left_stick_y.toDouble() * speedMultiplier
        var inputY = -gamepad.left_stick_x.toDouble() * speedMultiplier

        if (fieldCentric) {
            val heading = fieldCentricHeading ?: io.position().rot
            val rotated = Matrix2d().rotate(-heading).transform(org.joml.Vector2d(inputX, inputY))
            inputX = rotated.x
            inputY = rotated.y
        }

        val robotPower = Pose2d(
            inputX,
            inputY,
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
        var wheelPowers = Vector4d(
            robotPower.v.x - robotPower.v.y - robotPower.rot,
            robotPower.v.x + robotPower.v.y - robotPower.rot,
            robotPower.v.x - robotPower.v.y + robotPower.rot,
            robotPower.v.x + robotPower.v.y + robotPower.rot,
        )

        val maxComponent = wheelPowers[wheelPowers.maxComponent()]
        if (maxComponent > 1.0) {
            wheelPowers *= (1.0 / maxComponent)
        }

        val raw = doubleArrayOf(wheelPowers[0], wheelPowers[1], wheelPowers[2], wheelPowers[3])

        val powers = if (antiWheelieFilter != null) {
            val fieldVel = io.velocity()
            val heading = io.position().rot
            val robotVel = Pose2d(fieldVel.v.x, fieldVel.v.y, fieldVel.rot).also {
                it.v = Matrix2d().rotate(-heading) * it.v
            }
            antiWheelieFilter.filter(raw, robotVel)
        } else raw

        io.driveFL = powers[0]
        io.driveBL = powers[1]
        io.driveBR = powers[2]
        io.driveFR = powers[3]

        return powers
    }
    
    fun getSpeedMultiplier(): Double = speedMultiplier
}
