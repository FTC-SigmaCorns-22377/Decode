package sigmacorns.constants

import org.joml.Vector2d
import sigmacorns.control.MotorRangeMapper
import sigmacorns.opmode.tune.FlywheelDeadbeatConfig
import sigmacorns.sim.MecanumParameters
import sigmacorns.sim.FlywheelParameters
import sigmacorns.sim.LinearDcMotor
import kotlin.math.PI

// specs of a goBilda motor without a gearbox (Modern Robotics 12v DC motor)
// from https://motors.vex.com/other-motors/modern-robotics-12vdc

/**
 * Modern Robotics 12v DC motor top speed in rad/s
 */
val bareMotorTopSpeed = 617.84


/**
 * Modern Robotics 12v DC motor stall torque in N*m
 */
val bareMotorStallTorque = 0.187


/**
 * Gear ratio used with a Modern Robotics 12v DC motor.
 * See goBilda website for values
 */
val driveGearRatio = 13.7

val driveMotor = LinearDcMotor(bareMotorTopSpeed/driveGearRatio,bareMotorStallTorque*driveGearRatio)

/**
 * Parameters of the mecanum drivetrain being used on the bot
 */
val drivetrainParameters = MecanumParameters(
    driveMotor,
    0.2,
    0.2,
    0.048,
    15.0,
    1.3,
    0.1,
    0.05
)


val drivetrainCenter = Vector2d(0.03996203, 0.0)

val flywheelGearRatio = 1.0
val flywheelMotor = LinearDcMotor(bareMotorTopSpeed/flywheelGearRatio,bareMotorStallTorque*flywheelGearRatio)

/**
 * Parameters of the dual-motor flywheel used in the simulator
 */
val flywheelParameters = FlywheelParameters(
    flywheelMotor,
    FlywheelDeadbeatConfig.inertia,
    0.0001,
)

/**
 * Default launch angle of the projectile measured from the floor plane.
 */
const val DEFAULT_BALL_LAUNCH_ANGLE_DEGREES = 45.0

val turretTicksPerRad = (1.0 + (46.0 / 11.0)) * 28.0 / (2*PI) * 76 / 19
val turretRange = MotorRangeMapper(
    limits = -PI/2.0..PI/2.0,           // turret can rotate +/- 190 degrees
    limitsTick = -PI/2.0*turretTicksPerRad..PI/2.0*turretTicksPerRad,           // turret can rotate +/- 190 degrees
    slowdownDist = 0.3           // slow down within 0.3 rad of limits
)