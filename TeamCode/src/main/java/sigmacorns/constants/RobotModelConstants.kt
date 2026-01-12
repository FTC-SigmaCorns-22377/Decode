package sigmacorns.constants

import sigmacorns.sim.MecanumParameters
import sigmacorns.sim.FlywheelParameters
import sigmacorns.sim.LinearDcMotor
import sigmacorns.sim.SpindexerParameters
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
val driveGearRatio = 19.2

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
    0.5
)

val flywheelGearRatio = 13.7
val flywheelMotor = LinearDcMotor(bareMotorTopSpeed/flywheelGearRatio,bareMotorStallTorque*flywheelGearRatio)
val spindexerGearRatio = 10.0
val spinMotor = LinearDcMotor(bareMotorTopSpeed/spindexerGearRatio,bareMotorStallTorque*spindexerGearRatio)

/**
 * Parameters of the dual-motor flywheel used in the simulator
 */
val flywheelParameters = FlywheelParameters(
    flywheelMotor,
    0.025,
    0.0001,
)

val spindexerParameters = SpindexerParameters(
    spinMotor,
    6.0,
    7.0
)

/**
 * Default launch angle of the projectile measured from the floor plane.
 */
const val DEFAULT_BALL_LAUNCH_ANGLE_DEGREES = 45.0

/**
 * Launch angle expressed in radians for convenience when simulating trajectories.
 */
val BALL_LAUNCH_ANGLE_RADIANS = DEFAULT_BALL_LAUNCH_ANGLE_DEGREES * (PI / 180.0)

/**
 * Approximated linear scale factor converting flywheel angular velocity to muzzle speed (m/s per rad/s).
 */
const val BALL_EXIT_SPEED_PER_RADIAN = 0.025

/**
 * Approximate release height of the launcher exit from the floor in meters.
 */
const val BALL_LAUNCH_HEIGHT_METERS = 0.12

/**
 * Magnitude of gravitational acceleration applied to launched projectiles in the simulator.
 */
const val BALL_GRAVITY_MAGNITUDE = 9.81

/**
 * Time it takes for a ball to come from the spindexer to launch.
 */

const val TIME_LAG_LAUNCH = 0.00