package sigmacorns.constants

import sigmacorns.sim.MecanumParameters

// specs of a goBilda motor without a gearbox (Modern Robotics 12v DC motor)
// from https://motors.vex.com/other-motors/modern-robotics-12vdc

/**
 * Modern Robotics 12v DC motor top speed in rad/s
 */
private val bareMotorTopSpeed = 617.84

/**
 * Modern Robotics 12v DC motor stall torque in N*m
 */
private val bareMotorStallTorque = 0.187


/**
 * Gear ratio used with a Modern Robotics 12v DC motor.
 * See goBilda website for values
 */
val gearRatio = 13.7

/**
 * Parameters of the mecanum drivetrain being used on the bot
 */
val drivetrainParameters = MecanumParameters(
    bareMotorTopSpeed/gearRatio,
    bareMotorStallTorque*gearRatio,
    0.2,
    0.2,
    0.048,
    15.0,
    0.870966
)