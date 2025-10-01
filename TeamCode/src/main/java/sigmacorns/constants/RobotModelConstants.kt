package sigmacorns.constants

import sigmacorns.sim.MecanumParameters
import sigmacorns.sim.FlywheelParameters
import sigmacorns.sim.LinearDcMotor

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
    0.870966
)

val flywheelGearRatio = 13.7
val flywheelMotor = LinearDcMotor(bareMotorTopSpeed/flywheelGearRatio,bareMotorStallTorque*flywheelGearRatio)

/**
 * Parameters of the dual-motor flywheel used in the simulator
 */
val flywheelParameters = FlywheelParameters(
    flywheelMotor,
    0.0025,
    0.0001,
)