package sigmacorns.constants

import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.control.MotorRangeMapper
import sigmacorns.sim.MecanumParameters
import sigmacorns.sim.FlywheelParameters
import sigmacorns.sim.LinearDcMotor
import kotlin.math.PI

// goBILDA 5203 series bare motor (RS-555) specs at 12V
// from https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/

/**
 * RS-555 bare motor free speed in rad/s (6000 RPM)
 */
val bareMotorTopSpeed = 628.32


/**
 * RS-555 bare motor stall torque in N·m (1.47 kg·cm)
 */
val bareMotorStallTorque = 0.14416


/**
 * goBILDA 5203 planetary gearbox ratio for drive motors (13.7:1 variant)
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
    14.1,
    0.5,
    0.1,
    0.05
)

val flywheelEfficiency = 5650.0 / 6000.0

// Two motors geared 1:1 to flywheel — double the torque
val flywheelMotor = LinearDcMotor(
    bareMotorTopSpeed*flywheelEfficiency,
    2*bareMotorStallTorque*flywheelEfficiency
)

val flywheelRadius = 0.072

val intakeGearRatio = 3.0
val intakeMotor = LinearDcMotor(bareMotorTopSpeed/intakeGearRatio, bareMotorStallTorque*intakeGearRatio)

/**
 * Parameters of the dual-motor flywheel used in the simulator
 */
/** Flywheel inertia in kg*m^2: ½mr² = 0.5 * 0.38709252 kg * (0.046 m)² */
const val FLYWHEEL_INERTIA = 0.000410

val flywheelParameters = FlywheelParameters(
    flywheelMotor,
    FLYWHEEL_INERTIA,
    0.0001,
)

val turretTicksPerRad = (1.0 + (46.0 / 11.0)) * 28.0 / (2*PI) * 76 / 19

// turret center, at at the height of the flywheel center (ballistics assumes turret center is position the ball arc is straight up)
val turretPos = Vector3d(
    -0.05007500, 0.0, 0.313
)

val ballExitRadius = 0.0785