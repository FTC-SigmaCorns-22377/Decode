package sigmacorns.constants

import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.control.MotorRangeMapper
import sigmacorns.opmode.tune.FlywheelDeadbeatConfig
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


val drivetrainCenter = Vector2d(0.03996203, 0.0)

val flywheelGearRatio = 1.0
// Two motors geared 1:1 to flywheel — double the torque
val flywheelMotor = LinearDcMotor(bareMotorTopSpeed/flywheelGearRatio, 2*bareMotorStallTorque*flywheelGearRatio)

val intakeGearRatio = 3.0
val intakeMotor = LinearDcMotor(bareMotorTopSpeed/intakeGearRatio, bareMotorStallTorque*intakeGearRatio)

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
    limits = -PI..PI,           // turret can rotate +/- 180 degrees
    limitsTick = -PI*turretTicksPerRad..PI*turretTicksPerRad,
    slowdownDist = 0.3           // slow down within 0.3 rad of limits
)

val turretPos = Vector3d(
    0.305, 0.0, 0.3289
)

val ballExitRadius = 0.0785