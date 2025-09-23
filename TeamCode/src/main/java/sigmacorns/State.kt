package sigmacorns

import sigmacorns.math.Pose2d

data class State (
    var flywheelSpeed: Double,
    val driveTrainPosition: Pose2d,
    val driveTrainVelocity: Pose2d,
    val driveTrainAcceleration: Pose2d,
    val intakeFlapPosition: Double,
    val intakeRollerPower: Double
)