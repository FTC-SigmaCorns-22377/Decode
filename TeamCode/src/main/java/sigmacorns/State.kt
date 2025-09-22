package sigmacorns

data class State (
    var flywheelSpeed: Double,
    val driveTrainPosition: DoubleArray,
    val driveTrainVelocity: Double,
    val driveTrainAcceleration: Double,
    val intakeFlapPosition: Double,
    val intakeRollerPower: Double
)