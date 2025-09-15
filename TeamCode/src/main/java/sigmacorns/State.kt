package sigmacorns

data class State (
    var flywheelSpeed: Float,
    val driveTrainPosition: FloatArray,
    val driveTrainVelocity: Float,
    val driveTrainAcceleration: Float,
    val intakeFlapPosition: Float,
    val intakeRollerPower: Float

)