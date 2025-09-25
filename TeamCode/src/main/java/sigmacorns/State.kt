package sigmacorns

import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d

data class State (
    var flywheelSpeed: Double,
    var driveTrainPosition: Pose2d,
    var driveTrainVelocity: Pose2d,
    var driveTrainAcceleration: Pose2d,
    var intakeFlapPosition: Double,
    var intakeRollerPower: Double,
    var timestamp: Double
) {
    fun update(io: SigmaIO) {
        flywheelSpeed = io.flywheelVelocity()
        driveTrainPosition = io.position()

        val lastVel = driveTrainVelocity
        driveTrainVelocity = io.velocity()

        val lastTime = timestamp

        timestamp = io.time()
        val dt = timestamp - lastTime

        driveTrainAcceleration = (driveTrainVelocity - lastVel)/dt

        intakeRollerPower = io.intake

    }

    fun toNativeArray(): FloatArray {
        val l = listOf(
            flywheelSpeed,
            driveTrainPosition.v.x, driveTrainPosition.v.y, driveTrainPosition.rot,
            driveTrainVelocity.v.x, driveTrainVelocity.v.y, driveTrainVelocity.rot,
            driveTrainAcceleration.v.x, driveTrainAcceleration.v.y, driveTrainAcceleration.rot,
            intakeFlapPosition,
            intakeRollerPower,
            timestamp
        )

        return FloatArray(l.size) {
            l[it].toFloat()
        }
    }
}