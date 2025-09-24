package sigmacorns

import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import kotlin.time.ComparableTimeMark
import kotlin.time.TimeSource

data class State (
    var flywheelSpeed: Double,
    var driveTrainPosition: Pose2d,
    var driveTrainVelocity: Pose2d,
    var driveTrainAcceleration: Pose2d,
    var intakeFlapPosition: Double,
    var intakeRollerPower: Double
) {
    var lastUpdateTime: ComparableTimeMark? = null
    fun update(io: SigmaIO) {
        flywheelSpeed = io.flywheelVelocity()
        driveTrainPosition = io.position()

        val lastVel = driveTrainVelocity
        driveTrainVelocity = io.velocity()

        val t = TimeSource.Monotonic.markNow()
        val dt = t - (lastUpdateTime ?: t)
        lastUpdateTime = t

        driveTrainAcceleration = (driveTrainVelocity - lastVel)/(dt.inWholeMicroseconds.toDouble() / 1_000_000.0)

        intakeRollerPower = io.intakeMotor
    }

    fun toNativeArray(): FloatArray {
        val l = listOf(
            flywheelSpeed,
            driveTrainPosition.v.x, driveTrainPosition.v.y, driveTrainPosition.rot,
            driveTrainVelocity.v.x, driveTrainVelocity.v.y, driveTrainVelocity.rot,
            driveTrainAcceleration.v.x, driveTrainAcceleration.v.y, driveTrainAcceleration.rot,
            intakeFlapPosition,
            intakeRollerPower
        )

        return FloatArray(l.size) {
            l[it].toFloat()
        }
    }
}