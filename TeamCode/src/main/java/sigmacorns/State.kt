package sigmacorns

import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.FlywheelState
import sigmacorns.sim.MecanumState
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

data class State (
    var flywheelSpeed: Double,
    var driveTrainPosition: Pose2d,
    var driveTrainVelocity: Pose2d,
    var driveTrainAcceleration: Pose2d,
    var intakeFlapPosition: Double,
    var intakeRollerPower: Double,
    var timestamp: Duration
) {
    val mecanumState
        get() = MecanumState(driveTrainVelocity,driveTrainPosition)
    val flyWheelState
        get() = FlywheelState(flywheelSpeed)

    constructor(io: SigmaIO): this(
        0.0,
        Pose2d(),
        Pose2d(),
        Pose2d(),
        0.0,
        0.0,
        0.seconds
    ) {
        update(io)
    }

    fun update(io: SigmaIO) {
        flywheelSpeed = io.flywheelVelocity()
        driveTrainPosition = io.position()

        val lastVel = driveTrainVelocity
        driveTrainVelocity = io.velocity()

        val lastTime = timestamp

        timestamp = io.time()
        val dt = timestamp - lastTime

        driveTrainAcceleration = (driveTrainVelocity - lastVel)/dt.toDouble(DurationUnit.SECONDS)

        intakeRollerPower = io.intake

    }

    fun toDoubleArray(): DoubleArray {
        return doubleArrayOf(
            flywheelSpeed,
            driveTrainPosition.v.x, driveTrainPosition.v.y, driveTrainPosition.rot,
            driveTrainVelocity.v.x, driveTrainVelocity.v.y, driveTrainVelocity.rot,
            driveTrainAcceleration.v.x, driveTrainAcceleration.v.y, driveTrainAcceleration.rot,
            intakeFlapPosition,
            intakeRollerPower,
            timestamp.toDouble(DurationUnit.SECONDS)
        )
    }

    fun toFloatArray(): FloatArray {
        val l = toDoubleArray()
        return FloatArray(l.size) {
            l[it].toFloat()
        }
    }
}