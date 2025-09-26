package sigmacorns.sim

import sigmacorns.math.Pose2d

/**
 * The state of a mecanum drivetrain
 *
 * @param vel the velocity of the drivetrain. units: `Pose2d(m/s, m/s, rad/s)`
 * @param pos the position of the drivetrain. units: `Pose2d(m, m, rad)`
 */
data class MecanumState(
    var vel: Pose2d,
    var pos: Pose2d
) {
    constructor(data: DoubleArray): this(
        Pose2d(data[0],data[1],data[2]),
        Pose2d(data[3],data[4],data[5])
    )

    fun toDoubleArray(): DoubleArray {
        return doubleArrayOf(vel.v.x, vel.v.y, vel.rot, pos.v.x, pos.v.y, pos.rot)
    }
}
