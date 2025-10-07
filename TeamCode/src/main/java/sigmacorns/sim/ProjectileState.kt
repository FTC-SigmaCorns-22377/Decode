package sigmacorns.sim

import org.joml.Vector3d

/**
 * Represents a projectile's translational state for dynamics integration.
 */
data class ProjectileState(
    val position: Vector3d = Vector3d(),
    val velocity: Vector3d = Vector3d(),
    val active: Boolean = true,
) {
    constructor(data: DoubleArray, active: Boolean) : this(
        position = Vector3d(data[0], data[1], data[2]),
        velocity = Vector3d(data[3], data[4], data[5]),
        active = active,
    )

    fun toDoubleArray(): DoubleArray = doubleArrayOf(
        position.x,
        position.y,
        position.z,
        velocity.x,
        velocity.y,
        velocity.z,
    )
}
