package sigmacorns.control.aim.ballistic

import sigmacorns.subsystem.HoodConfig
import kotlin.math.PI

/**
 * Constants for the ballistic trajectory solver.
 *
 * Physical constants reference [HoodConfig] where possible.
 * Q coefficients and search parameters are tunable via Panels dashboard.
 */
object BallisticConstants {
    // Physical constants (delegates to HoodConfig so dashboard tuning stays in sync)
    val G get() = HoodConfig.gravity

    // Actuator limits (radians for angles)
    val PHI_MIN get() = Math.toRadians(HoodConfig.minAngleDeg)
    val PHI_MAX get() = Math.toRadians(HoodConfig.maxAngleDeg)
    const val OMEGA_MIN = 50.0   // rad/s flywheel
    const val OMEGA_MAX = 628.0  // rad/s flywheel
    const val THETA_MIN = -PI / 2.0 // turret limit (rad)
    const val THETA_MAX = PI / 2.0  // turret limit (rad)

    // Transition cost settling-time coefficients (seconds per unit change)
    @JvmField var Q_PHI = 0.15     // hood settling time per radian
    @JvmField var Q_OMEGA = 0.005  // flywheel settling time per rad/s
    @JvmField var Q_THETA = 0.10   // turret settling time per radian

    // Golden section search parameters
    const val GOLDEN_RATIO = 1.618033988749895
    const val SEARCH_TOLERANCE = 0.001 // seconds
    const val MAX_SEARCH_ITERATIONS = 50
    const val WARM_START_WINDOW_FRACTION = 0.3

    // Robust shot planner
    const val ROBUST_GRID_RESOLUTION = 20
    @JvmField var FLYWHEEL_DROP_FRACTION = 0.7 // fraction of omega retained after shot
}
