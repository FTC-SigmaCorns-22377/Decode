package sigmacorns.control.mpc

/**
 * MPC tuning parameters matching the mecanum_mpc formulation.
 * See gen_mpc_native.py for cost function details.
 *
 * Cost = w_normal * normal_err^2 + w_lag * lag_err^2 + w_heading * heading_err^2
 *      + w_u * ||u||^2 + w_du * ||u - u_last||^2
 *      + w_vel * vel_err^2 + w_omega * omega_err^2
 *
 * Tuning guidelines:
 * - wNormal: High values (1000-10000) keep robot close to path. Start high, reduce if too aggressive.
 * - wLag: Moderate values (100-1000) control how tightly robot follows timing. Lower = more relaxed.
 * - wHeading: Lower values (50-500) for heading. Too high causes oscillation when path changes direction.
 * - wU: Control effort penalty (1-50). Higher = slower, smoother movements. Prevents motor saturation.
 * - wDU: Control rate penalty (10-200). CRITICAL for smoothness. Higher = less jitter but slower response.
 * - wVel: Velocity tracking weight (0-100). Penalizes deviation from target linear velocity.
 * - wOmega: Angular velocity tracking weight (0-100). Penalizes deviation from target angular velocity.
 *
 * For jitter issues: Increase wDU first (try 50-100), then wU (try 10-30).
 */
data class MPCTuning(
    /** Weight for normal error (perpendicular to path) */
    var wNormal: Double = 500.0,
    /** Weight for lag error (along path direction) */
    var wLag: Double = 350.0,
    /** Weight for heading error */
    var wHeading: Double = 80.0,
    /** Weight for control effort (penalizes torque) */
    var wU: Double = 0.2,
    /** Weight for control rate (penalizes rapid changes - reduces jitter) */
    var wDU: Double = 2.5,
    /** Weight for velocity tracking error */
    var wVel: Double = 5.0,
    /** Weight for angular velocity tracking error */
    var wOmega: Double = 10.0,
) {
    fun toArray(): DoubleArray = doubleArrayOf(
        wNormal, wLag, wHeading, wU, wDU, wVel, wOmega
    )
}
