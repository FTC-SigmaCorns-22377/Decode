package sigmacorns.control.aim.ballistic

import sigmacorns.math.normalizeAngle
import kotlin.math.abs

/**
 * Current actuator state snapshot for transition cost computation.
 *
 * @param phi current hood pitch angle (radians)
 * @param omega current flywheel angular velocity (rad/s)
 * @param theta current turret yaw angle in field frame (radians)
 */
data class ActuatorState(
    val phi: Double,
    val omega: Double,
    val theta: Double
)

/**
 * Transition cost function J_delta.
 *
 * Measures the time to transition from the current actuator state to a
 * candidate shot's required state. Modeled as the maximum of weighted
 * deltas (the slowest actuator dominates).
 *
 * See docs/OPTIMIZED_SHOOT.md Section 2.
 */
object TransitionCost {

    /**
     * Compute J_delta: estimated transition time from current state to shot.
     *
     * @return max(Q_phi*|dphi|, Q_omega*|domega|, Q_theta*|dtheta|) in seconds
     */
    fun compute(shot: ShotSolution, current: ActuatorState): Double {
        val dPhi = abs(shot.phi - current.phi)
        val dOmega = abs(shot.omega - current.omega)
        val dTheta = abs(normalizeAngle(shot.theta - current.theta))

        return maxOf(
            BallisticConstants.Q_PHI * dPhi,
            BallisticConstants.Q_OMEGA * dOmega,
            BallisticConstants.Q_THETA * dTheta
        )
    }

    /**
     * Variant using a physics-based flywheel spinup time instead of linear Q_OMEGA.
     *
     * @param flywheelSpinupTime precomputed from [sigmacorns.subsystem.Flywheel.spinupTime]
     */
    fun computeWithSpinup(
        shot: ShotSolution,
        current: ActuatorState,
        flywheelSpinupTime: Double
    ): Double {
        val dPhi = abs(shot.phi - current.phi)
        val dTheta = abs(normalizeAngle(shot.theta - current.theta))

        return maxOf(
            BallisticConstants.Q_PHI * dPhi,
            flywheelSpinupTime,
            BallisticConstants.Q_THETA * dTheta
        )
    }
}
