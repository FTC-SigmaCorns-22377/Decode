package sigmacorns.control.aim.ballistic

import sigmacorns.math.normalizeAngle
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * Immutable snapshot of the world state needed to solve the ballistic subproblem.
 *
 * All coordinates are in field frame (meters, radians).
 */
data class ShotState(
    val xRobot: Double,
    val yRobot: Double,
    val vRx: Double,
    val vRy: Double,
    val xTarget: Double,
    val yTarget: Double,
    val zTarget: Double,
    val zTurret: Double,
    val robotHeading: Double
)

/**
 * Result of solving the fixed-T ballistic subproblem.
 *
 * @param T travel time (seconds)
 * @param theta turret yaw angle in field frame (radians)
 * @param phi hood pitch angle (radians), 0 = horizontal
 * @param vExit ball exit velocity magnitude (m/s)
 * @param omega flywheel angular velocity (rad/s)
 * @param feasible true if all parameters are within actuator limits
 */
data class ShotSolution(
    val T: Double,
    val theta: Double,
    val phi: Double,
    val vExit: Double,
    val omega: Double,
    val feasible: Boolean
)

/**
 * Fixed-T ballistic subproblem solver.
 *
 * Given a travel time T, computes the shot parameters (theta, phi, v_exit)
 * that make the ball hit the target. Uses the "without hood arc" formulation
 * (Section 1a of the spec). Hood arc can be added later by setting r_h > 0.
 *
 * See docs/OPTIMIZED_SHOOT.md Section 1 for derivation.
 */
object ShotGeometry {

    /**
     * Solve for shot parameters given travel time T.
     *
     * @param state world state snapshot
     * @param T ball travel time in seconds
     * @param flywheelMap calibration map for omega(phi, v_exit)
     * @return solution, or null if T <= 0
     */
    fun solve(state: ShotState, T: Double, flywheelMap: FlywheelMap): ShotSolution? {
        if (T <= 0.0) return null

        val dx = state.xTarget - state.xRobot
        val dy = state.yTarget - state.yRobot
        val dz = state.zTarget - state.zTurret

        // Decompose required velocity into components
        val a = dx / T - state.vRx
        val b = dy / T - state.vRy
        val c = dz / T + BallisticConstants.G * T / 2.0

        // Solve for theta, phi, v_exit
        val theta = atan2(b, a)
        val horizontalSpeed = sqrt(a * a + b * b)
        val phi = atan2(c, horizontalSpeed)
        val vExit = sqrt(a * a + b * b + c * c)

        if (vExit <= 0.0) return null

        // Convert to flywheel omega via calibration map
        val omega = flywheelMap.getOmega(vExit)

        // Check actuator limits
        val robotTheta = normalizeAngle(theta - state.robotHeading)
        val feasible = phi in BallisticConstants.PHI_MIN..BallisticConstants.PHI_MAX &&
            omega in BallisticConstants.OMEGA_MIN..BallisticConstants.OMEGA_MAX &&
            robotTheta in BallisticConstants.THETA_MIN..BallisticConstants.THETA_MAX

        return ShotSolution(T, theta, phi, vExit, omega, feasible)
    }
}
