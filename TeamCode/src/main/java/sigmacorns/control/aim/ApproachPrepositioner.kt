package sigmacorns.control.aim

import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.control.aim.TurretPlannerBridge.PrepositionIdx
import sigmacorns.math.Pose2d

/**
 * Plans a preposition target for the turret while the robot is approaching a
 * shooting location. Uses constant-velocity (mecanum) trajectory prediction
 * to build a future-position path, then calls
 * [TurretPlannerBridge.computeRobustPreposition] to compute a robust target
 * biased toward shots that are cheap to transition between under flywheel loss.
 *
 * ## Frame transform
 *
 * The native `computeRobustPreposition` takes a sequence of target positions
 * with a **fixed** turret pose. Our case is the opposite — stationary goal,
 * moving turret — so we exploit Galilean invariance. In a frame moving with
 * the robot, the goal appears to move at `-v_robot`. At time `t_i = i·dt` the
 * apparent goal position (seen from the *current* robot pose) is
 *
 *     g_i = goal - v_robot · t_i
 *
 * Passing the `g_i` sequence as path samples with `turret = currentRobotPos`
 * and `robot_v = v_robot` makes the C++ solver recover
 * `dx_i = g_i − turret = (goal − (robotPos + v_robot·t_i))`, i.e. the correct
 * displacement from the robot's **future** position at time `t_i` to the
 * stationary goal. The velocity input still contributes the ballistics
 * shoot-while-move correction at each sample.
 *
 * The sampled robust pair (path[i], path[i+1]) thus corresponds to "the shot
 * I would take at time `t_i` plus the shot I would take at time `t_{i+1}`",
 * which is exactly the back-to-back-shot robustness objective we want to plan
 * for while closing on a shooting zone.
 */
class ApproachPrepositioner(
    private val bridge: TurretPlannerBridge,
    private val weights: FloatArray,
    private val bounds: FloatArray,
    private val physConfig: FloatArray,
    private val omegaCoeffs: FloatArray,
) {

    /**
     * @param robotPose Current robot pose (field frame).
     * @param robotVel Current robot field-frame velocity (m/s).
     * @param goal Goal position in 3D (field frame, m).
     * @param turretZ Turret pivot height above ground (m).
     * @param curTheta Current turret angle in field frame (rad).
     * @param curPhi Current hood angle (rad).
     * @param curOmega Current flywheel speed (rad/s).
     * @param omegaDrop Expected flywheel drop between consecutive shots (rad/s).
     * @param horizonSeconds Look-ahead horizon for the predicted trajectory (s).
     * @param steps Number of samples along the horizon (>= 2).
     * @param lambdaDecay Exponential decay on sample weights (0 = uniform).
     * @param tAvailable Slew time available before the path begins (s). Used
     *   to clamp the result to the reachable set.
     * @return FloatArray(4) `[theta, phi, omega, expectedEarliestT]` — see
     *   [PrepositionIdx]. Returns `null` on JNI failure.
     */
    fun compute(
        robotPose: Pose2d,
        robotVel: Vector2d,
        goal: Vector3d,
        turretZ: Double,
        curTheta: Double,
        curPhi: Double,
        curOmega: Double,
        omegaDrop: Double,
        horizonSeconds: Double,
        steps: Int,
        lambdaDecay: Double,
        tAvailable: Double,
    ): FloatArray? {
        val k = maxOf(2, steps)
        val dt = horizonSeconds / (k - 1)

        // Build path samples in the current-robot frame (see class docstring).
        // path[i].(x,y) = goal − v_robot · t_i
        val path = FloatArray(k * 5)
        for (i in 0 until k) {
            val ti = i * dt
            val gx = goal.x - robotVel.x * ti
            val gy = goal.y - robotVel.y * ti
            path[i * 5 + 0] = ti.toFloat()
            path[i * 5 + 1] = gx.toFloat()
            path[i * 5 + 2] = gy.toFloat()
            path[i * 5 + 3] = 0f   // sample vx — unused by preposition solver
            path[i * 5 + 4] = 0f
        }

        return try {
            bridge.computeRobustPreposition(
                path, k,
                robotPose.v.x.toFloat(), robotPose.v.y.toFloat(), turretZ.toFloat(),
                robotVel.x.toFloat(), robotVel.y.toFloat(),
                goal.z.toFloat(),
                curTheta.toFloat(), curPhi.toFloat(), curOmega.toFloat(),
                tAvailable.toFloat(), lambdaDecay.toFloat(), k,
                omegaDrop.toFloat(),
                weights, bounds, physConfig, omegaCoeffs
            )
        } catch (e: Exception) {
            null
        }
    }
}
