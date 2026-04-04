package sigmacorns.control

import org.joml.Vector4d
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumDynamics

/**
 * Configuration for the anti-wheelie filter.
 *
 * All lengths are in metres. The coordinate origin is the centre of the wheel
 * rectangle (equidistant from front/rear and left/right axles).
 *
 * @param comHeight height of the centre of mass above the ground plane (m)
 * @param comOffsetX longitudinal COM offset from the wheel-rectangle centre (+forward, m)
 * @param comOffsetY lateral COM offset from the wheel-rectangle centre (+left, m)
 * @param minNormalFraction minimum fraction of total robot weight to keep on each
 *   wheel pair (front, rear, left, right). 0.05 means no pair may drop below 5 % of
 *   total weight. Set to 0.0 to allow tipping up to the exact balance point.
 */
data class AntiWheelieConfig(
    val comHeight: Double,
    val comOffsetX: Double = 0.0,
    val comOffsetY: Double = 0.0,
    val minNormalFraction: Double = 0.05
)

/**
 * Predictive anti-wheelie filter for a mecanum drivetrain.
 *
 * Each control loop:
 * 1. The requested duty cycles are converted to wheel torques using the DC motor
 *    model (which includes back-EMF — so hard counter-direction commands while
 *    moving fast produce torques *larger* than stall and are naturally the most
 *    aggressively limited).
 * 2. The resulting robot-frame translational acceleration is computed via the
 *    forward acceleration kinematics.
 * 3. The X and Y components are independently clamped to the maximum values that
 *    keep every wheel pair's normal force above the configured safety margin.
 * 4. If clamping was necessary, the target torques are back-solved and converted
 *    to new duty cycles.
 *
 * The rotational (omega) command is passed through unchanged — rotational tipping
 * is negligible for this chassis geometry.
 *
 * The filter is most permissive from standstill (zero back-EMF → bounded stall
 * torque) and most restrictive during sustained hard deceleration (elevated
 * effective torque amplified by back-EMF), which matches the desired behaviour.
 */
class AntiWheelieFilter(
    val dynamics: MecanumDynamics,
    val config: AntiWheelieConfig
) {
    private val g = 9.81
    private val p = dynamics.p
    private val lx = p.lx
    private val ly = p.ly

    // Maximum forward (+X) acceleration before the front wheel pair lifts
    val axLimitFwd: Double = g * (lx + config.comOffsetX - 2.0 * lx * config.minNormalFraction) / config.comHeight

    // Maximum backward (-X) deceleration magnitude before the rear wheel pair lifts
    val axLimitBwd: Double = g * (lx - config.comOffsetX - 2.0 * lx * config.minNormalFraction) / config.comHeight

    // Maximum leftward (+Y) acceleration before the right wheel pair lifts
    val ayLimitLeft: Double = g * (ly + config.comOffsetY - 2.0 * ly * config.minNormalFraction) / config.comHeight

    // Maximum rightward (-Y) deceleration magnitude before the left wheel pair lifts
    val ayLimitRight: Double = g * (ly - config.comOffsetY - 2.0 * ly * config.minNormalFraction) / config.comHeight

    /**
     * Filter requested wheel duty cycles to prevent tipping.
     *
     * @param requestedPowers duty cycles [FL, BL, BR, FR] ∈ [-1, 1]
     * @param robotVel current robot-relative velocity (vx m/s, vy m/s, omega rad/s)
     * @return filtered duty cycles [FL, BL, BR, FR]; unchanged if within limits
     */
    fun filter(requestedPowers: DoubleArray, robotVel: Pose2d): DoubleArray {
        // Wheel angular velocities (rad/s) from current robot-relative velocity
        val wheelVels = dynamics.mecanumInverseVelKinematics(robotVel)

        // Motor torques including back-EMF effect
        val torques = Vector4d(
            p.motor.torque(requestedPowers[0], wheelVels.x),
            p.motor.torque(requestedPowers[1], wheelVels.y),
            p.motor.torque(requestedPowers[2], wheelVels.z),
            p.motor.torque(requestedPowers[3], wheelVels.w),
        )

        // Robot-frame acceleration that would result from these torques
        val acc = dynamics.mecanumForwardAccKinematics(torques)
        val ax = acc.v.x
        val ay = acc.v.y

        // Clamp to tipping-safe bounds (asymmetric in each axis due to COM offset)
        val axClamped = ax.coerceIn(-axLimitBwd, axLimitFwd)
        val ayClamped = ay.coerceIn(-ayLimitRight, ayLimitLeft)

        if (axClamped == ax && ayClamped == ay) return requestedPowers

        // Back-solve: minimum-norm wheel torques that achieve the clamped acceleration
        val targetTorques = dynamics.mecanumInverseAccKinematics(Pose2d(axClamped, ayClamped, acc.rot))

        // Convert target torques to duty cycles via the inverted motor model:
        //   torque = stallTorque * (power - omega / freeSpeed)
        //   => power = torque / stallTorque + omega / freeSpeed
        return DoubleArray(4) { i ->
            val omega = when (i) { 0 -> wheelVels.x; 1 -> wheelVels.y; 2 -> wheelVels.z; else -> wheelVels.w }
            val torque = when (i) { 0 -> targetTorques.x; 1 -> targetTorques.y; 2 -> targetTorques.z; else -> targetTorques.w }
            (torque / p.motor.stallTorque + omega / p.motor.freeSpeed).coerceIn(-1.0, 1.0)
        }
    }
}
