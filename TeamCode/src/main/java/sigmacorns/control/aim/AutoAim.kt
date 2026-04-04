package sigmacorns.control.aim

import org.joml.Vector2d
import sigmacorns.control.localization.GTSAMEstimator
import sigmacorns.math.Pose2d
import kotlin.time.Duration

/**
 * Interface for the full auto-aim pipeline: vision → sensor fusion → shot solver → subsystem commands.
 *
 * Two concrete implementations exist:
 *   - [sigmacorns.logic.AimingSystem]  — pure-Kotlin Piyavskii-Shubert solver
 *   - [sigmacorns.logic.NativeAutoAim] — C++ turret_planner via JNI
 *
 * Swap in [sigmacorns.Robot] by passing `useNativeAim = true`.
 */
interface AutoAim : AutoCloseable {

    /**
     * Underlying GTSAM estimator. Exposed so opmodes can read [GTSAMEstimator.fusedPose],
     * [GTSAMEstimator.hasVisionTarget], and call [GTSAMEstimator.enableDebugLogging].
     */
    val autoAim: GTSAMEstimator

    /** Distance from fused pose to goal, updated each frame. */
    val targetDistance: Double

    /**
     * True when turret, hood, and flywheel are all within tolerance of the current target.
     * Set to false if no feasible solution is found.
     */
    val readyToShoot: Boolean

    /**
     * Set to true to arm a shot. The implementation fires (triggers TRANSFERRING) as soon as
     * all actuators are within tolerance. Cleared automatically after firing.
     */
    var shotRequested: Boolean

    /** Field-space 2D goal position. Opmodes may override (e.g. mirrored alliance goal). */
    var goalPosition: Vector2d

    /**
     * When non-null, overrides the turret field/robot target angle after the solver runs.
     * Useful for manual aim-override opmodes.
     */
    var positionOverride: Double?

    /** Called once before the main loop. Initialises GTSAM with the robot's starting pose. */
    fun init(initialPose: Pose2d, apriltagTracking: Boolean)

    /**
     * Full pipeline update: vision → fusion → solver → subsystem commands.
     *
     * @param dt         time step (from [sigmacorns.Robot.update])
     * @param aimTurret  when false the turret target angle is not written (caller controls turret)
     */
    fun update(dt: Duration, aimTurret: Boolean = true)

    override fun close()
}
