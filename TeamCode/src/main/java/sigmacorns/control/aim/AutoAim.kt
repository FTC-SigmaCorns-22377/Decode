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

    /**
     * Most recent solver/preposition target as a ballistic shot state
     * (theta, phi, vExit in field frame). Null when no feasible solve has been
     * produced yet. Exposed for visualization/telemetry only.
     */
    val primaryShotState: Ballistics.ShotState?

    /**
     * Second leg of the most recent robust solve (theta, phi, vExit). Non-null only
     * while a robust solver branch is active ([isRobustActive]). Exposed for
     * visualization/telemetry only.
     */
    val secondaryShotState: Ballistics.ShotState?

    /** True when the most recent update produced a robust (two-shot) solve. */
    val isRobustActive: Boolean

    /** True when the most recent update produced an approach-preposition target. */
    val isPrepositionActive: Boolean

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
