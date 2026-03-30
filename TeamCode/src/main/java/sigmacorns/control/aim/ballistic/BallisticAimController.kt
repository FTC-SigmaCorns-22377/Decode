package sigmacorns.control.aim.ballistic

import sigmacorns.math.Pose2d
import sigmacorns.subsystem.HoodConfig

/**
 * High-level ballistic aiming controller.
 *
 * Replaces the simple lead-angle shoot-while-move approach with a proper
 * ballistic trajectory solver. Computes optimal shot parameters (theta, phi, omega)
 * that minimize transition time from the current actuator state.
 *
 * Usage in [sigmacorns.subsystem.AimingSystem]:
 * - Call [update] each frame (200Hz) with current state
 * - Read [currentSolution] for turret/hood/flywheel targets
 * - Call [planShot] once when operator triggers a shot sequence
 */
class BallisticAimController(
    private val xGoal: Double,
    private val yGoal: Double,
    private val zGoal: Double,
    flywheelMapPath: String? = null
) {
    val flywheelMap = FlywheelMap(flywheelMapPath ?: FlywheelMap.DEFAULT_DATA_FILE)
    private val search = OptimalShotSearch(flywheelMap)

    /** Latest solution from the 200Hz loop. */
    var currentSolution: ShotSolution? = null
        private set

    /** Latest robust shot plan (computed once per shot sequence). */
    var robustPlan: RobustShotPlanner.RobustShotResult? = null
        private set

    /** Current travel time bounds (for telemetry). */
    var currentTRange: ClosedRange<Double>? = null
        private set

    fun init() {
        flywheelMap.load()
    }

    /**
     * 200Hz update: recompute optimal shot parameters.
     *
     * Builds immutable [ShotState] and [ActuatorState] snapshots, then runs
     * the golden-section search to find the best travel time T.
     *
     * @return best feasible solution, or null if none found
     */
    fun update(
        fusedPose: Pose2d,
        velocity: Pose2d,
        hoodAngle: Double,
        flywheelOmega: Double,
        turretAngleField: Double
    ): ShotSolution? {
        val state = ShotState(
            xRobot = fusedPose.v.x,
            yRobot = fusedPose.v.y,
            vRx = velocity.v.x,
            vRy = velocity.v.y,
            xTarget = xGoal,
            yTarget = yGoal,
            zTarget = zGoal,
            zTurret = HoodConfig.launchHeight,
            robotHeading = fusedPose.rot
        )

        val current = ActuatorState(
            phi = hoodAngle,
            omega = flywheelOmega,
            theta = turretAngleField
        )

        val tRange = TravelTimeBounds.compute(state, flywheelMap)
        currentTRange = tRange
        if (tRange == null) {
            currentSolution = null
            return null
        }

        currentSolution = search.search(state, current, tRange)
        return currentSolution
    }

    /**
     * One-shot planning for robust multi-ball shot sequences.
     *
     * Call when the operator triggers a shot. Computes the best (T1, T2) pair
     * that minimizes the flywheel recovery time between consecutive shots.
     */
    fun planShot(
        fusedPose: Pose2d,
        velocity: Pose2d,
        hoodAngle: Double,
        flywheelOmega: Double,
        turretAngleField: Double
    ) {
        val state = ShotState(
            xRobot = fusedPose.v.x,
            yRobot = fusedPose.v.y,
            vRx = velocity.v.x,
            vRy = velocity.v.y,
            xTarget = xGoal,
            yTarget = yGoal,
            zTarget = zGoal,
            zTurret = HoodConfig.launchHeight,
            robotHeading = fusedPose.rot
        )

        val current = ActuatorState(
            phi = hoodAngle,
            omega = flywheelOmega,
            theta = turretAngleField
        )

        val tRange = TravelTimeBounds.compute(state, flywheelMap) ?: return
        robustPlan = RobustShotPlanner.plan(state, current, tRange, flywheelMap)
    }

    fun reset() {
        search.reset()
        currentSolution = null
        robustPlan = null
        currentTRange = null
    }
}
