package sigmacorns.subsystem

import org.joml.Vector2d
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.ShootWhileMoveConstants
import sigmacorns.constants.turretRange
import sigmacorns.control.localization.GTSAMEstimator
import sigmacorns.control.localization.VisionTracker
import sigmacorns.control.aim.tune.AdaptiveTuner
import sigmacorns.control.aim.tune.ShotDataStore
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.time.Duration

/**
 * Shared auto-aim + turret + vision subsystem used across opmodes.
 *
 * Encapsulates AutoAimGTSAM, VisionTracker, and Turret initialization and the
 * core vision → auto-aim → turret update pipeline. Opmodes can override turret
 * targeting between [updateVision] and [updateTurret] for manual control.
 */
class AimingSystem(
    private val robot: Robot,
    private val blue: Boolean
) {
    lateinit var autoAim: GTSAMEstimator
        private set
    var visionTracker: VisionTracker? = null
        private set
    lateinit var adaptiveTuner: AdaptiveTuner
        private set

    var goalPosition: Vector2d = FieldLandmarks.goalPosition(blue)

    var positionOverride: Double? = null

    /** Distance from fused pose to goal, updated each [updateVision] call. */
    var targetDistance: Double = 3.0
        private set

    var radialVelocity: Double = 0.0
        private set

    var turretLeadAngle: Double = 0.0
        private set

    /**
     * Decompose the robot's field-relative velocity into radial and lateral
     * components relative to the goal direction, and compute the turret lead
     * angle to compensate for lateral motion.
     */
    fun updateVelocityComponents() {
        val fusedPose = autoAim.fusedPose
        val vel = robot.io.velocity()

        val toGoal = Vector2d(
            goalPosition.x - fusedPose.v.x,
            goalPosition.y - fusedPose.v.y
        )
        val dist = toGoal.length()
        if (dist < 0.01) {
            radialVelocity = 0.0
            turretLeadAngle = 0.0
            return
        }

        val radialDir = Vector2d(toGoal).div(dist)
        val lateralDir = Vector2d(-radialDir.y, radialDir.x)

        radialVelocity = vel.v.dot(radialDir)
        val lateralVelocity = vel.v.dot(lateralDir)

        turretLeadAngle = atan2(-lateralVelocity, dist) * ShootWhileMoveConstants.turretLookAheadTime
    }

    /**
     * Initialize all subsystems. Call once before the main loop.
     */
    fun init(initialPose: Pose2d, apriltagTracking: Boolean) {
        autoAim = GTSAMEstimator(
            landmarkPositions = FieldLandmarks.landmarks,
            initialPose = initialPose,
        )

        val limelight = (robot.io as? HardwareIO).takeIf { apriltagTracking }?.limelight
        visionTracker = VisionTracker(
            limelight = limelight,
            allowedTagIds = FieldLandmarks.landmarkTagIds
        )

        // Initialize adaptive tuner for flywheel velocity
        val dataStore = ShotDataStore()
        dataStore.load()
        adaptiveTuner = AdaptiveTuner(dataStore)

        autoAim.enabled = true
    }

    /**
     * Read vision, update sensor fusion, and recompute target distance.
     * Call at the start of each loop iteration.
     */
    fun updateVision() {
        val visionResult = visionTracker?.read()
        autoAim.update(robot.io.position(), robot.turret.pos, visionResult)

        val fusedPose = autoAim.fusedPose
        targetDistance = hypot(goalPosition.x - fusedPose.v.x, goalPosition.y - fusedPose.v.y)
    }

    /**
     * Apply auto-aim turret targeting if enabled and a target is visible.
     * Optionally skipped if the opmode wants full manual control of turret target.
     */
    fun applyAutoAimTarget() {
    }

    /**
     * Update turret heading feedforward and run turret PID.
     * Call after setting turret target (auto or manual).
     */
    fun updateTurret(dt: Duration) {
        robot.turret.robotHeading = autoAim.fusedPose.rot
        robot.turret.robotAngularVelocity = robot.io.velocity().rot
        robot.turret.targetDistance = targetDistance

        positionOverride?.let {
            if(robot.turret.fieldRelativeMode) robot.turret.targetAngle = it else robot.turret.fieldTargetAngle = it
        }

        robot.turret.update(dt)
    }

    /**
     * Get recommended flywheel velocity in rad/s for current target distance.
     * Returns null if adaptive tuner doesn't have enough calibration points.
     */
    fun getRecommendedFlywheelVelocity(): Double? {
        return adaptiveTuner.getRecommendedSpeed(targetDistance)
    }

    /**
     * Convenience: full pipeline update (vision → auto-aim → turret).
     * Use this when no manual turret override is needed.
     */
    fun update(dt: Duration, target: Boolean = true) {
        updateVision()
        updateVelocityComponents()
        if(target) applyAutoAimTarget()
        updateTurret(dt)
    }

    fun close() {
        autoAim.close()
    }
}
