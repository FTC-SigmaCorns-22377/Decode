package sigmacorns.subsystem

import org.joml.Vector2d
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.turretRange
import sigmacorns.control.aim.TurretTargeting
import sigmacorns.control.localization.GTSAMEstimator
import sigmacorns.control.localization.VisionTracker
import sigmacorns.control.aim.tune.AdaptiveTuner
import sigmacorns.control.aim.tune.ShotDataStore
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
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
    private val io: SigmaIO,
    private val blue: Boolean
) {
    lateinit var autoAim: GTSAMEstimator
        private set
    var visionTracker: VisionTracker? = null
        private set
    lateinit var turret: Turret
        private set
    lateinit var adaptiveTuner: AdaptiveTuner
        private set

    var goalPosition: Vector2d = FieldLandmarks.goalPosition(blue)
    private val targeting = TurretTargeting(goalPosition)

    var positionOverride: Double? = null

    /** Distance from fused pose to goal, updated each [updateVision] call. */
    var targetDistance: Double = 3.0
        private set

    /**
     * Initialize all subsystems. Call once before the main loop.
     */
    fun init(initialPose: Pose2d, apriltagTracking: Boolean) {
        turret = Turret(turretRange, io)

        autoAim = GTSAMEstimator(
            landmarkPositions = FieldLandmarks.landmarks,
            initialPose = initialPose,
        )

        val limelight = (io as? HardwareIO).takeIf { apriltagTracking }?.limelight
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
        autoAim.update(io.position(), turret.pos, visionResult)

        val fusedPose = autoAim.fusedPose
        targetDistance = hypot(goalPosition.x - fusedPose.v.x, goalPosition.y - fusedPose.v.y)
    }

    /**
     * Apply auto-aim turret targeting if enabled and a target is visible.
     * Optionally skipped if the opmode wants full manual control of turret target.
     */
    fun applyAutoAimTarget() {
        if (!autoAim.enabled) return
        val angles = targeting.computeAngles(autoAim.fusedPose)
        turret.fieldRelativeMode = true
        turret.fieldTargetAngle = angles.fieldAngle
        targetDistance = angles.distance
    }

    /**
     * Update turret heading feedforward and run turret PID.
     * Call after setting turret target (auto or manual).
     */
    fun updateTurret(dt: Duration) {
        turret.robotHeading = autoAim.fusedPose.rot
        turret.robotAngularVelocity = io.velocity().rot
        turret.targetDistance = targetDistance

        positionOverride?.let {
            if(turret.fieldRelativeMode) turret.targetAngle = it else turret.fieldTargetAngle = it
        }

        turret.update(dt)
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
        if(target) applyAutoAimTarget()
        updateTurret(dt)
    }

    fun close() {
        autoAim.close()
    }
}
