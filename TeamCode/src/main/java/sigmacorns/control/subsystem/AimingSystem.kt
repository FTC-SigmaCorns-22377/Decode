package sigmacorns.control.subsystem

import org.joml.Vector2d
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.turretRange
import sigmacorns.control.aim.AutoAimGTSAM
import sigmacorns.control.aim.VisionTracker
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.test.AutoAimGTSAMTest
import sigmacorns.opmode.test.AutoAimGTSAMTest.Companion.applyRuntimeConfig
import sigmacorns.tuning.AdaptiveTuner
import sigmacorns.tuning.ShotDataStore
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
    lateinit var autoAim: AutoAimGTSAM
        private set
    lateinit var visionTracker: VisionTracker
        private set
    lateinit var turret: Turret
        private set
    lateinit var adaptiveTuner: AdaptiveTuner
        private set

    val goalPosition: Vector2d = FieldLandmarks.goalPosition(blue)

    /** Distance from fused pose to goal, updated each [updateVision] call. */
    var targetDistance: Double = 3.0
        private set

    /**
     * Initialize all subsystems. Call once before the main loop.
     */
    fun init(initialPose: Pose2d) {
        turret = Turret(turretRange, io)

        autoAim = AutoAimGTSAM(
            landmarkPositions = FieldLandmarks.landmarks,
            goalPosition = goalPosition,
            initialPose = initialPose,
            estimatorConfig = AutoAimGTSAMTest.buildEstimatorConfig()
        )

        val limelight = (io as? HardwareIO)?.limelight
        visionTracker = VisionTracker(
            limelight = limelight,
            allowedTagIds = FieldLandmarks.landmarkTagIds
        )

        // Initialize adaptive tuner for flywheel velocity
        val dataStore = ShotDataStore()
        dataStore.load()
        adaptiveTuner = AdaptiveTuner(dataStore)

        applyRuntimeConfig(autoAim)
        visionTracker.configure(pipeline = AutoAimGTSAMTest.AutoAimGTSAMTestConfig.pipeline)
        autoAim.enabled = true
    }

    /**
     * Read vision, update sensor fusion, and recompute target distance.
     * Call at the start of each loop iteration.
     */
    fun updateVision() {
        applyRuntimeConfig(autoAim)
        val visionResult = visionTracker.read()
        autoAim.update(io.position(), turret.pos, visionResult)

        val fusedPose = autoAim.fusedPose
        targetDistance = hypot(goalPosition.x - fusedPose.v.x, goalPosition.y - fusedPose.v.y)
    }

    /**
     * Apply auto-aim turret targeting if enabled and a target is visible.
     * Optionally skipped if the opmode wants full manual control of turret target.
     */
    fun applyAutoAimTarget() {
        if (autoAim.enabled && autoAim.hasTarget) {
            if (turret.fieldRelativeMode) {
                autoAim.getTargetFieldAngle()?.let { turret.fieldTargetAngle = it }
            } else {
                autoAim.getTargetTurretAngle()?.let { turret.targetAngle = it }
            }
            targetDistance = targetDistance.coerceIn(0.1, 10.0)
        }
    }

    /**
     * Update turret heading feedforward and run turret PID.
     * Call after setting turret target (auto or manual).
     */
    fun updateTurret(dt: Duration) {
        turret.robotHeading = autoAim.fusedPose.rot
        turret.robotAngularVelocity = io.velocity().rot
        turret.targetDistance = targetDistance
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
    fun update(dt: Duration) {
        updateVision()
        applyAutoAimTarget()
        updateTurret(dt)
    }

    fun close() {
        autoAim.close()
        visionTracker.stop()
    }
}
