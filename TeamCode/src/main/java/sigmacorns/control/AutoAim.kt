package sigmacorns.control

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.joml.Vector2d
import sigmacorns.math.Pose2d
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Auto-aiming system using Limelight camera and AprilTag detection with sensor fusion.
 *
 * Coordinate Transform Chain (camera on turret on robot):
 * 1. Camera detects target at angle (tx, ty) relative to camera center
 * 2. Camera frame → Turret frame (camera points where turret points, offset by cameraMountingOffsetYaw)
 * 3. Turret frame → Robot frame (add current turret angle)
 * 4. Robot frame → Field frame (use robot odometry pose)
 *
 * Sensor Fusion:
 * - When vision is available: Update target field position and use vision for fine adjustment
 * - When vision is lost: Use last known target field position + odometry to predict aim angle
 *
 * @param limelight The Limelight3A camera instance
 * @param targetAprilTagIds List of AprilTag IDs to track (goal targets)
 */
class AutoAim(
    private val limelight: Limelight3A?,
    private val targetAprilTagIds: Set<Int> = setOf(20, 24)
) {
    /** Whether auto-aim is currently enabled */
    var enabled: Boolean = false

    /** Whether a valid target is currently detected by vision */
    var hasVisionTarget: Boolean = false
        private set

    /** Whether we have a valid target (either vision or predicted from odometry) */
    var hasTarget: Boolean = false
        private set

    /** Horizontal offset to target in radians (positive = target is to the right of camera) */
    var targetTx: Double = 0.0
        private set

    /** Vertical offset to target in radians */
    var targetTy: Double = 0.0
        private set

    /** Estimated distance to target in meters */
    var targetDistance: Double = 0.0
        private set

    /** ID of the currently tracked AprilTag */
    var trackedTagId: Int = -1
        private set

    /** Number of AprilTags detected in the last frame */
    var detectedTagCount: Int = 0
        private set

    /** Raw tx value from Limelight before offset correction (degrees) */
    var rawTxDegrees: Double = 0.0
        private set

    /** Raw ty value from Limelight (degrees) */
    var rawTyDegrees: Double = 0.0
        private set

    /** Whether the Limelight result was valid */
    var lastResultValid: Boolean = false
        private set

    /** Timestamp of last successful detection (ms) */
    var lastDetectionTimeMs: Long = 0
        private set

    /** Camera mounting offset from turret center (radians) - adjust based on your robot */
    var cameraMountingOffsetYaw: Double = 0.0

    /** Camera mounting height above ground (meters) */
    var cameraMountingHeight: Double = 0.33

    /** Goal height above ground (meters) */
    var goalHeight: Double = 0.762

    /** Time in ms after which we stop using predicted target position */
    var predictionTimeoutMs: Long = 2000

    /**
     * Sign multiplier for tx.
     * Limelight reports tx positive when target is to the RIGHT.
     * This value flips the correction direction.
     */
    var txSignMultiplier: Double = -1.0

    /** Deadband for tx - ignore vision corrections smaller than this (radians) */
    var txDeadband: Double = 0.01  // ~0.5 degrees

    /** Maximum allowed deviation from predicted position (radians) - readings beyond this are rejected */
    var maxInnovation: Double = 0.35  // ~20 degrees - reject wild readings

    /** Whether the last reading was rejected as an outlier */
    var lastReadingRejected: Boolean = false
        private set

    // ===== Sensor Fusion State =====

    /** Target position in field coordinates (meters) - updated when vision is available */
    var targetFieldPosition: Vector2d? = null
        private set

    /** Last robot pose when target was updated (for odometry prediction) */
    private var lastRobotPoseAtUpdate: Pose2d? = null

    /** Last turret angle when target was updated */
    private var lastTurretAngleAtUpdate: Double = 0.0

    /** The required turret angle in field coordinates to point at target */
    var targetFieldAngle: Double = 0.0
        private set

    /** The required turret angle in robot coordinates to point at target */
    var targetRobotAngle: Double = 0.0
        private set

    /** Whether we're currently using predicted position (vs live vision) */
    var usingPrediction: Boolean = false
        private set

    /** Current uncertainty level (0 = certain, 1 = very uncertain) */
    var uncertainty: Double = 0.0
        private set

    /** Threshold above which vision measurements are rejected */
    var maxAcceptableUncertainty: Double = 0.7

    // Low-pass filter for target position to reduce jitter
    private var filteredTargetFieldPosition: Vector2d? = null
    private val positionFilterAlpha = 0.3 // Higher = more responsive, lower = more smooth

    // Motion tracking for uncertainty estimation
    private var lastRobotPose: Pose2d? = null
    private var lastTurretAngle: Double = 0.0
    private var robotAngularVelocity: Double = 0.0
    private var turretAngularVelocity: Double = 0.0
    private var lastUpdateTimeMs: Long = 0

    // Uncertainty parameters
    private val robotRotationUncertaintyGain = 2.0  // uncertainty per rad/s of robot rotation
    private val turretRotationUncertaintyGain = 1.5 // uncertainty per rad/s of turret rotation
    private val uncertaintyDecayRate = 0.85         // decay per update when stationary

    /**
     * Updates the auto-aim state with full coordinate transforms and sensor fusion.
     *
     * @param robotPose Current robot pose from odometry (field coordinates)
     * @param turretAngle Current turret angle relative to robot (radians)
     */
    fun update(robotPose: Pose2d, turretAngle: Double) {
        val currentTime = System.currentTimeMillis()
        val dt = if (lastUpdateTimeMs > 0) (currentTime - lastUpdateTimeMs) / 1000.0 else 0.02

        // Update motion-based uncertainty
        updateUncertainty(robotPose, turretAngle, dt)

        // Update vision
        updateVision()

        val timeSinceDetection = currentTime - lastDetectionTimeMs

        if (hasVisionTarget && uncertainty < maxAcceptableUncertainty) {
            // Vision is available AND uncertainty is low enough to trust it
            usingPrediction = false

            // When we have good vision, use tx DIRECTLY for the turret adjustment
            // The camera is telling us exactly how far off we are - trust it!
            // Only update field position for prediction purposes
            val targetFieldPos = computeTargetFieldPosition(robotPose, turretAngle)
            if (targetFieldPos != null) {
                // Weight the filter alpha by confidence (lower uncertainty = more weight to new measurement)
                val adaptiveAlpha = positionFilterAlpha * (1.0 - uncertainty)

                filteredTargetFieldPosition = if (filteredTargetFieldPosition == null) {
                    targetFieldPos
                } else {
                    Vector2d(
                        filteredTargetFieldPosition!!.x * (1 - adaptiveAlpha) + targetFieldPos.x * adaptiveAlpha,
                        filteredTargetFieldPosition!!.y * (1 - adaptiveAlpha) + targetFieldPos.y * adaptiveAlpha
                    )
                }

                targetFieldPosition = filteredTargetFieldPosition
                lastRobotPoseAtUpdate = robotPose
                lastTurretAngleAtUpdate = turretAngle
            }

            // For direct vision mode, the target angles come from current turret + tx adjustment
            // Apply sign multiplier and deadband to tx
            val adjustedTx = if (targetTx.absoluteValue < txDeadband) {
                0.0  // In deadband, no correction needed
            } else {
                targetTx * txSignMultiplier
            }

            // This is the KEY FIX: when tx=0, targetRobotAngle = current turret angle (no change needed)
            targetRobotAngle = turretAngle + adjustedTx
            targetFieldAngle = robotPose.rot + targetRobotAngle

            hasTarget = true
        } else if (hasVisionTarget && uncertainty >= maxAcceptableUncertainty) {
            // Vision available but too uncertain - still use prediction if we have it
            if (targetFieldPosition != null && timeSinceDetection < predictionTimeoutMs) {
                usingPrediction = true
                hasTarget = true
                computeAimAnglesFromFieldPosition(robotPose)
            } else {
                usingPrediction = false
                hasTarget = false
            }
        } else if (targetFieldPosition != null && timeSinceDetection < predictionTimeoutMs) {
            // Vision lost but we have a recent target position - use prediction
            usingPrediction = true
            hasTarget = true
            computeAimAnglesFromFieldPosition(robotPose)
        } else {
            // No vision and prediction timed out
            usingPrediction = false
            hasTarget = false
        }

        lastUpdateTimeMs = currentTime
    }

    /**
     * Updates the uncertainty estimate based on motion.
     * High angular velocity of robot or turret = high uncertainty in vision.
     */
    private fun updateUncertainty(robotPose: Pose2d, turretAngle: Double, dt: Double) {
        if (lastRobotPose != null && dt > 0) {
            // Compute angular velocities
            val robotDeltaAngle = normalizeAngle(robotPose.rot - lastRobotPose!!.rot)
            robotAngularVelocity = robotDeltaAngle.absoluteValue / dt

            val turretDeltaAngle = normalizeAngle(turretAngle - lastTurretAngle)
            turretAngularVelocity = turretDeltaAngle.absoluteValue / dt

            // Compute motion-induced uncertainty
            val motionUncertainty = (robotAngularVelocity * robotRotationUncertaintyGain +
                    turretAngularVelocity * turretRotationUncertaintyGain).coerceIn(0.0, 1.0)

            // Blend: uncertainty increases immediately with motion, decays slowly when stationary
            uncertainty = if (motionUncertainty > uncertainty) {
                motionUncertainty
            } else {
                uncertainty * uncertaintyDecayRate + motionUncertainty * (1 - uncertaintyDecayRate)
            }
        }

        lastRobotPose = robotPose
        lastTurretAngle = turretAngle
    }

    /**
     * Legacy update method for backwards compatibility.
     * Use update(robotPose, turretAngle) for full sensor fusion.
     */
    fun update() {
        updateVision()
        hasTarget = hasVisionTarget
    }

    /**
     * Polls the Limelight for vision updates.
     */
    private fun updateVision() {
        if (limelight == null) {
            hasVisionTarget = false
            lastResultValid = false
            detectedTagCount = 0
            return
        }

        val result: LLResult? = limelight.latestResult

        if (result == null || !result.isValid) {
            hasVisionTarget = false
            lastResultValid = false
            detectedTagCount = 0
            return
        }

        lastResultValid = true

        // Check for fiducial (AprilTag) results
        val fiducials: List<FiducialResult> = result.fiducialResults
        detectedTagCount = fiducials.size

        // Find the first target AprilTag that matches our target IDs
        val targetFiducial = fiducials.firstOrNull { fiducial ->
            fiducial.fiducialId in targetAprilTagIds
        }

        if (targetFiducial != null) {
            hasVisionTarget = true
            trackedTagId = targetFiducial.fiducialId
            lastDetectionTimeMs = System.currentTimeMillis()

            // Store raw values for logging
            rawTxDegrees = targetFiducial.targetXDegrees
            rawTyDegrees = targetFiducial.targetYDegrees

            // tx is horizontal offset in degrees, convert to radians
            // Add camera mounting offset to account for camera not being centered on turret
            targetTx = Math.toRadians(targetFiducial.targetXDegrees) + cameraMountingOffsetYaw
            targetTy = Math.toRadians(targetFiducial.targetYDegrees)

            // Get distance from the 3D pose if available
            val robotPoseInTargetSpace = targetFiducial.robotPoseTargetSpace
            if (robotPoseInTargetSpace != null) {
                // Compute Euclidean distance from the 3D position
                val pos = robotPoseInTargetSpace.position
                targetDistance = hypot(hypot(pos.x, pos.y), pos.z)
            } else {
                // Fallback: estimate distance from ty and known heights
                targetDistance = estimateDistanceFromTy(targetTy)
            }
        } else {
            // No matching AprilTag found - ignore non-target tags.
            hasVisionTarget = false
            trackedTagId = -1
        }
    }

    /**
     * Gets the adjusted tx value with sign multiplier and deadband applied.
     */
    fun getAdjustedTx(): Double {
        return if (targetTx.absoluteValue < txDeadband) {
            0.0
        } else {
            targetTx * txSignMultiplier
        }
    }

    /**
     * Computes the target position in field coordinates using the coordinate transform chain.
     *
     * Transform chain:
     * 1. Camera sees target at angle tx (relative to camera forward)
     * 2. Camera forward = turret forward + cameraMountingOffsetYaw
     * 3. Turret forward = robot forward + turretAngle
     * 4. Robot forward = field heading (from odometry)
     *
     * So target field angle = robotHeading + turretAngle + cameraMountingOffsetYaw + tx
     */
    private fun computeTargetFieldPosition(robotPose: Pose2d, turretAngle: Double): Vector2d? {
        if (!hasVisionTarget || targetDistance <= 0) return null

        // Get adjusted tx with sign multiplier applied
        val adjustedTx = getAdjustedTx()

        // Transform through the kinematic chain:
        // Camera → Turret: camera points where turret points (offset already in targetTx via cameraMountingOffsetYaw)
        // Turret → Robot: add turret angle
        // Robot → Field: add robot heading
        val targetAngleInField = robotPose.rot + turretAngle + adjustedTx

        // Compute target position in field coordinates
        // Start from robot position and go targetDistance in the direction of targetAngleInField
        val targetX = robotPose.v.x + targetDistance * cos(targetAngleInField)
        val targetY = robotPose.v.y + targetDistance * sin(targetAngleInField)

        return Vector2d(targetX, targetY)
    }

    /**
     * Computes the required aim angles based on stored target field position (for prediction mode).
     */
    private fun computeAimAnglesFromFieldPosition(robotPose: Pose2d) {
        val target = targetFieldPosition ?: return

        // Vector from robot to target
        val dx = target.x - robotPose.v.x
        val dy = target.y - robotPose.v.y

        // Distance to target (update from field position)
        targetDistance = hypot(dx, dy)

        // Angle to target in field coordinates
        targetFieldAngle = atan2(dy, dx)

        // Convert to robot-relative angle (this is what the turret should point at)
        targetRobotAngle = normalizeAngle(targetFieldAngle - robotPose.rot)
    }

    /**
     * Normalizes angle to [-PI, PI] range.
     */
    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle
        while (normalized > PI) normalized -= 2 * PI
        while (normalized < -PI) normalized += 2 * PI
        return normalized
    }

    /**
     * Estimates distance to target based on vertical angle.
     */
    private fun estimateDistanceFromTy(ty: Double): Double {
        val deltaHeight = goalHeight - cameraMountingHeight
        return if (ty.absoluteValue > 0.01) {
            (deltaHeight / kotlin.math.tan(ty)).absoluteValue
        } else {
            5.0 // Default distance when angle is too small
        }
    }

    /**
     * Gets the turret yaw adjustment needed to center the target.
     * This is the direct vision feedback for fine-tuning when target is visible.
     * Applies sign multiplier and deadband.
     *
     * @return The angle in radians to add to current turret position.
     */
    fun getTurretYawAdjustment(): Double {
        if (!enabled || !hasVisionTarget) {
            return 0.0
        }
        return getAdjustedTx()
    }

    /**
     * Gets the target turret angle in robot-relative coordinates.
     * This uses sensor fusion - when vision is available it's precise,
     * when vision is lost it uses the predicted position from odometry.
     *
     * @return The target turret angle in radians (robot-relative), or null if no target.
     */
    fun getTargetTurretAngle(): Double? {
        if (!enabled || !hasTarget) {
            return null
        }
        return targetRobotAngle
    }

    /**
     * Gets the target turret angle in field-relative coordinates.
     *
     * @return The target turret angle in radians (field-relative), or null if no target.
     */
    fun getTargetFieldAngle(): Double? {
        if (!enabled || !hasTarget) {
            return null
        }
        return targetFieldAngle
    }

    /**
     * Gets the recommended flywheel power based on target distance.
     */
    fun getRecommendedFlywheelPower(maxDistance: Double = 10.0): Double {
        if (!enabled || !hasTarget) {
            return 0.0
        }
        return (targetDistance / maxDistance).coerceIn(0.3, 1.0)
    }

    /**
     * Resets the tracked target position.
     * Call this when you want to clear the prediction state.
     */
    fun resetTarget() {
        targetFieldPosition = null
        filteredTargetFieldPosition = null
        lastRobotPoseAtUpdate = null
        lastRobotPose = null
        hasTarget = false
        usingPrediction = false
        uncertainty = 0.0
    }

    /**
     * Manually sets the target field position.
     * Useful for setting known goal positions.
     */
    fun setTargetFieldPosition(position: Vector2d) {
        targetFieldPosition = position
        filteredTargetFieldPosition = position
        lastDetectionTimeMs = System.currentTimeMillis()
    }

    /**
     * Configures the Limelight for AprilTag detection.
     */
    fun configure(pipeline: Int = 0) {
        limelight?.pipelineSwitch(pipeline)
        limelight?.start()
    }

    /**
     * Stops the Limelight camera.
     */
    fun stop() {
        limelight?.stop()
    }
}
