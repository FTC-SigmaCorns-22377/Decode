package sigmacorns.control.aim

import org.joml.Vector2d
import org.joml.Vector3d
import dev.frozenmilk.sinister.util.NativeLibraryLoader
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

/**
 * Auto-aiming system with GTSAM-based sensor fusion.
 *
 * Fuses wheel odometry with AprilTag vision measurements using GTSAM factor graph
 * optimization for robust localization and targeting. Runs GTSAM on a dedicated
 * worker thread to avoid blocking the main control loop.
 */
class AutoAimGTSAM(
    private val landmarkPositions: Map<Int, LandmarkSpec>,
    private val goalPosition: Vector2d,
    private val initialPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val estimatorConfig: EstimatorConfig = EstimatorConfig(),
    var aimConfig: AimConfig = AimConfig(),
    private val logger: Logger = LogcatLogger(LOG_TAG)
) : AutoCloseable {

    companion object {
        private const val LOG_TAG = "AutoAimGTSAM"
    }

    // Track whether native library loaded successfully
    private var nativeLibraryLoaded = false

    data class LandmarkSpec(
        val position: Vector3d,
        val roll: Double = 0.0,
        val pitch: Double = 0.0,
        val yaw: Double = 0.0,
        val size: Double = 0.165
    )

    data class EstimatorConfig(
        val priorSigmaXY: Double = 0.05,
        val priorSigmaTheta: Double = 0.02,
        val odomSigmaXY: Double = 0.02,
        val odomSigmaTheta: Double = 0.01,
        val defaultPixelSigma: Double = 1.0,
        val relinearizeThreshold: Double = 0.01,
        val relinearizeSkip: Int = 1,
        val enablePartialRelinearization: Boolean = true,
        val compactOdometry: Boolean = true,
        val enableRobustTagLoss: Boolean = false,
        val robustTagLoss: Int = 0,
        val robustTagLossK: Double = 1.5,
        val enableTagGating: Boolean = true,
        val minTagAreaPx: Double = 50.0,
        val maxTagViewAngleDeg: Double = 60.0,
        val enableCheiralityCheck: Boolean = false,
        val cheiralitySigma: Double = 0.1,
        val minTagZDistance: Double = 0.5,
        val enablePostProcess: Boolean = true,
        val postProcessVisionGapS: Double = 0.4,
        val postProcessSettleS: Double = 2.0,
        val postProcessSettleUpdates: Int = 3,
        val fx: Double = 1000.0,
        val fy: Double = 1000.0,
        val cx: Double = 320.0,
        val cy: Double = 240.0,
        val k1: Double = 0.0,
        val k2: Double = 0.0,
        val k3: Double = 0.0,
        val p1: Double = 0.0,
        val p2: Double = 0.0,
        val cameraOffsetX: Double = 0.0,
        val cameraOffsetY: Double = 0.0,
        val cameraOffsetZ: Double = 0.5,
        val cameraRoll: Double = -PI / 2.0,
        val cameraPitch: Double = 0.0,
        val cameraYaw: Double = -PI / 2.0,
        val pixelSigmaAngleK: Double = 2.0,
        val enableSpatialCorrelation: Boolean = true,
        val correlationDistanceM: Double = 0.3,
        val correlationDownweightFactor: Double = 2.0,
        val correlationHistorySize: Int = 100,
        val enableBiasCorrection: Boolean = true,
        val radialBiasK: Double = 0.01,
        val enableMultiHypothesisInit: Boolean = true,
        val multiHypothesisThetaThreshold: Double = 1.0,
        val enableHeadingFlipRecovery: Boolean = true,
        val headingFlipMinTags: Int = 1
    )

    // ===== Native Library Loading =====

    init {
        try {
            val appInfo = AppUtil.getDefContext().applicationInfo
            logger.log(
                LogLevel.INFO,
                "JNI load: classLoader=${javaClass.classLoader} nativeLibraryDir=${appInfo.nativeLibraryDir} " +
                    "sourceDir=${appInfo.sourceDir}"
            )
            val resolvedPath = NativeLibraryLoader.resolveLatestHashedLibrary("decode_estimator_jni")
            if (resolvedPath != null) {
                logger.log(LogLevel.INFO, "Loading native library from: $resolvedPath")
                System.load(resolvedPath)
                logger.log(LogLevel.INFO, "Loaded decode_estimator_jni (resolved path)")
                nativeLibraryLoaded = true
            } else {
                logger.log(LogLevel.INFO, "No hashed library found, using System.loadLibrary")
                System.loadLibrary("decode_estimator_jni")
                logger.log(LogLevel.INFO, "Loaded decode_estimator_jni (System.loadLibrary)")
                nativeLibraryLoaded = true
            }
        } catch (e: UnsatisfiedLinkError) {
            logger.log(LogLevel.ERROR, "Failed to load decode_estimator library: ${e.message}")
            logger.log(LogLevel.ERROR, "JNI load failed: classLoader=${javaClass.classLoader}")
            logger.log(LogLevel.WARN, "GTSAM fusion will not be available - will use odometry fallback in sim mode")
            nativeLibraryLoaded = false
        }
    }

    // ===== Thread Management =====

    private val fusionWorker = FusionWorker(estimatorConfig, initialPose)
    private val turretTargeting = TurretTargeting(goalPosition)

    // ===== Thread-Safe State =====

    @Volatile private var fusedPoseInternal: Pose2d = initialPose
    private val enqueuedOdomCount = AtomicLong(0)
    private val enqueuedVisionCount = AtomicLong(0)

    // ===== Main Thread State =====

    private var lastRobotPose: Pose2d? = null
    private var lastTurretAngle: Double = 0.0
    private var lastUpdateTimeMs: Long = 0
    private var lastRobotPoseForUncertainty: Pose2d? = null
    private var lastDiagLogMs: Long = 0
    
    // ===== Debug Logging =====
    private var debugLogger: GTSAMDebugLogger? = null

    // ===== Public Interface =====

    var enabled: Boolean = false

    var hasVisionTarget: Boolean = false
        private set

    var targetTx: Double = 0.0
        private set

    var targetTy: Double = 0.0
        private set

    var trackedTagId: Int = -1
        private set

    var detectedTagCount: Int = 0
        private set

    var rawTxDegrees: Double = 0.0
        private set

    var rawTyDegrees: Double = 0.0
        private set

    var lastResultValid: Boolean = false
        private set

    var lastDetectionTimeMs: Long = 0
        private set

    var uncertainty: Double = 0.0
        private set

    var fusedPose: Pose2d = initialPose
        private set

    val hasTarget: Boolean
        get() {
            val effectivelyInitialized = fusionWorker.isInitialized ||
                (SigmaOpMode.SIM && !nativeLibraryLoaded)
            return turretTargeting.hasTarget(
                gtsamInitialized = effectivelyInitialized,
                lastDetectionTimeMs = lastDetectionTimeMs,
                predictionTimeoutMs = aimConfig.predictionTimeoutMs
            )
        }

    val usingPrediction: Boolean
        get() = turretTargeting.usingPrediction(
            hasVisionTarget = hasVisionTarget,
            uncertainty = uncertainty,
            maxAcceptableUncertainty = aimConfig.maxAcceptableUncertainty
        )

    var enableDebugLogging: Boolean = true

    private var hasLoggedLandmarks: Boolean = false
    
    fun enableDebugLogging(recordingId: String = "gtsam_debug",
                           savePath: String = "/sdcard/FIRST/kotlin_gtsam.rrd") {
        if (debugLogger == null) {
            debugLogger = GTSAMDebugLogger(recordingId, savePath).apply { 
                enable() 
                logLandmarks(landmarkPositions)
            }
        }
    }

    // ===== Main Update Method =====

    fun update(robotPose: Pose2d, turretAngle: Double, visionResult: VisionResult?) {
        val currentTime = System.currentTimeMillis()
        val dt = if (lastUpdateTimeMs > 0) (currentTime - lastUpdateTimeMs) / 1000.0 else 0.02

        // Check if we should use odometry fallback (sim mode + no native library)
        val useOdometryFallback = SigmaOpMode.SIM && !nativeLibraryLoaded

        if (useOdometryFallback) {
            // Fallback mode: use odometry directly instead of GTSAM fusion
            fusedPoseInternal = robotPose
            fusedPose = robotPose
            logger.log(LogLevel.DEBUG, "Using odometry fallback (native library not available in sim mode)")
        } else {
            // Normal mode: use GTSAM fusion
            if (!fusionWorker.isInitialized && !fusionWorker.hasPendingInit()) {
                logger.log(LogLevel.INFO, "Queueing GTSAM initialization")
                fusionWorker.requestInitialize(robotPose, landmarkPositions)
            }

            // Log raw odometry
            val velocity = if (lastRobotPose != null) {
                val dPose = robotPose.minus(lastRobotPose!!)
                Pose2d(dPose.v.div(dt), dPose.rot / dt)
            } else {
                Pose2d(0.0, 0.0, 0.0)
            }
            debugLogger?.logRawOdometry(robotPose, velocity, currentTime / 1000.0)

            if (lastRobotPose != null) {
                processOdometryDelta(robotPose)
            }

            fusionWorker.tick()
            fusedPoseInternal = fusionWorker.fusedPose
            fusedPose = fusedPoseInternal
        }

        updateVision(visionResult, turretAngle)
        
        debugLogger?.logFusedPose(fusedPose)
        
        // Log frames
        debugLogger?.logCoordinateFrames(
            fusedPose,
            turretAngle, 
            Vector3d(estimatorConfig.cameraOffsetX, estimatorConfig.cameraOffsetY, estimatorConfig.cameraOffsetZ),
            estimatorConfig.cameraRoll, estimatorConfig.cameraPitch, estimatorConfig.cameraYaw
        )
        
        // Log predicted corners for visible tags (only in GTSAM mode)
        if (!useOdometryFallback && visionResult?.frame?.observations != null) {
            for (obs in visionResult.frame.observations) {
                val predicted = fusionWorker.getPredictedCorners(obs.tagId)
                if (predicted.isNotEmpty()) {
                    debugLogger?.logPredictedCorners(obs.tagId, predicted)

                    // Calculate reprojection error
                    var totalError = 0.0
                    for (i in 0 until 4) {
                        val px = obs.corners[i * 2]
                        val py = obs.corners[i * 2 + 1]
                        val dx = px - predicted[i].x
                        val dy = py - predicted[i].y
                        totalError += Math.sqrt(dx * dx + dy * dy)
                    }
                    debugLogger?.logReprojectionError(obs.tagId, totalError / 4.0)
                }
            }
        }

        updateUncertainty(robotPose, turretAngle, dt)

        lastRobotPose = robotPose
        lastTurretAngle = turretAngle
        lastUpdateTimeMs = currentTime

        // Poll for errors only in GTSAM mode
        if (!useOdometryFallback) {
            fusionWorker.pollError()?.let { error ->
                logger.log(LogLevel.ERROR, error)
            }
        }

        if (enableDebugLogging && currentTime - lastDiagLogMs > 1000) {
            if (useOdometryFallback) {
                logger.log(
                    LogLevel.INFO,
                    "Diag (odometry fallback): odom=N/A vision=${enqueuedVisionCount.get()} " +
                        "hasVision=$hasVisionTarget uncertainty=%.2f".format(uncertainty)
                )
            } else {
                val memorySummary = formatMemoryUsage(fusionWorker.lastMemoryUsage)
                val diagnosticsSummary = formatDiagnosticsSnapshot(fusionWorker.lastDiagnosticsSnapshot)
                logger.log(
                    LogLevel.INFO,
                    "Diag: odom=${enqueuedOdomCount.get()} vision=${enqueuedVisionCount.get()} " +
                        "hasVision=$hasVisionTarget uncertainty=%.2f solveMs=%.3f %s %s".format(
                            uncertainty,
                            fusionWorker.lastSolveTimeMs,
                            memorySummary,
                            diagnosticsSummary
                        )
                )
            }
            lastDiagLogMs = currentTime
        }
    }

    // ===== Odometry Processing =====

    private fun processOdometryDelta(currentPose: Pose2d) {
        val prevPose = lastRobotPose ?: return
        val timestampSeconds = System.currentTimeMillis() / 1000.0

        val dxGlobal = currentPose.v.x - prevPose.v.x
        val dyGlobal = currentPose.v.y - prevPose.v.y
        val dtheta = normalizeAngle(currentPose.rot - prevPose.rot)

        val cosPrev = cos(prevPose.rot)
        val sinPrev = sin(prevPose.rot)
        val dxLocal = cosPrev * dxGlobal + sinPrev * dyGlobal
        val dyLocal = -sinPrev * dxGlobal + cosPrev * dyGlobal

        fusionWorker.enqueueOdometry(dxLocal, dyLocal, dtheta, timestampSeconds)
        enqueuedOdomCount.incrementAndGet()
        
        debugLogger?.logOdometryDelta(dxLocal, dyLocal, dtheta, dxGlobal, dyGlobal, prevPose.rot)
    }

    // ===== Vision Processing =====

    private fun updateVision(visionResult: VisionResult?, turretAngle: Double) {
        if (visionResult == null) {
            clearVisionState()
            return
        }

        val status = visionResult.status
        lastResultValid = status.lastResultValid
        detectedTagCount = status.detectedTagCount
        hasVisionTarget = status.hasVisionTarget
        trackedTagId = status.trackedTagId
        rawTxDegrees = status.rawTxDegrees
        rawTyDegrees = status.rawTyDegrees
        lastDetectionTimeMs = status.lastDetectionTimeMs

        if (!hasVisionTarget) {
            targetTx = 0.0
            targetTy = 0.0
            return
        }

        targetTx = Math.toRadians(rawTxDegrees) + aimConfig.cameraMountingOffsetYaw
        targetTy = Math.toRadians(rawTyDegrees)

        val frame = visionResult.frame ?: return
        if (!fusionWorker.isInitialized) {
            return
        }

        for (observation in frame.observations) {
            fusionWorker.enqueueVision(
                tagId = observation.tagId,
                corners = observation.corners,
                pixelSigma = aimConfig.visionPixelSigma,
                timestampSeconds = frame.timestampSeconds,
                turretYawRad = turretAngle
            )
            enqueuedVisionCount.incrementAndGet()

            val corners = observation.corners.toList().chunked(2).map { Vector2d(it[0],it[1]) }
            debugLogger?.logTagDetection(observation.tagId, corners, turretAngle)
        }
    }

    private fun clearVisionState() {
        hasVisionTarget = false
        lastResultValid = false
        detectedTagCount = 0
        trackedTagId = -1
        rawTxDegrees = 0.0
        rawTyDegrees = 0.0
        targetTx = 0.0
        targetTy = 0.0
        lastDetectionTimeMs = 0
    }

    // ===== Uncertainty Estimation =====

    private fun updateUncertainty(robotPose: Pose2d, turretAngle: Double, dt: Double) {
        if (lastRobotPoseForUncertainty != null && dt > 0) {
            val robotAngVel = normalizeAngle(robotPose.rot - lastRobotPoseForUncertainty!!.rot) / dt
            val turretAngVel = normalizeAngle(turretAngle - lastTurretAngle) / dt

            val fieldRelTurretAngVel = abs(robotAngVel + turretAngVel)
            val motionUncertainty =
                (fieldRelTurretAngVel * aimConfig.turretRotationUncertaintyGain).coerceIn(0.0, 1.0)

            uncertainty = if (motionUncertainty > uncertainty) {
                motionUncertainty
            } else {
                val decayFactor = aimConfig.uncertaintyDecayRate.pow(dt)
                uncertainty * decayFactor + motionUncertainty * (1 - decayFactor)
            }
        }

        lastRobotPoseForUncertainty = robotPose
    }

    // ===== Public API Methods =====

    fun getTurretYawAdjustment(): Double {
        return turretTargeting.turretAdjustment(enabled, hasVisionTarget, targetTx, aimConfig)
    }

    fun getTargetTurretAngle(): Double? {
        if (!enabled || !hasTarget) {
            return null
        }
        return normalizeAngle(turretTargeting.computeAngles(fusedPose).robotAngle)
    }

    fun getTargetFieldAngle(): Double? {
        if (!enabled || !hasTarget) {
            return null
        }
        return normalizeAngle(turretTargeting.computeAngles(fusedPose).fieldAngle)
    }

    fun resetTarget(newPose: Pose2d? = null) {
        clearVisionState()
        uncertainty = 0.0
        newPose?.let {
            fusionWorker.requestInitialize(it, landmarkPositions)
        }
    }

    // ===== Utility Functions =====

    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle
        while (normalized > PI) normalized -= 2 * PI
        while (normalized < -PI) normalized += 2 * PI
        return normalized
    }

    private fun formatMemoryUsage(usage: FusionWorker.MemoryUsage?): String {
        if (usage == null || !usage.valid) {
            return "mem=unavailable"
        }
        val mb = 1024.0 * 1024.0
        return "mem=virt=%.1fMB rss=%.1fMB data=%.1fMB sh=%.1fMB".format(
            usage.virtualBytes / mb,
            usage.residentBytes / mb,
            usage.dataBytes / mb,
            usage.sharedBytes / mb
        )
    }

    private fun formatDiagnosticsSnapshot(snapshot: FusionWorker.DiagnosticsSnapshot?): String {
        if (snapshot == null) {
            return "diag=unavailable"
        }
        return ("diag=tags=%d graph=%d values=%d pose=%d horizon=%d last=%.2fms avg=%.2fms " +
            "reset=%.2fms cov=%.2fms resetPose=%d").format(
                snapshot.pendingTags,
                snapshot.pendingGraphFactors,
                snapshot.pendingValues,
                snapshot.currentPoseIndex,
                snapshot.horizonCapacity,
                snapshot.lastSolveMs.toDouble(),
                snapshot.avgSolveMs.toDouble(),
                snapshot.lastHorizonResetMs.toDouble(),
                snapshot.lastHorizonCovMs.toDouble(),
                snapshot.lastHorizonResetPoseIndex
            )
    }

    // ===== Landmark Corner Logging =====

    /**
     * Logs all landmark corner positions after initialization.
     * Corner positions are retrieved from the native pose estimator and logged in world frame.
     */
    fun logLandmarkCorners() {
        try {
            val handle = fusionWorker.getEstimatorHandle()
            if (handle == 0L) {
                logger.log(LogLevel.WARN, "Estimator handle is invalid, skipping landmark corner logging")
                return
            }

            logger.log(LogLevel.INFO, "=== Landmark Corner Positions (World Frame) ===")

            try {
                val allCorners = PoseEstimatorBridge.nativeGetAllLandmarkCorners(handle)
                if (allCorners.isEmpty()) {
                    logger.log(LogLevel.INFO, "No landmark corners available")
                    return
                }

                // Each landmark has 13 values: [tagId, corner1(x,y,z), corner2(x,y,z), corner3(x,y,z), corner4(x,y,z)]
                val landmarkCount = allCorners.size / 13
                for (i in 0 until landmarkCount) {
                    val baseIdx = i * 13
                    val tagId = allCorners[baseIdx].toInt()

                    val corner1 = Triple(allCorners[baseIdx + 1], allCorners[baseIdx + 2], allCorners[baseIdx + 3])
                    val corner2 = Triple(allCorners[baseIdx + 4], allCorners[baseIdx + 5], allCorners[baseIdx + 6])
                    val corner3 = Triple(allCorners[baseIdx + 7], allCorners[baseIdx + 8], allCorners[baseIdx + 9])
                    val corner4 = Triple(allCorners[baseIdx + 10], allCorners[baseIdx + 11], allCorners[baseIdx + 12])

                    logger.log(
                        LogLevel.INFO,
                        "Tag %d: TL=(%.3f, %.3f, %.3f) TR=(%.3f, %.3f, %.3f) BR=(%.3f, %.3f, %.3f) BL=(%.3f, %.3f, %.3f)".format(
                            tagId,
                            corner1.first, corner1.second, corner1.third,
                            corner2.first, corner2.second, corner2.third,
                            corner3.first, corner3.second, corner3.third,
                            corner4.first, corner4.second, corner4.third
                        )
                    )
                }
            } catch (e: Exception) {
                logger.log(LogLevel.ERROR, "Failed to log all landmark corners: ${e.message}")
            }

            logger.log(LogLevel.INFO, "=== End Landmark Corner Positions ===")
        } catch (e: Exception) {
            logger.log(LogLevel.ERROR, "Error during landmark corner logging: ${e.message}")
        }
    }

    /**
     * Logs corner positions for a specific landmark by tag ID.
     * @param tagId The AprilTag ID of the landmark to log
     */
    fun logLandmarkCorners(tagId: Int) {
        try {
            val handle = fusionWorker.getEstimatorHandle()
            if (handle == 0L) {
                logger.log(LogLevel.WARN, "Estimator handle is invalid, skipping landmark corner logging")
                return
            }

            try {
                val corners = PoseEstimatorBridge.nativeGetLandmarkCorners(handle, tagId)
                if (corners.size < 12) {
                    logger.log(LogLevel.WARN, "Tag $tagId: Invalid corner data (expected 12 values, got ${corners.size})")
                    return
                }

                // Each landmark has 12 values: [corner1(x,y,z), corner2(x,y,z), corner3(x,y,z), corner4(x,y,z)]
                val corner1 = Triple(corners[0], corners[1], corners[2])
                val corner2 = Triple(corners[3], corners[4], corners[5])
                val corner3 = Triple(corners[6], corners[7], corners[8])
                val corner4 = Triple(corners[9], corners[10], corners[11])

                logger.log(
                    LogLevel.INFO,
                    "Tag %d corners: TL=(%.3f, %.3f, %.3f) TR=(%.3f, %.3f, %.3f) BR=(%.3f, %.3f, %.3f) BL=(%.3f, %.3f, %.3f)".format(
                        tagId,
                        corner1.first, corner1.second, corner1.third,
                        corner2.first, corner2.second, corner2.third,
                        corner3.first, corner3.second, corner3.third,
                        corner4.first, corner4.second, corner4.third
                    )
                )
            } catch (e: Exception) {
                logger.log(LogLevel.ERROR, "Failed to log corners for tag $tagId: ${e.message}")
            }
        } catch (e: Exception) {
            logger.log(LogLevel.ERROR, "Error during landmark corner logging for tag $tagId: ${e.message}")
        }
    }

    /**
     * Logs camera unit vectors (position and orientation) in world frame.
     * Includes camera position, right vector, up vector, and forward vector.
     */
    fun logCameraUnitVectors() {
        try {
            val handle = fusionWorker.getEstimatorHandle()
            if (handle == 0L) {
                logger.log(LogLevel.WARN, "Estimator handle is invalid, skipping camera vector logging")
                return
            }

            try {
                val vectors = PoseEstimatorBridge.nativeGetCameraUnitVectors(handle)
                if (vectors.size < 12) {
                    logger.log(LogLevel.WARN, "Invalid camera vector data (expected 12 values, got ${vectors.size})")
                    return
                }

                // Array structure: [pos_x, pos_y, pos_z, right_x, right_y, right_z, up_x, up_y, up_z, fwd_x, fwd_y, fwd_z]
                val posX = vectors[0]
                val posY = vectors[1]
                val posZ = vectors[2]

                val rightX = vectors[3]
                val rightY = vectors[4]
                val rightZ = vectors[5]

                val upX = vectors[6]
                val upY = vectors[7]
                val upZ = vectors[8]

                val fwdX = vectors[9]
                val fwdY = vectors[10]
                val fwdZ = vectors[11]

                logger.log(
                    LogLevel.INFO,
                    "Camera vectors: pos=(%.3f, %.3f, %.3f) right=(%.3f, %.3f, %.3f) up=(%.3f, %.3f, %.3f) fwd=(%.3f, %.3f, %.3f)".format(
                        posX, posY, posZ,
                        rightX, rightY, rightZ,
                        upX, upY, upZ,
                        fwdX, fwdY, fwdZ
                    )
                )
            } catch (e: Exception) {
                logger.log(LogLevel.ERROR, "Failed to log camera vectors: ${e.message}")
            }
        } catch (e: Exception) {
            logger.log(LogLevel.ERROR, "Error during camera vector logging: ${e.message}")
        }
    }

    // ===== Cleanup =====

    override fun close() {
        logger.log(LogLevel.INFO, "Closing AutoAimGTSAM")
        fusionWorker.close()
        debugLogger?.close()
    }
}
