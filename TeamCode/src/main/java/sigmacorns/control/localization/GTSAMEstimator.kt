package sigmacorns.control.localization

import org.joml.Vector2d
import org.joml.Vector3d
import dev.frozenmilk.sinister.util.NativeLibraryLoader
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import sigmacorns.control.aim.LogLevel
import sigmacorns.control.aim.LogcatLogger
import sigmacorns.control.aim.Logger
import sigmacorns.math.Pose2d
import sigmacorns.math.normalizeAngle
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * GTSAM-based sensor fusion.
 *
 * Fuses wheel odometry with AprilTag vision measurements using GTSAM factor graph
 * optimization for robust localization. Runs GTSAM on a dedicated
 * worker thread to avoid blocking the main control loop.
 */
class GTSAMEstimator(
    private val landmarkPositions: Map<Int, LandmarkSpec>,
    private val initialPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val logger: Logger = LogcatLogger(LOG_TAG)
) : AutoCloseable {

    companion object {
        private const val LOG_TAG = "AutoAimGTSAM"
    }

    data class LandmarkSpec(
        val position: Vector3d,
        val roll: Double = 0.0,
        val pitch: Double = 0.0,
        val yaw: Double = 0.0,
        val size: Double = 0.165
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
            } else {
                logger.log(LogLevel.INFO, "No hashed library found, using System.loadLibrary")
                System.loadLibrary("decode_estimator_jni")
                logger.log(LogLevel.INFO, "Loaded decode_estimator_jni (System.loadLibrary)")
            }
        } catch (_: Throwable) {
            // FTC Android path failed (e.g. running in host test environment), fall back
            try {
                System.loadLibrary("decode_estimator_jni")
            } catch (e: UnsatisfiedLinkError) {
                logger.log(LogLevel.ERROR, "Failed to load decode_estimator library: ${e.message}")
            }
        }
    }

    // ===== Thread Management =====

    private val fusionWorker = FusionWorker(initialPose)

    // ===== Thread-Safe State =====
    @Volatile private var fusedPoseInternal: Pose2d = initialPose
    private val enqueuedOdomCount = AtomicLong(0)
    private val enqueuedVisionCount = AtomicLong(0)

    // ===== Main Thread State =====

    private var lastRobotPose: Pose2d? = null
    private var lastUpdateTimeMs: Long = 0
    private var lastDiagLogMs: Long = 0
    
    // ===== Debug Logging =====
    private var debugLogger: GTSAMDebugLogger? = null

    // ===== Public Interface =====

    var enabled: Boolean = false

    var hasVisionTarget: Boolean = false
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
    var fusedPose: Pose2d = initialPose
        private set

    var enableDebugLogging: Boolean = true

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

        updateVision(visionResult, turretAngle)

        fusionWorker.tick()
        fusedPoseInternal = fusionWorker.fusedPose
        fusedPose = fusedPoseInternal
        
        debugLogger?.logFusedPose(fusedPose)
        
        // Log frames
        debugLogger?.logCoordinateFrames(
            fusedPose,
            turretAngle, 
            Vector3d(EstimatorConfig    .cameraOffsetX, EstimatorConfig.cameraOffsetY, EstimatorConfig.cameraOffsetZ),
            EstimatorConfig.cameraRoll, EstimatorConfig.cameraPitch, EstimatorConfig.cameraYaw
        )
        
        // Log predicted corners for visible tags
        if (visionResult?.frame?.observations != null) {
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
                        totalError += sqrt(dx * dx + dy * dy)
                    }
                    debugLogger?.logReprojectionError(obs.tagId, totalError / 4.0)
                }
            }
        }

        lastRobotPose = robotPose
        lastUpdateTimeMs = currentTime

        fusionWorker.pollError()?.let { error ->
            logger.log(LogLevel.ERROR, error)
        }

        if (enableDebugLogging && currentTime - lastDiagLogMs > 1000) {
            val memorySummary = formatMemoryUsage(fusionWorker.lastMemoryUsage)
            val diagnosticsSummary = formatDiagnosticsSnapshot(fusionWorker.lastDiagnosticsSnapshot)
            logger.log(
                LogLevel.INFO,
                "Diag: odom=${enqueuedOdomCount.get()} vision=${enqueuedVisionCount.get()} " +
                    "hasVision=$hasVisionTarget solveMs=%.3f %s %s".format(
                        fusionWorker.lastSolveTimeMs,
                        memorySummary,
                        diagnosticsSummary
                    )
            )
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

        val frame = visionResult.frame ?: return
        if (!fusionWorker.isInitialized) {
            return
        }

        for (observation in frame.observations) {
            fusionWorker.enqueueVision(
                tagId = observation.tagId,
                corners = observation.corners,
                pixelSigma = EstimatorConfig.defaultPixelSigma,
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
        lastDetectionTimeMs = 0
    }

    // ===== Utility Functions =====
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
                snapshot.lastSolveMs,
                snapshot.avgSolveMs,
                snapshot.lastHorizonResetMs,
                snapshot.lastHorizonCovMs,
                snapshot.lastHorizonResetPoseIndex
            )
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
