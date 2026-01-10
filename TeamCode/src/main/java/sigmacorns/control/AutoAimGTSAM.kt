package sigmacorns.control

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import android.util.Log
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.math.Pose2d
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.*

/**
 * Auto-aiming system with GTSAM-based sensor fusion.
 *
 * Fuses wheel odometry with AprilTag vision measurements using GTSAM factor graph
 * optimization for robust localization and targeting. Runs GTSAM on a dedicated
 * worker thread to avoid blocking the main control loop.
 *
 * Coordinate Frames:
 * - Field frame: Fixed world frame (origin at field corner)
 * - Robot frame: Robot-centric frame (x forward, y left, z up)
 * - Turret frame: Turret-centric frame (rotates with turret)
 * - Camera frame: Limelight sensor frame (mounted on turret)
 *
 * @param limelight The Limelight3A camera instance for AprilTag detection
 * @param landmarkPositions Map of AprilTag IDs to their field positions (meters)
 * @param goalPosition Known goal position in field frame for aiming
 * @param initialPose Initial robot pose estimate for GTSAM initialization
 */
class AutoAimGTSAM(
    private val limelight: Limelight3A?,
    private val landmarkPositions: Map<Int, LandmarkSpec>,
    private val goalPosition: Vector2d,
    private val initialPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val estimatorConfig: EstimatorConfig = EstimatorConfig()
) : AutoCloseable {

    companion object {
        private const val LOG_TAG = "AutoAimGTSAM"
    }

    var logSink: ((String) -> Unit)? = null
    var enableDebugLogging: Boolean = true
    private val pendingLogs = ConcurrentLinkedQueue<String>()
    private val pendingLogCount = java.util.concurrent.atomic.AtomicInteger(0)
    private val maxPendingLogs = 200
    private val maxFlushPerUpdate = 50

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
        val cameraRoll: Double = 0.0,
        val cameraPitch: Double = 0.0,
        val cameraYaw: Double = 0.0
    )

    // ===== Native Library Loading =====

    init {
        try {
            System.loadLibrary("decode_estimator_jni")
            logInfo("Loaded decode_estimator_jni")
        } catch (e: UnsatisfiedLinkError) {
            logError("Failed to load decode_estimator library: ${e.message}", e)
        }
    }

    // ===== JNI Bridge to PoseEstimator =====
    // IMPORTANT: ALL native methods called ONLY from GTSAM worker thread

    // ===== Measurement Types for Thread Communication =====

    private sealed class Measurement

    private data class OdometryMeasurement(
        val dx: Double,
        val dy: Double,
        val dtheta: Double,
        val timestampSeconds: Double
    ) : Measurement()

    private data class VisionMeasurement(
        val tagId: Int,
        val corners: DoubleArray,
        val pixelSigma: Double,
        val timestampSeconds: Double,
        val turretYawRad: Double
    ) : Measurement()

    private data class InitializeCommand(
        val pose: Pose2d,
        val landmarks: Map<Int, LandmarkSpec>
    ) : Measurement()

    private object ShutdownCommand : Measurement()

    // ===== Thread Management =====

    private val measurementQueue = LinkedBlockingQueue<Measurement>()
    private val running = AtomicBoolean(false)
    private val gtsamThread: Thread

    // ===== Thread-Safe State =====

    @Volatile private var fusedPoseInternal: Pose2d = initialPose
    @Volatile private var gtsamInitialized = false

    // ===== Main Thread State =====

    private var lastRobotPose: Pose2d? = null
    private var lastTurretAngle: Double = 0.0
    private var lastUpdateTimeMs: Long = 0
    private var lastRobotPoseForUncertainty: Pose2d? = null
    private var lastDiagLogMs: Long = 0
    private var enqueuedOdomCount: Long = 0
    private var enqueuedVisionCount: Long = 0

    // ===== Public Interface (matching AutoAim) =====

    var enabled: Boolean = false

    var hasVisionTarget: Boolean = false
        private set

    var hasTarget: Boolean = false
        private set

    var targetTx: Double = 0.0
        private set

    var targetTy: Double = 0.0
        private set

    var targetDistance: Double = 0.0
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

    var targetFieldAngle: Double = 0.0
        private set

    var targetRobotAngle: Double = 0.0
        private set

    var uncertainty: Double = 0.0
        private set

    var usingPrediction: Boolean = false
        private set

    var fusedPose: Pose2d = initialPose
        private set

    // ===== Configuration Parameters =====

    var cameraMountingOffsetYaw: Double = 0.0
    var txSignMultiplier: Double = -1.0
    var txDeadband: Double = 0.01  // ~0.57 degrees
    var maxAcceptableUncertainty: Double = Double.MAX_VALUE
    var predictionTimeoutMs: Long = 2000
    var visionPixelSigma: Double = estimatorConfig.defaultPixelSigma  // pixels
    var maxPoseDivergence: Double = 2.0  // meters

    var turretRotationUncertaintyGain: Double = 1.5
    var uncertaintyDecayRate: Double = 0.01


    // ===== GTSAM Worker Thread =====

    private val gtsamWorker = Runnable {
        running.set(true)
        var gtsamHandle: Long = 0L
        var initialized = false

        try {
            logWorkerInfo("GTSAM worker started")
            while (running.get()) {
                val measurement = measurementQueue.poll(100, TimeUnit.MILLISECONDS) ?: continue

                when (measurement) {
                    is InitializeCommand -> {
                        // Clean up existing handle if reinitializing
                        if (gtsamHandle != 0L) {
                            PoseEstimatorBridge.nativeDestroy(gtsamHandle)
                        }

                        val config = estimatorConfig
                        gtsamHandle = PoseEstimatorBridge.nativeCreateWithConfig(
                            config.priorSigmaXY,
                            config.priorSigmaTheta,
                            config.odomSigmaXY,
                            config.odomSigmaTheta,
                            config.defaultPixelSigma,
                            config.relinearizeThreshold,
                            config.relinearizeSkip,
                            config.enablePartialRelinearization,
                            config.compactOdometry,
                            config.enableRobustTagLoss,
                            config.robustTagLoss,
                            config.robustTagLossK,
                            config.enableTagGating,
                            config.minTagAreaPx,
                            config.maxTagViewAngleDeg,
                            config.enablePostProcess,
                            config.postProcessVisionGapS,
                            config.postProcessSettleS,
                            config.postProcessSettleUpdates,
                            config.fx,
                            config.fy,
                            config.cx,
                            config.cy,
                            config.k1,
                            config.k2,
                            config.k3,
                            config.p1,
                            config.p2,
                            config.cameraOffsetX,
                            config.cameraOffsetY,
                            config.cameraOffsetZ,
                            config.cameraRoll,
                            config.cameraPitch,
                            config.cameraYaw
                        )
                        if (gtsamHandle == 0L) {
                            logWorkerError("Failed to create GTSAM handle")
                            continue
                        }

                        // Convert landmarks to parallel arrays for JNI
                        val ids = IntArray(measurement.landmarks.size)
                        val xs = DoubleArray(measurement.landmarks.size)
                        val ys = DoubleArray(measurement.landmarks.size)
                        val zs = DoubleArray(measurement.landmarks.size)
                        val rolls = DoubleArray(measurement.landmarks.size)
                        val pitches = DoubleArray(measurement.landmarks.size)
                        val yaws = DoubleArray(measurement.landmarks.size)
                        val sizes = DoubleArray(measurement.landmarks.size)
                        measurement.landmarks.entries.forEachIndexed { i, (id, pos) ->
                            ids[i] = id
                            xs[i] = pos.position.x
                            ys[i] = pos.position.y
                            zs[i] = pos.position.z
                            rolls[i] = pos.roll
                            pitches[i] = pos.pitch
                            yaws[i] = pos.yaw
                            sizes[i] = pos.size
                        }

                        PoseEstimatorBridge.nativeInitialize(
                            gtsamHandle,
                            ids,
                            xs,
                            ys,
                            zs,
                            rolls,
                            pitches,
                            yaws,
                            sizes,
                            measurement.pose.v.x,
                            measurement.pose.v.y,
                            measurement.pose.rot
                        )
                        initialized = true

                        fusedPoseInternal = measurement.pose
                        gtsamInitialized = true
                        logWorkerInfo(
                            "GTSAM initialized with ${measurement.landmarks.size} landmarks " +
                                "at (%.2f, %.2f, %.2f)".format(
                                    measurement.pose.v.x,
                                    measurement.pose.v.y,
                                    measurement.pose.rot
                                )
                        )
                    }

                    is OdometryMeasurement -> {
                        if (initialized) {
                            PoseEstimatorBridge.nativeProcessOdometry(
                                gtsamHandle,
                                measurement.dx,
                                measurement.dy,
                                measurement.dtheta,
                                measurement.timestampSeconds
                            )
                            optimizeAndUpdatePose(gtsamHandle)
                        }
                    }

                    is VisionMeasurement -> {
                        if (initialized) {
                            PoseEstimatorBridge.nativeAddTagMeasurement(
                                gtsamHandle,
                                measurement.tagId,
                                measurement.corners,
                                measurement.pixelSigma,
                                measurement.timestampSeconds,
                                measurement.turretYawRad
                            )
                            optimizeAndUpdatePose(gtsamHandle)
                        }
                    }

                    is ShutdownCommand -> {
                        logWorkerInfo("GTSAM shutdown requested")
                        break
                    }
                }
            }
        } catch (e: Exception) {
            logWorkerError("Worker thread error: ${e.message}", e)
        } finally {
            if (gtsamHandle != 0L) {
                try {
                    PoseEstimatorBridge.nativeDestroy(gtsamHandle)
                } catch (e: Exception) {
                    logWorkerError("Error destroying GTSAM handle: ${e.message}", e)
                }
            }
            logWorkerInfo("GTSAM worker stopped")
        }
    }

    // ===== Initialization =====

    init {
        gtsamThread = Thread(gtsamWorker, "GTSAM-Worker")
        gtsamThread.start()
    }

    private fun optimizeAndUpdatePose(handle: Long) {
        try {
            val startNs = System.nanoTime()
            PoseEstimatorBridge.nativeUpdate(handle)
            val elapsedMs = (System.nanoTime() - startNs) / 1_000_000.0
            logWorkerInfo("Solver update took %.3f ms".format(elapsedMs))
            val estimate = PoseEstimatorBridge.nativeGetCurrentEstimate(handle)
            if (estimate.size >= 3) {
                fusedPoseInternal = Pose2d(estimate[0], estimate[1], estimate[2])
            }
        } catch (e: Exception) {
            logWorkerError("Error in optimize: ${e.message}", e)
        }
    }

    // ===== Main Update Method =====

    fun update(robotPose: Pose2d, turretAngle: Double) {
        flushLogs()
        val currentTime = System.currentTimeMillis()
        val dt = if (lastUpdateTimeMs > 0) (currentTime - lastUpdateTimeMs) / 1000.0 else 0.02

        // 1. Initialize GTSAM on first call
        if (!gtsamInitialized && measurementQueue.find { it is InitializeCommand } == null) {
            logInfo("Queueing GTSAM initialization")
            measurementQueue.offer(InitializeCommand(robotPose, landmarkPositions))
            // Note: we don't block waiting for initialization, it happens asynchronously
        }

        // 2. Process odometry delta
        if (lastRobotPose != null) {
            processOdometryDelta(robotPose)
        }

        // 3. Update vision measurements
        updateVision(turretAngle)

        // 4. Read latest fused pose from GTSAM thread
        fusedPose = fusedPoseInternal

        // 5. Update uncertainty estimate
        updateUncertainty(robotPose, turretAngle, dt)

        // 6. Compute aiming angles to goal using fused pose
        computeAimingAngles()

        // 7. Update state tracking
        lastRobotPose = robotPose
        lastTurretAngle = turretAngle
        lastUpdateTimeMs = currentTime

        if (enableDebugLogging && currentTime - lastDiagLogMs > 1000) {
            logInfo(
                "Diag: odom=$enqueuedOdomCount vision=$enqueuedVisionCount " +
                    "hasVision=$hasVisionTarget uncertainty=%.2f".format(uncertainty)
            )
            lastDiagLogMs = currentTime
        }
    }

    // ===== Odometry Processing =====

    private fun processOdometryDelta(currentPose: Pose2d) {
        val prevPose = lastRobotPose ?: return
        val timestampSeconds = System.currentTimeMillis() / 1000.0

        // Compute delta in global frame
        val dx_global = currentPose.v.x - prevPose.v.x
        val dy_global = currentPose.v.y - prevPose.v.y
        val dtheta = normalizeAngle(currentPose.rot - prevPose.rot)

        // Transform translation delta to previous robot frame
        val cos_prev = cos(prevPose.rot)
        val sin_prev = sin(prevPose.rot)
        val dx_local = cos_prev * dx_global + sin_prev * dy_global
        val dy_local = -sin_prev * dx_global + cos_prev * dy_global

        measurementQueue.offer(OdometryMeasurement(dx_local, dy_local, dtheta, timestampSeconds))
        enqueuedOdomCount++
    }

    // ===== Vision Processing =====

    private fun updateVision(turretAngle: Double) {
        if (limelight == null) {
            hasVisionTarget = false
            lastResultValid = false
            detectedTagCount = 0
            return
        }

        val result: LLResult = limelight.latestResult

        if (!result.isValid) {
            logDebug("Limelight result invalid")
            hasVisionTarget = false
            lastResultValid = false
            detectedTagCount = 0
            return
        }

        lastResultValid = true
        val fiducials: List<FiducialResult> = result.fiducialResults
        detectedTagCount = fiducials.size
        logDebug("Limelight fiducials=${fiducials.size}")

        // Process all visible landmarks
        var sawMatchingTag = false
        var enqueuedAny = false

        for (fiducial in fiducials) {
            val tagId = fiducial.fiducialId

            // Only process tags that are in our landmark map
            if (tagId !in landmarkPositions) continue
            sawMatchingTag = true

            // Store the first valid tag for telemetry
            if (!hasVisionTarget) {
                hasVisionTarget = true
                trackedTagId = tagId
                rawTxDegrees = fiducial.targetXDegrees
                rawTyDegrees = fiducial.targetYDegrees
                targetTx = Math.toRadians(fiducial.targetXDegrees) + cameraMountingOffsetYaw
                targetTy = Math.toRadians(fiducial.targetYDegrees)
                lastDetectionTimeMs = result.controlHubTimeStamp
            }

            val corners = fiducial.targetCorners
            val cornersArray = extractCornerArray(corners, tagId) ?: continue

            // Add measurements to GTSAM (if uncertainty is acceptable)
            if (gtsamInitialized && uncertainty < maxAcceptableUncertainty) {
                measurementQueue.offer(
                    VisionMeasurement(
                        tagId = tagId,
                        corners = cornersArray,
                        pixelSigma = visionPixelSigma,
                        timestampSeconds = result.controlHubTimeStamp / 1000.0,
                        turretYawRad = turretAngle
                    )
                )
                enqueuedAny = true
                enqueuedVisionCount++
            } else if (!gtsamInitialized) {
                logDebug("Skipping tag $tagId (GTSAM not initialized)")
            } else {
                logDebug(
                    "Skipping tag $tagId (uncertainty %.2f >= %.2f)".format(
                        uncertainty,
                        maxAcceptableUncertainty
                    )
                )
            }
        }

        if (!sawMatchingTag) {
            hasVisionTarget = false
            trackedTagId = -1
        } else if (!enqueuedAny) {
            logDebug("Matching tag(s) seen but not enqueued")
        }
    }

    // ===== Uncertainty Estimation =====

    private fun updateUncertainty(robotPose: Pose2d, turretAngle: Double, dt: Double) {
        if (lastRobotPoseForUncertainty != null && dt > 0) {
            val robotAngVel = normalizeAngle(robotPose.rot - lastRobotPoseForUncertainty!!.rot) / dt
            val turretAngVel = normalizeAngle(turretAngle - lastTurretAngle) / dt

            val fieldRelTurretAngVel = abs(robotAngVel + turretAngVel)

            val motionUncertainty = (fieldRelTurretAngVel * turretRotationUncertaintyGain).coerceIn(0.0, 1.0)

            uncertainty = if (motionUncertainty > uncertainty) {
                motionUncertainty
            } else {
                val decayFactor = uncertaintyDecayRate.pow(dt)
                uncertainty * decayFactor + motionUncertainty * (1 - decayFactor)
            }
        }

        lastRobotPoseForUncertainty = robotPose
    }

    // ===== Aiming Computation =====

    private fun computeAimingAngles() {
        // Vector from robot to goal (using fused pose)
        val dx = goalPosition.x - fusedPose.v.x
        val dy = goalPosition.y - fusedPose.v.y

        // Distance to goal
        targetDistance = hypot(dx, dy)

        // Angle to goal in field frame
        targetFieldAngle = atan2(dy, dx)

        // Convert to robot-relative angle
        targetRobotAngle = normalizeAngle(targetFieldAngle - fusedPose.rot)

        // Determine target availability
        val timeSinceDetection = System.currentTimeMillis() - lastDetectionTimeMs

        hasTarget = when {
            // Always have target if GTSAM initialized (we know goal position)
            gtsamInitialized -> true
            // Fallback: only if we have recent vision
            timeSinceDetection < predictionTimeoutMs -> true
            else -> false
        }

        // Determine if using prediction
        usingPrediction = !hasVisionTarget || uncertainty >= maxAcceptableUncertainty
    }

    // ===== Public API Methods (matching AutoAim interface) =====

    fun getTurretYawAdjustment(): Double {
        if (!enabled || !hasVisionTarget) {
            return 0.0
        }
        val adjustedTx = if (targetTx.absoluteValue < txDeadband) {
            0.0
        } else {
            targetTx * txSignMultiplier
        }
        return adjustedTx
    }

    fun getTargetTurretAngle(): Double? {
        if (!enabled || !hasTarget) {
            return null
        }
        return targetRobotAngle
    }

    fun getTargetFieldAngle(): Double? {
        if (!enabled || !hasTarget) {
            return null
        }
        return targetFieldAngle
    }

    fun resetTarget(newPose: Pose2d? = null) {
        hasTarget = false
        hasVisionTarget = false
        usingPrediction = false
        uncertainty = 0.0

        newPose?.let {
            measurementQueue.offer(InitializeCommand(it, landmarkPositions))
        }
    }

    fun configure(pipeline: Int = 0) {
        limelight?.pipelineSwitch(pipeline)
        limelight?.start()
    }

    fun stop() {
        limelight?.stop()
    }

    // ===== Utility Functions =====

    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle
        while (normalized > PI) normalized -= 2 * PI
        while (normalized < -PI) normalized += 2 * PI
        return normalized
    }

    // ===== Cleanup =====

    override fun close() {
        logInfo("Closing AutoAimGTSAM")
        running.set(false)
        measurementQueue.offer(ShutdownCommand)

        // Wait for worker thread to finish (with timeout)
        try {
            gtsamThread.join(1000)

            if (gtsamThread.isAlive) {
                logError("Worker thread did not shut down gracefully")
                gtsamThread.interrupt()
            }
        } catch (e: InterruptedException) {
            logError("Interrupted while waiting for worker thread", e)
        }

        limelight?.stop()
    }

    private fun logInfo(message: String) {
        enqueueLog(message)
        Log.i(LOG_TAG, message)
    }

    private fun logDebug(message: String) {
        if (enableDebugLogging) {
            logInfo(message)
        }
    }

    private fun logError(message: String, throwable: Throwable? = null) {
        val suffix = if (throwable == null) "" else " (${throwable.message})"
        enqueueLog("ERROR: $message$suffix")
        if (throwable != null) {
            Log.e(LOG_TAG, message, throwable)
        } else {
            Log.e(LOG_TAG, message)
        }
    }

    private fun extractCornerArray(corners: List<List<Double>>, tagId: Int): DoubleArray? {
        if (corners.size >= 4 && corners.all { it.size >= 2 }) {
            val cornersArray = DoubleArray(8)
            for (i in 0 until 4) {
                val corner = corners[i]
                cornersArray[i * 2] = corner[0]
                cornersArray[i * 2 + 1] = corner[1]
            }
            return cornersArray
        }

        val flat = corners.flatMap { it.asIterable() }
        if (flat.size >= 8) {
            return DoubleArray(8) { i -> flat[i] }
        }

        logDebug(
            "Skipping tag $tagId (invalid corner data size=${corners.size} " +
                "lens=${corners.joinToString(prefix = "\"[\"", postfix = "\"]\"") { it.size.toString() }})"
        )
        return null
    }

    private fun flushLogs() {
        val sink = logSink ?: return
        var remaining = maxFlushPerUpdate
        while (remaining-- > 0) {
            val next = pendingLogs.poll() ?: break
            pendingLogCount.decrementAndGet()
            sink(next)
        }
    }

    private fun enqueueLog(message: String) {
        if (pendingLogCount.incrementAndGet() <= maxPendingLogs) {
            pendingLogs.add(message)
        } else {
            pendingLogCount.decrementAndGet()
        }
    }

    private fun logWorkerInfo(message: String) {
        Log.i(LOG_TAG, message)
        System.out.println("$LOG_TAG: $message")
    }

    private fun logWorkerError(message: String, throwable: Throwable? = null) {
        if (throwable != null) {
            Log.e(LOG_TAG, message, throwable)
        } else {
            Log.e(LOG_TAG, message)
        }
        System.out.println("$LOG_TAG ERROR: $message")
        throwable?.let { System.out.println("$LOG_TAG ERROR: ${it.stackTraceToString()}") }
    }
}
