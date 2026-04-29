package sigmacorns.control.localization

import sigmacorns.math.Pose2d
import org.joml.Vector2d
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

class FusionWorker(
    private val initialPose: Pose2d
) : AutoCloseable {

    private data class OdometryMeasurement(
        val dx: Double,
        val dy: Double,
        val dtheta: Double,
        val timestampSeconds: Double
    )

    private data class VisionMeasurement(
        val tagId: Int,
        val corners: DoubleArray,
        val pixelSigma: Double,
        val timestampSeconds: Double,
        val turretYawRad: Double
    )

    private data class InitializeCommand(
        val pose: Pose2d,
        val landmarks: Map<Int, GTSAMEstimator.LandmarkSpec>
    )

    data class MemoryUsage(
        val virtualBytes: Long,
        val residentBytes: Long,
        val sharedBytes: Long,
        val dataBytes: Long,
        val valid: Boolean
    )

    data class DiagnosticsSnapshot(
        val pendingTags: Long,
        val pendingGraphFactors: Long,
        val pendingValues: Long,
        val currentPoseIndex: Long,
        val horizonCapacity: Long,
        val lastSolveMs: Double,
        val avgSolveMs: Double,
        val lastHorizonResetMs: Double,
        val lastHorizonCovMs: Double,
        val lastHorizonResetPoseIndex: Long
    )

    data class FactorGraphData(
        val factorTypes: List<PoseEstimatorBridge.FactorType>,
        val factorKeys: List<List<Int>>,
        val poseValues: List<Pose2d>,
        val trajectory: List<Pose2d>
    )

    private object Tick

    private val odomQueue = LinkedBlockingQueue<OdometryMeasurement>()
    private val visionQueue = LinkedBlockingQueue<VisionMeasurement>()
    private val tickQueue = LinkedBlockingQueue<Tick>()
    private val pendingInit = AtomicReference<InitializeCommand?>(null)
    private val running = AtomicBoolean(false)
    private val lastError = AtomicReference<String?>(null)
    private val workerThread: Thread

    @Volatile var fusedPose: Pose2d = initialPose
        private set

    @Volatile var isInitialized: Boolean = false
        private set

    @Volatile var lastSolveTimeMs: Double = 0.0
        private set

    @Volatile var lastMemoryUsage: MemoryUsage? = null
        private set

    @Volatile var lastDiagnosticsSnapshot: DiagnosticsSnapshot? = null
        private set

    @Volatile private var lastDiagnosticsUpdateMs: Long = 0

    @Volatile private var gtsamHandle: Long = 0L


    fun requestInitialize(pose: Pose2d, landmarks: Map<Int, GTSAMEstimator.LandmarkSpec>) {
        pendingInit.set(InitializeCommand(pose, landmarks))
        tickQueue.offer(Tick)
    }

    fun hasPendingInit(): Boolean = pendingInit.get() != null

    fun enqueueOdometry(dx: Double, dy: Double, dtheta: Double, timestampSeconds: Double) {
        odomQueue.offer(OdometryMeasurement(dx, dy, dtheta, timestampSeconds))
    }

    fun enqueueVision(
        tagId: Int,
        corners: DoubleArray,
        pixelSigma: Double,
        timestampSeconds: Double,
        turretYawRad: Double
    ) {
        visionQueue.offer(
            VisionMeasurement(
                tagId = tagId,
                corners = corners,
                pixelSigma = pixelSigma,
                timestampSeconds = timestampSeconds,
                turretYawRad = turretYawRad
            )
        )
    }
    
    // New method to retrieve predicted corners
    fun getPredictedCorners(tagId: Int): List<Vector2d> {
        val handle = gtsamHandle
        if (handle == 0L) return emptyList()
        try {
            val corners = PoseEstimatorBridge.nativeGetPredictedCorners(handle, tagId)
            if (corners == null || corners.isEmpty()) return emptyList()
            
            val result = ArrayList<Vector2d>(corners.size / 2)
            for (i in 0 until corners.size / 2) {
                result.add(Vector2d(corners[2*i], corners[2*i+1]))
            }
            return result
        } catch (e: Exception) {
            return emptyList()
        }
    }

    fun tick() {
        tickQueue.offer(Tick)
    }

    fun pollError(): String? = lastError.getAndSet(null)

    fun getEstimatorHandle(): Long = gtsamHandle

    fun getGraphData(): FactorGraphData? {
        val handle = gtsamHandle
        if (handle == 0L) return null
        try {
            val factorTypes = PoseEstimatorBridge.nativeGetGraphFactorTypes(handle)
            val factorKeys = PoseEstimatorBridge.nativeGetGraphFactorKeys(handle)
            val poseValues = PoseEstimatorBridge.nativeGetGraphPoseValues(handle)
            val trajectory = PoseEstimatorBridge.nativeGetCurrentTrajectory(handle)

            if (factorTypes == null || factorKeys == null || poseValues == null || trajectory == null) {
                return null
            }

            // Parse factor types
            val types = factorTypes.map { typeOrdinal ->
                PoseEstimatorBridge.FactorType.entries.getOrElse(typeOrdinal) {
                    PoseEstimatorBridge.FactorType.UNKNOWN
                }
            }

            // Parse factor keys: [num_factors, factor0_num_keys, factor0_key0, factor0_key1, ...]
            val keys = mutableListOf<List<Int>>()
            var idx = 0
            val numFactors = factorKeys[idx++]
            for (i in 0 until numFactors) {
                val numKeys = factorKeys[idx++]
                val keyList = (0 until numKeys).map { factorKeys[idx++] }
                keys.add(keyList)
            }

            // Parse pose values: [num_poses, pose0_x, pose0_y, pose0_theta, ...]
            val poses = mutableListOf<Pose2d>()
            idx = 0
            val numPoses = poseValues[idx++].toInt()
            for (i in 0 until numPoses) {
                poses.add(Pose2d(poseValues[idx++], poseValues[idx++], poseValues[idx++]))
            }

            // Parse trajectory: [num_poses, pose0_x, pose0_y, pose0_theta, ...]
            val traj = mutableListOf<Pose2d>()
            idx = 0
            val numTrajPoses = trajectory[idx++].toInt()
            for (i in 0 until numTrajPoses) {
                traj.add(Pose2d(trajectory[idx++], trajectory[idx++], trajectory[idx++]))
            }

            return FactorGraphData(types, keys, poses, traj)
        } catch (e: Exception) {
            return null
        }
    }

    override fun close() {
        running.set(false)
        tickQueue.offer(Tick)
        try {
            workerThread.join(1000)
            if (workerThread.isAlive) {
                workerThread.interrupt()
            }
        } catch (_: InterruptedException) {
            workerThread.interrupt()
        }
    }

    private val workerLoop = Runnable {
        running.set(true)
        var localHandle: Long = 0L

        try {
            while (running.get()) {
                tickQueue.poll(100, TimeUnit.MILLISECONDS) ?: continue

                val init = pendingInit.getAndSet(null)
                if (init != null) {
                    try {
                        if (localHandle != 0L) {
                            PoseEstimatorBridge.nativeDestroy(localHandle)
                        }
                        localHandle = createHandle()
                        gtsamHandle = localHandle
                        
                        if (localHandle == 0L) {
                            lastError.set("Failed to create GTSAM handle")
                            isInitialized = false
                            continue
                        }
                        initializeHandle(localHandle, init)
                        fusedPose = init.pose
                        isInitialized = true
                    } catch (e: Exception) {
                        lastError.set("GTSAM initialization failed: ${e.message}")
                        isInitialized = false
                    }
                }

                if (!isInitialized) {
                    continue
                }

                var hadUpdates = false
                var odom = odomQueue.poll()
                while (odom != null) {
                    PoseEstimatorBridge.nativeProcessOdometry(
                        localHandle,
                        odom.dx,
                        odom.dy,
                        odom.dtheta,
                        odom.timestampSeconds
                    )
                    hadUpdates = true
                    odom = odomQueue.poll()
                }

                var vision = visionQueue.poll()
                while (vision != null) {
                    PoseEstimatorBridge.nativeAddTagMeasurement(
                        localHandle,
                        vision.tagId,
                        vision.corners,
                        vision.pixelSigma,
                        vision.timestampSeconds,
                        vision.turretYawRad
                    )
                    hadUpdates = true
                    vision = visionQueue.poll()
                }

                if (hadUpdates) {
                    try {
                        val startNs = System.nanoTime()
                        PoseEstimatorBridge.nativeUpdate(localHandle)
                        lastSolveTimeMs = (System.nanoTime() - startNs) / 1_000_000.0
                        val estimate = PoseEstimatorBridge.nativeGetCurrentEstimate(localHandle)
                        if (estimate.size >= 3) {
                            fusedPose = Pose2d(estimate[0], estimate[1], estimate[2])
                        }
                        updateDiagnosticsIfNeeded(localHandle)
                    } catch (e: Exception) {
                        lastError.set("GTSAM update failed: ${e.message}")
                    }
                }
            }
        } finally {
            if (localHandle != 0L) {
                try {
                    PoseEstimatorBridge.nativeDestroy(localHandle)
                } catch (e: Exception) {
                    lastError.set("GTSAM destroy failed: ${e.message}")
                }
            }
            gtsamHandle = 0L
        }
    }

    init {
        workerThread = Thread(workerLoop, "GTSAM-Worker")
        workerThread.start()
    }

    private fun updateDiagnosticsIfNeeded(handle: Long) {
        val nowMs = System.currentTimeMillis()
        if (nowMs - lastDiagnosticsUpdateMs < 1000) {
            return
        }
        lastDiagnosticsUpdateMs = nowMs

        try {
            val usage = PoseEstimatorBridge.nativeGetLastMemoryUsage(handle)
            if (usage.size >= 5) {
                lastMemoryUsage = MemoryUsage(
                    virtualBytes = usage[0],
                    residentBytes = usage[1],
                    sharedBytes = usage[2],
                    dataBytes = usage[3],
                    valid = usage[4] != 0L
                )
            }
        } catch (e: Exception) {
            lastError.set("GTSAM memory usage failed: ${e.message}")
        }

        try {
            val snapshot = PoseEstimatorBridge.nativeGetDiagnosticsSnapshot(handle)
            if (snapshot.size >= 10) {
                lastDiagnosticsSnapshot = DiagnosticsSnapshot(
                    pendingTags = snapshot[0].toLong(),
                    pendingGraphFactors = snapshot[1].toLong(),
                    pendingValues = snapshot[2].toLong(),
                    currentPoseIndex = snapshot[3].toLong(),
                    horizonCapacity = snapshot[4].toLong(),
                    lastSolveMs = snapshot[5].toDouble(),
                    avgSolveMs = snapshot[6].toDouble(),
                    lastHorizonResetMs = snapshot[7].toDouble(),
                    lastHorizonCovMs = snapshot[8].toDouble(),
                    lastHorizonResetPoseIndex = snapshot[9].toLong()
                )
            }
        } catch (e: Exception) {
            lastError.set("GTSAM diagnostics failed: ${e.message}")
        }
    }

    private fun createHandle(): Long {
        val config = EstimatorConfig
        return PoseEstimatorBridge.nativeCreateWithConfig(
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
            config.minTagAreaPct,
            config.maxTagViewAngleDeg,
            config.enableCheiralityCheck,
            config.cheiralitySigma,
            config.minTagZDistance,
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
            config.cameraYaw,
            config.pixelSigmaAngleK,
            config.enableSpatialCorrelation,
            config.correlationDistanceM,
            config.correlationDownweightFactor,
            config.correlationHistorySize,
            config.enableBiasCorrection,
            config.radialBiasK,
            config.enableMultiHypothesisInit,
            config.multiHypothesisThetaThreshold,
            config.enableHeadingFlipRecovery,
            config.headingFlipMinTags
        )
    }

    private fun initializeHandle(handle: Long, init: InitializeCommand) {
        val ids = IntArray(init.landmarks.size)
        val xs = DoubleArray(init.landmarks.size)
        val ys = DoubleArray(init.landmarks.size)
        val zs = DoubleArray(init.landmarks.size)
        val rolls = DoubleArray(init.landmarks.size)
        val pitches = DoubleArray(init.landmarks.size)
        val yaws = DoubleArray(init.landmarks.size)
        val sizes = DoubleArray(init.landmarks.size)
        init.landmarks.entries.forEachIndexed { i, (id, pos) ->
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
            handle,
            ids,
            xs,
            ys,
            zs,
            rolls,
            pitches,
            yaws,
            sizes,
            init.pose.v.x,
            init.pose.v.y,
            init.pose.rot
        )
    }
}
