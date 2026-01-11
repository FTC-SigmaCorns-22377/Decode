package sigmacorns.control.aim

object PoseEstimatorBridge {
    @JvmStatic external fun nativeCreate(): Long
    @JvmStatic external fun nativeCreateWithConfig(
        priorSigmaXY: Double,
        priorSigmaTheta: Double,
        odomSigmaXY: Double,
        odomSigmaTheta: Double,
        defaultPixelSigma: Double,
        relinearizeThreshold: Double,
        relinearizeSkip: Int,
        enablePartialRelinearization: Boolean,
        compactOdometry: Boolean,
        enableRobustTagLoss: Boolean,
        robustTagLoss: Int,
        robustTagLossK: Double,
        enableTagGating: Boolean,
        minTagAreaPx: Double,
        maxTagViewAngleDeg: Double,
        enableCheiralityCheck: Boolean,
        cheiralitySigma: Double,
        minTagZDistance: Double,
        enablePostProcess: Boolean,
        postProcessVisionGapS: Double,
        postProcessSettleS: Double,
        postProcessSettleUpdates: Int,
        fx: Double,
        fy: Double,
        cx: Double,
        cy: Double,
        k1: Double,
        k2: Double,
        k3: Double,
        p1: Double,
        p2: Double,
        cameraOffsetX: Double,
        cameraOffsetY: Double,
        cameraOffsetZ: Double,
        cameraRoll: Double,
        cameraPitch: Double,
        cameraYaw: Double
    ): Long

    @JvmStatic external fun nativeDestroy(handle: Long)
    @JvmStatic external fun nativeInitialize(
        handle: Long,
        tagIds: IntArray,
        landmarkX: DoubleArray,
        landmarkY: DoubleArray,
        landmarkZ: DoubleArray,
        landmarkRoll: DoubleArray,
        landmarkPitch: DoubleArray,
        landmarkYaw: DoubleArray,
        landmarkSize: DoubleArray,
        initialX: Double,
        initialY: Double,
        initialTheta: Double
    )

    @JvmStatic external fun nativeReset(handle: Long)
    @JvmStatic external fun nativeProcessOdometry(
        handle: Long,
        dx: Double,
        dy: Double,
        dtheta: Double,
        timestamp: Double
    )

    @JvmStatic external fun nativeAddTagMeasurement(
        handle: Long,
        tagId: Int,
        corners: DoubleArray,
        pixelSigma: Double,
        timestamp: Double,
        turretYawRad: Double
    )

    @JvmStatic external fun nativeUpdate(handle: Long)
    @JvmStatic external fun nativeGetCurrentEstimate(handle: Long): DoubleArray
    @JvmStatic external fun nativeGetCurrentEstimateWithCovariance(handle: Long): DoubleArray
    @JvmStatic external fun nativeIsInitialized(handle: Long): Boolean
    @JvmStatic external fun nativeGetLastSolveTimeMs(handle: Long): Double
    @JvmStatic external fun nativeGetAverageSolveTimeMs(handle: Long): Double
    @JvmStatic external fun nativeGetLastMemoryUsage(handle: Long): LongArray
    @JvmStatic external fun nativeGetDiagnosticsSnapshot(handle: Long): DoubleArray
    @JvmStatic external fun nativeGetAllLandmarkCorners(handle: Long): DoubleArray
    @JvmStatic external fun nativeGetLandmarkCorners(handle: Long, tagId: Int): DoubleArray
    @JvmStatic external fun nativeGetCameraUnitVectors(handle: Long): DoubleArray
    
    // Visualization
    @JvmStatic external fun nativeEnableVisualization(handle: Long, enabled: Boolean)
    @JvmStatic external fun nativeConfigureVisualization(handle: Long, stream: Boolean, urlOrPath: String, appId: String)
    @JvmStatic external fun nativeFlushVisualization(handle: Long)
    @JvmStatic external fun nativeGetPredictedCorners(handle: Long, tagId: Int): DoubleArray
}
