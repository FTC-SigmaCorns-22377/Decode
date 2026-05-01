package sigmacorns.control.localization

import com.bylazar.configurables.annotations.Configurable
import kotlin.math.PI

@Configurable
object EstimatorConfig {
    @JvmField var priorSigmaXY = 5.0
    @JvmField var priorSigmaTheta = 5.0
    @JvmField var odomSigmaXY = 0.003
    @JvmField var odomSigmaTheta = 0.005
    @JvmField var defaultPixelSigma = 2.0
    @JvmField var relinearizeThreshold = 0.01
    @JvmField var relinearizeSkip = 1
    @JvmField var enablePartialRelinearization = true
    @JvmField var compactOdometry = true
    @JvmField var enableRobustTagLoss = true
    @JvmField var robustTagLoss = 0
    @JvmField var robustTagLossK = 1.5
    @JvmField var enableTagGating = true
    /** Minimum tag area as % of image (Limelight ta value, 0-100). Tags below this are discarded. */
    @JvmField var minTagAreaPct = 0.1
    @JvmField var maxTagViewAngleDeg = 60.0
    @JvmField var enableCheiralityCheck = true
    @JvmField var cheiralitySigma = 0.1
    @JvmField var minTagZDistance = 0.02
    @JvmField var enablePostProcess = true
    @JvmField var postProcessVisionGapS = 0.4
    @JvmField var postProcessSettleS = 1.0
    @JvmField var postProcessSettleUpdates = 3
    @JvmField var postGapExtraIsam2Updates = 2
    @JvmField var fx = 1221.445
    @JvmField var fy = 1223.398
    @JvmField var cx = 637.226
    @JvmField var cy = 502.549
    @JvmField var k1 = 0.177168
    @JvmField var k2 = -0.457341
    @JvmField var k3 = 0.178259
    @JvmField var p1 = 0.000360
    @JvmField var p2 = 0.002753
    @JvmField var cameraOffsetX = 0.143044
    @JvmField var cameraOffsetY = 0.0
    @JvmField var cameraOffsetZ = 0.30596824
    @JvmField var cameraRoll = -Math.toRadians(80.0)
    @JvmField var cameraPitch = 0.0
    @JvmField var cameraYaw = -PI / 2.0

    @JvmField var pixelSigmaAngleK = 2.0
    @JvmField var enableSpatialCorrelation = true
    @JvmField var correlationDistanceM = 0.3
    @JvmField var correlationDownweightFactor = 2.0
    @JvmField var correlationHistorySize = 100
    @JvmField var enableBiasCorrection = true
    @JvmField var radialBiasK = 0.01
    @JvmField var enableMultiHypothesisInit = true
    @JvmField var multiHypothesisThetaThreshold = 1.0
    @JvmField var enableHeadingFlipRecovery = true
    @JvmField var headingFlipMinTags = 1

    /** Max robot angular velocity (rad/s) before vision updates are suppressed. 0 = disabled. */
    @JvmField var maxRobotAngularVelForVision = 1.5
    /** Max turret angular velocity (rad/s) before vision updates are suppressed. 0 = disabled. */
    @JvmField var maxTurretAngularVelForVision = 2.0

    @JvmField var headingFlipConsecutiveFrames = 2
    @JvmField var enableReprojectionGate = true
    @JvmField var maxReprojectionErrorPx = 50.0
    @JvmField var enablePoseJumpGuard = true
    @JvmField var poseJumpMaxM = 0.5
    @JvmField var poseJumpMaxRad = 0.5
    @JvmField var multiHypothesisMinTags = 2
}
