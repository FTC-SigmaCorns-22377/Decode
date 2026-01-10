package sigmacorns.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.constants.FieldZones
import sigmacorns.control.AutoAimGTSAM
import sigmacorns.control.MotorRangeMapper
import sigmacorns.control.Turret
import sigmacorns.io.HardwareIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@TeleOp(name = "AutoAim GTSAM Test", group = "Test")
class AutoAimGTSAMTest : SigmaOpMode() {

    @Configurable
    object AutoAimGTSAMTestConfig {
        @JvmField var pipeline = 0
        @JvmField var cameraMountingOffsetYaw = 0.0
        @JvmField var txSignMultiplier = -1.0
        @JvmField var txDeadband = 0.01
        @JvmField var maxAcceptableUncertainty = Double.MAX_VALUE
        @JvmField var predictionTimeoutMs = 2000L
        @JvmField var visionPixelSigma = 1.0
        @JvmField var maxPoseDivergence = 2.0
        @JvmField var turretRotationUncertaintyGain = 1.5
        @JvmField var uncertaintyDecayRate = 0.01

        @JvmField var priorSigmaXY = 0.05
        @JvmField var priorSigmaTheta = 0.02
        @JvmField var odomSigmaXY = 0.0002
        @JvmField var odomSigmaTheta = 0.001
        @JvmField var defaultPixelSigma = 2.0
        @JvmField var relinearizeThreshold = 0.01
        @JvmField var relinearizeSkip = 1
        @JvmField var enablePartialRelinearization = true
        @JvmField var compactOdometry = true
        @JvmField var enableRobustTagLoss = false
        @JvmField var robustTagLoss = 0
        @JvmField var robustTagLossK = 1.5
        @JvmField var enableTagGating = true
        @JvmField var minTagAreaPx = 50.0
        @JvmField var maxTagViewAngleDeg = 60.0
        @JvmField var enablePostProcess = true
        @JvmField var postProcessVisionGapS = 0.4
        @JvmField var postProcessSettleS = 2.0
        @JvmField var postProcessSettleUpdates = 3
        @JvmField var fx = 1221.445 / 2.0
        @JvmField var fy = 1223.398 / 2.0
        @JvmField var cx = 637.226 / 2.0
        @JvmField var cy = 502.549 / 2.0
        @JvmField var k1 = 0.177168
        @JvmField var k2 = -0.457341
        @JvmField var k3 = 0.178259
        @JvmField var p1 = 0.000360
        @JvmField var p2 = 0.002753
        @JvmField var cameraOffsetX = 0.18312971
        @JvmField var cameraOffsetY = 0.0
        @JvmField var cameraOffsetZ = 0.32448638
        @JvmField var cameraRoll = 0.0
        @JvmField var cameraPitch = Math.toRadians(10.0)
        @JvmField var cameraYaw = 0.0
    }

    private val ticksPerRad = (1.0 + (46.0 / 11.0)) * 28.0 / (2 * PI) * 76 / 19
    private val turretRange = MotorRangeMapper(
        limits = -PI / 2.0..PI / 2.0,
        limitsTick = -PI / 2.0 * ticksPerRad..PI / 2.0 * ticksPerRad,
        slowdownDist = 0.3
    )

    private fun buildEstimatorConfig(): AutoAimGTSAM.EstimatorConfig {
        return AutoAimGTSAM.EstimatorConfig(
            priorSigmaXY = AutoAimGTSAMTestConfig.priorSigmaXY,
            priorSigmaTheta = AutoAimGTSAMTestConfig.priorSigmaTheta,
            odomSigmaXY = AutoAimGTSAMTestConfig.odomSigmaXY,
            odomSigmaTheta = AutoAimGTSAMTestConfig.odomSigmaTheta,
            defaultPixelSigma = AutoAimGTSAMTestConfig.defaultPixelSigma,
            relinearizeThreshold = AutoAimGTSAMTestConfig.relinearizeThreshold,
            relinearizeSkip = AutoAimGTSAMTestConfig.relinearizeSkip,
            enablePartialRelinearization = AutoAimGTSAMTestConfig.enablePartialRelinearization,
            compactOdometry = AutoAimGTSAMTestConfig.compactOdometry,
            enableRobustTagLoss = AutoAimGTSAMTestConfig.enableRobustTagLoss,
            robustTagLoss = AutoAimGTSAMTestConfig.robustTagLoss,
            robustTagLossK = AutoAimGTSAMTestConfig.robustTagLossK,
            enableTagGating = AutoAimGTSAMTestConfig.enableTagGating,
            minTagAreaPx = AutoAimGTSAMTestConfig.minTagAreaPx,
            maxTagViewAngleDeg = AutoAimGTSAMTestConfig.maxTagViewAngleDeg,
            enablePostProcess = AutoAimGTSAMTestConfig.enablePostProcess,
            postProcessVisionGapS = AutoAimGTSAMTestConfig.postProcessVisionGapS,
            postProcessSettleS = AutoAimGTSAMTestConfig.postProcessSettleS,
            postProcessSettleUpdates = AutoAimGTSAMTestConfig.postProcessSettleUpdates,
            fx = AutoAimGTSAMTestConfig.fx,
            fy = AutoAimGTSAMTestConfig.fy,
            cx = AutoAimGTSAMTestConfig.cx,
            cy = AutoAimGTSAMTestConfig.cy,
            k1 = AutoAimGTSAMTestConfig.k1,
            k2 = AutoAimGTSAMTestConfig.k2,
            k3 = AutoAimGTSAMTestConfig.k3,
            p1 = AutoAimGTSAMTestConfig.p1,
            p2 = AutoAimGTSAMTestConfig.p2,
            cameraOffsetX = AutoAimGTSAMTestConfig.cameraOffsetX,
            cameraOffsetY = AutoAimGTSAMTestConfig.cameraOffsetY,
            cameraOffsetZ = AutoAimGTSAMTestConfig.cameraOffsetZ,
            cameraRoll = AutoAimGTSAMTestConfig.cameraRoll,
            cameraPitch = AutoAimGTSAMTestConfig.cameraPitch,
            cameraYaw = AutoAimGTSAMTestConfig.cameraYaw
        )
    }

    private fun applyRuntimeConfig(autoAim: AutoAimGTSAM) {
        autoAim.cameraMountingOffsetYaw = AutoAimGTSAMTestConfig.cameraMountingOffsetYaw
        autoAim.txSignMultiplier = AutoAimGTSAMTestConfig.txSignMultiplier
        autoAim.txDeadband = AutoAimGTSAMTestConfig.txDeadband
        autoAim.maxAcceptableUncertainty = AutoAimGTSAMTestConfig.maxAcceptableUncertainty
        autoAim.predictionTimeoutMs = AutoAimGTSAMTestConfig.predictionTimeoutMs
        autoAim.visionPixelSigma = AutoAimGTSAMTestConfig.visionPixelSigma
        autoAim.maxPoseDivergence = AutoAimGTSAMTestConfig.maxPoseDivergence
        autoAim.turretRotationUncertaintyGain = AutoAimGTSAMTestConfig.turretRotationUncertaintyGain
        autoAim.uncertaintyDecayRate = AutoAimGTSAMTestConfig.uncertaintyDecayRate
    }

    override fun runOpMode() {
        io.configurePinpoint()
        io.setPosition(Pose2d(0.0,0.0,PI/2.0))

        val hardwareIO = io as? HardwareIO
        val turret = Turret(turretRange, io)
        turret.fieldRelativeMode = true

        // Update these AprilTag positions (meters) to match your field layout.
        val landmarks = mapOf(
            20 to AutoAimGTSAM.LandmarkSpec(
                Vector3d(-1.413321, 1.481870, 0.7493),
                size = 0.165
            ),
            24 to AutoAimGTSAM.LandmarkSpec(
                Vector3d(1.413321, 1.481870, 0.7493),
                size = 0.165
            ),
        )
        val goalPosition = Vector2d(-1.480126, 1.598982)

        val autoAim = AutoAimGTSAM(
            limelight = hardwareIO?.limelight,
            landmarkPositions = landmarks,
            goalPosition = goalPosition,
            initialPose = Pose2d(),
            estimatorConfig = buildEstimatorConfig()
        )
        autoAim.logSink = { message -> telemetry.log().add(message) }

        applyRuntimeConfig(autoAim)
        autoAim.configure(pipeline = AutoAimGTSAMTestConfig.pipeline)
        autoAim.enabled = true

        telemetry.addLine("AutoAim GTSAM test ready")
        telemetry.update()

        waitForStart()

        try {
            ioLoop { _, dt ->
                applyRuntimeConfig(autoAim)
                autoAim.update(io.position(), turret.pos)

                autoAim.getTargetFieldAngle()?.let { targetAngle ->
                    turret.fieldTargetAngle = targetAngle
                }
                turret.update(dt)

                val fusedPose = autoAim.fusedPose
                telemetry.addData(
                    "Fused Pose (m, m, rad)",
                    "%.2f, %.2f, %.2f",
                    fusedPose.v.x,
                    fusedPose.v.y,
                    fusedPose.rot
                )
                telemetry.addData("Vision Target", autoAim.hasVisionTarget)
                telemetry.addData("Has Target", autoAim.hasTarget)
                telemetry.addData("Tag ID", autoAim.trackedTagId)
                telemetry.addData("Detections", autoAim.detectedTagCount)
                telemetry.addData("Tx/Ty (deg)", "%.2f / %.2f", autoAim.rawTxDegrees, autoAim.rawTyDegrees)
                telemetry.addData("Uncertainty", "%.2f", autoAim.uncertainty)
                telemetry.addData("Prediction", autoAim.usingPrediction)
                telemetry.update()

                false
            }
        } finally {
            autoAim.close()
        }
    }
}
