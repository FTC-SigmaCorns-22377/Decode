package sigmacorns.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.panels.Panels
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.Quaterniond
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.control.subsystem.DriveController
import sigmacorns.control.aim.AimConfig
import sigmacorns.control.aim.AutoAimGTSAM
import sigmacorns.control.MotorRangeMapper
import sigmacorns.control.subsystem.Turret
import sigmacorns.control.aim.VisionTracker
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
        @JvmField var visionPixelSigma = 3.0
        @JvmField var maxPoseDivergence = 2.0
        @JvmField var turretRotationUncertaintyGain = 1.5
        @JvmField var uncertaintyDecayRate = 0.01

        @JvmField var priorSigmaXY = 2.0
        @JvmField var priorSigmaTheta = 2.0
        @JvmField var odomSigmaXY = 0.0002
        @JvmField var odomSigmaTheta = 0.0002
        @JvmField var defaultPixelSigma = 2.0
        @JvmField var relinearizeThreshold = 0.01
        @JvmField var relinearizeSkip = 1
        @JvmField var enablePartialRelinearization = true
        @JvmField var compactOdometry = true
        @JvmField var enableRobustTagLoss = true
        @JvmField var robustTagLoss = 0
        @JvmField var robustTagLossK = 1.5
        @JvmField var enableTagGating = false
        @JvmField var minTagAreaPx = 50.0
        @JvmField var maxTagViewAngleDeg = 60.0
        @JvmField var enableCheiralityCheck = true
        @JvmField var cheiralitySigma = 0.1
        @JvmField var minTagZDistance = 0.02
        @JvmField var enablePostProcess = true
        @JvmField var postProcessVisionGapS = 0.4
        @JvmField var postProcessSettleS = 1.0
        @JvmField var postProcessSettleUpdates = 3
        @JvmField var fx = 1221.445
        @JvmField var fy = 1223.398
        @JvmField var cx = 637.226
        @JvmField var cy = 502.549
        @JvmField var k1 = 0.177168
        @JvmField var k2 = -0.457341
        @JvmField var k3 = 0.178259
        @JvmField var p1 = 0.000360
        @JvmField var p2 = 0.002753
        @JvmField var cameraOffsetX = 0.18312971
        @JvmField var cameraOffsetY = 0.0
        @JvmField var cameraOffsetZ = 0.32448638
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
    }

    private val ticksPerRad = (1.0 + (46.0 / 11.0)) * 28.0 / (2 * PI) * 76 / 19
    private val turretRange = MotorRangeMapper(
        limits = -PI / 2.0..PI / 2.0,
        limitsTick = -PI / 2.0 * ticksPerRad..PI / 2.0 * ticksPerRad,
        slowdownDist = 0.3
    )

    companion object {
        fun buildEstimatorConfig(): AutoAimGTSAM.EstimatorConfig {
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
                enableCheiralityCheck = AutoAimGTSAMTestConfig.enableCheiralityCheck,
                cheiralitySigma = AutoAimGTSAMTestConfig.cheiralitySigma,
                minTagZDistance = AutoAimGTSAMTestConfig.minTagZDistance,
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
                cameraYaw = AutoAimGTSAMTestConfig.cameraYaw,
                pixelSigmaAngleK = AutoAimGTSAMTestConfig.pixelSigmaAngleK,
                enableSpatialCorrelation = AutoAimGTSAMTestConfig.enableSpatialCorrelation,
                correlationDistanceM = AutoAimGTSAMTestConfig.correlationDistanceM,
                correlationDownweightFactor = AutoAimGTSAMTestConfig.correlationDownweightFactor,
                correlationHistorySize = AutoAimGTSAMTestConfig.correlationHistorySize,
                enableBiasCorrection = AutoAimGTSAMTestConfig.enableBiasCorrection,
                radialBiasK = AutoAimGTSAMTestConfig.radialBiasK,
                enableMultiHypothesisInit = AutoAimGTSAMTestConfig.enableMultiHypothesisInit,
                multiHypothesisThetaThreshold = AutoAimGTSAMTestConfig.multiHypothesisThetaThreshold,
                enableHeadingFlipRecovery = AutoAimGTSAMTestConfig.enableHeadingFlipRecovery,
                headingFlipMinTags = AutoAimGTSAMTestConfig.headingFlipMinTags
            )
        }

        fun applyRuntimeConfig(autoAim: AutoAimGTSAM) {
            autoAim.aimConfig = AimConfig(
                cameraMountingOffsetYaw = AutoAimGTSAMTestConfig.cameraMountingOffsetYaw,
                txSignMultiplier = AutoAimGTSAMTestConfig.txSignMultiplier,
                txDeadband = AutoAimGTSAMTestConfig.txDeadband,
                maxAcceptableUncertainty = AutoAimGTSAMTestConfig.maxAcceptableUncertainty,
                predictionTimeoutMs = AutoAimGTSAMTestConfig.predictionTimeoutMs,
                visionPixelSigma = AutoAimGTSAMTestConfig.visionPixelSigma,
                maxPoseDivergence = AutoAimGTSAMTestConfig.maxPoseDivergence,
                turretRotationUncertaintyGain = AutoAimGTSAMTestConfig.turretRotationUncertaintyGain,
                uncertaintyDecayRate = AutoAimGTSAMTestConfig.uncertaintyDecayRate
            )
        }
    }


    override fun runOpMode() {
        Panels.config.enableLogs = false

        val driveController = DriveController()
        io.configurePinpoint()
        io.setPosition(Pose2d(0.0,0.0,PI/2.0))

        val ll = (io as? HardwareIO)!!.limelight!!
        ll.pipelineSwitch(0)
        ll.start()

        val hardwareIO = io as? HardwareIO
        val turret = Turret(turretRange, io)
        turret.fieldRelativeMode = true

        val q20 = Quaterniond().rotateX(-PI/2.0).rotateLocalZ(Math.toRadians(54.046000))
        val ypr20 = Vector3d()
        q20.getEulerAnglesZYX(ypr20)


        val q24 = Quaterniond().rotateX(-PI/2.0).rotateLocalZ(-Math.toRadians(54.046000))
        val ypr24 = Vector3d()
        q24.getEulerAnglesZYX(ypr24)
        // Update these AprilTag positions (meters) to match your field layout.
        val landmarks = mapOf(
            20 to AutoAimGTSAM.LandmarkSpec(
                Vector3d(-1.413321, 1.481870, 0.7493),
                pitch = ypr20.y,
                roll = ypr20.x,
                yaw = ypr20.z,
                size = 0.165
            ),
            24 to AutoAimGTSAM.LandmarkSpec(
                Vector3d(1.413321, 1.481870, 0.7493),
                pitch = ypr24.y,
                roll = ypr24.x,
                yaw = ypr24.z,
                size = 0.165
            ),
            22 to AutoAimGTSAM.LandmarkSpec(
                position = Vector3d(0.0,1.818888,0.459341),
                //position = Vector3d(0.0,0.0,0.459341),
                roll = -PI/2.0,
                pitch = 0.0,
                yaw = 0.0,
                size = 0.165
            )
        )
        val goalPosition = Vector2d(-1.480126, 1.598982)

        var autoAim: AutoAimGTSAM? = null
        var visionTracker: VisionTracker? = null

        try {
            autoAim = AutoAimGTSAM(
                landmarkPositions = landmarks,
                goalPosition = goalPosition,
                initialPose = Pose2d(),
                estimatorConfig = buildEstimatorConfig()
            )
            visionTracker = VisionTracker(
                limelight = hardwareIO?.limelight,
                allowedTagIds = landmarks.keys
            )

            applyRuntimeConfig(autoAim)
            visionTracker.configure(pipeline = AutoAimGTSAMTestConfig.pipeline)
            autoAim.enabled = true
            autoAim.enableDebugLogging()

            telemetry.addLine("AutoAim GTSAM test ready")
            telemetry.update()

            waitForStart()
            if (isStopRequested) {
                return
            }

            ioLoop { _, dt ->
                applyRuntimeConfig(autoAim)
                val visionResult = visionTracker.read()
                autoAim.update(io.position(), turret.pos, visionResult)

                // Keep field-relative turret aiming aligned with the current robot heading.
                turret.robotHeading = autoAim.fusedPose.rot
                turret.robotAngularVelocity = io.velocity().rot

                autoAim.getTargetFieldAngle()?.let { targetAngle ->
                    turret.fieldTargetAngle = targetAngle
                }
                turret.update(dt)

                driveController.update(gamepad1,io)

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

                autoAim.logLandmarkCorners(22)
                println("CALLING MANUALLY")
                autoAim.logCameraUnitVectors()

                false
            }
        } finally {
            autoAim?.close()
            visionTracker?.stop()
        }
    }
}
