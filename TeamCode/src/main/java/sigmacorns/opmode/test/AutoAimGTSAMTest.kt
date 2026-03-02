package sigmacorns.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.panels.Panels
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.Quaterniond
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.constants.turretRange
import sigmacorns.subsystem.DriveController
import sigmacorns.control.aim.AimConfig
import sigmacorns.control.localization.GTSAMEstimator
import sigmacorns.control.MotorRangeMapper
import sigmacorns.subsystem.Turret
import sigmacorns.control.localization.VisionTracker
import sigmacorns.io.HardwareIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@TeleOp(name = "AutoAim GTSAM Test", group = "Test")
class AutoAimGTSAMTest : SigmaOpMode() {
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
            20 to GTSAMEstimator.LandmarkSpec(
                Vector3d(-1.413321, 1.481870, 0.7493),
                pitch = ypr20.y,
                roll = ypr20.x,
                yaw = ypr20.z,
                size = 0.165
            ),
            24 to GTSAMEstimator.LandmarkSpec(
                Vector3d(1.413321, 1.481870, 0.7493),
                pitch = ypr24.y,
                roll = ypr24.x,
                yaw = ypr24.z,
                size = 0.165
            ),
            22 to GTSAMEstimator.LandmarkSpec(
                position = Vector3d(0.0,1.818888,0.459341),
                //position = Vector3d(0.0,0.0,0.459341),
                roll = -PI/2.0,
                pitch = 0.0,
                yaw = 0.0,
                size = 0.165
            )
        )
        val goalPosition = Vector2d(-1.480126, 1.598982)

        var autoAim: GTSAMEstimator? = null
        var visionTracker: VisionTracker? = null

        try {
            autoAim = GTSAMEstimator(
                landmarkPositions = landmarks,
                initialPose = Pose2d(),
            )
            visionTracker = VisionTracker(
                limelight = hardwareIO?.limelight,
                allowedTagIds = landmarks.keys
            )

            autoAim.enabled = true
            autoAim.enableDebugLogging()

            telemetry.addLine("AutoAim GTSAM test ready")
            telemetry.update()

            waitForStart()
            if (isStopRequested) {
                return
            }

            ioLoop { _, dt ->
                val visionResult = visionTracker.read()
                autoAim.update(io.position(), turret.pos, visionResult)

                // Keep field-relative turret aiming aligned with the current robot heading.
                turret.robotHeading = autoAim.fusedPose.rot
                turret.robotAngularVelocity = io.velocity().rot

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
                telemetry.addData("Tag ID", autoAim.trackedTagId)
                telemetry.addData("Detections", autoAim.detectedTagCount)
                telemetry.addData("Tx/Ty (deg)", "%.2f / %.2f", autoAim.rawTxDegrees, autoAim.rawTyDegrees)
                telemetry.update()

                autoAim.logLandmarkCorners(22)
                println("CALLING MANUALLY")
                autoAim.logCameraUnitVectors()

                false
            }
        } finally {
            autoAim?.close()
            ll.stop()
        }
    }
}
