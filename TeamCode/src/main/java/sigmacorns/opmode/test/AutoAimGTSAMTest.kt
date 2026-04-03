package sigmacorns.opmode.test

import com.bylazar.panels.Panels
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

@TeleOp(name = "AutoAim GTSAM Test", group = "Test")
class AutoAimGTSAMTest : SigmaOpMode() {
    override fun runOpMode() {
        Panels.config.enableLogs = false

        val robot = Robot(io, blue = true)
        robot.aimFlywheel = false
        robot.init(Pose2d(0.0, 0.0, PI / 2.0), apriltagTracking = true)
        robot.startApriltag()
        robot.turret.fieldRelativeMode = true
        robot.aimTurret = true

        robot.aim.autoAim.enableDebugLogging()

        telemetry.addLine("AutoAim GTSAM test ready")
        telemetry.update()

        waitForStart()
        if (isStopRequested) return

        ioLoop { _, dt ->
            robot.drive.update(gamepad1, io)
            robot.update()
            io.update()

            val autoAim = robot.aim.autoAim
            val fusedPose = autoAim.fusedPose
            telemetry.addData(
                "Fused Pose (m, m, rad)",
                "%.2f, %.2f, %.2f",
                fusedPose.v.x, fusedPose.v.y, fusedPose.rot
            )
            telemetry.addData("Vision Target", autoAim.hasVisionTarget)
            telemetry.addData("Tag ID", autoAim.trackedTagId)
            telemetry.addData("Detections", autoAim.detectedTagCount)
            telemetry.addData("Tx/Ty (deg)", "%.2f / %.2f", autoAim.rawTxDegrees, autoAim.rawTyDegrees)
            telemetry.addData("Turret (deg)", "%.1f", Math.toDegrees(robot.turret.pos))
            telemetry.addData("Goal Dist", "%.2f m", robot.aim.targetDistance)
            telemetry.update()

            false
        }

        robot.close()
    }
}
