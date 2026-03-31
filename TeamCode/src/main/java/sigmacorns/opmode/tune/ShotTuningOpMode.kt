package sigmacorns.opmode.tune

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.launch
import sigmacorns.Robot
import sigmacorns.control.aim.tune.AdaptiveTuner
import sigmacorns.control.aim.tune.ShotDataStore
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.PI

/**
 * Shot tuning OpMode with embedded web interface.
 *
 * Place the robot at a known distance from the goal, then use the web UI
 * to adjust flywheel speed and hood angle, fire test shots, and save
 * the optimal (distance, speed, hoodAngle) data points.
 *
 * The web UI is accessible at http://<robot-ip>:8080 from the driver
 * station laptop (typically http://192.168.43.1:8080 on WiFi Direct).
 *
 * Saved data persists to /sdcard/FIRST/shot_tuning_data.json and is
 * used by the main TeleOp for automatic flywheel speed and hood angle
 * based on distance to goal.
 */
@TeleOp(name = "Shot Tuning", group = "Tune")
class ShotTuningOpMode : SigmaOpMode() {

    private var tuningDistance = 2.0
    private var flywheelTarget = 400.0
    private var hoodAngleDeg = 45.0
    @Volatile private var shootRequested = false

    override fun runOpMode() {
        val robot = Robot(io, blue = false)
        robot.init(Pose2d(), apriltagTracking = false)
        robot.aimTurret = false
        robot.aimFlywheel = false
        robot.hood.autoAdjust = false

        val dataStore = ShotDataStore()
        dataStore.load()
        val tuner = AdaptiveTuner(dataStore)

        // Start web server
        val server = ShotTuningServer(port = 8080, handler = object : ShotTuningServer.RequestHandler {
            override fun getState() = ShotTuningServer.ShotTuningState(
                flywheelTarget = flywheelTarget,
                flywheelActual = io.flywheelVelocity(),
                flywheelRPM = io.flywheelVelocity() * 60.0 / (2.0 * PI),
                flywheelPower = io.flywheel,
                hoodAngle = hoodAngleDeg,
                hoodServo = robot.hood.currentServoPosition,
                distance = tuningDistance,
                isShooting = robot.intakeTransfer.isTransferring,
                ballCount = robot.beamBreak.ballCount
            )

            override fun shoot() { shootRequested = true }
            override fun setDistance(distance: Double) { tuningDistance = distance }
            override fun setFlywheelTarget(speed: Double) { flywheelTarget = speed }
            override fun setHoodAngle(angle: Double) { hoodAngleDeg = angle }

            override fun savePoint() {
                tuner.addPoint(tuningDistance, flywheelTarget, hoodAngleDeg)
                dataStore.save()
            }

            override fun deletePoint(index: Int) {
                tuner.removePoint(index)
                dataStore.save()
            }

            override fun getPoints(): String = dataStore.exportPointsJson()
            override fun saveToFile() = dataStore.save()
        })

        server.start()

        telemetry.addLine("Shot Tuning OpMode")
        telemetry.addLine("Web UI: http://192.168.43.1:8080")
        telemetry.addLine("Or check robot IP in WiFi settings")
        telemetry.addLine("Waiting for start...")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Apply flywheel target
            robot.flywheel.target = flywheelTarget
            robot.flywheel.update(io.flywheelVelocity(), dt)

            // Apply hood angle (manual from web UI)
            robot.hood.manualAngle = Math.toRadians(hoodAngleDeg)

            // Handle shoot request from web UI
            if (shootRequested) {
                shootRequested = false
                robot.scope.launch {
                    robot.intakeTransfer.startTransfer()
                    kotlinx.coroutines.delay(1500)
                    robot.intakeTransfer.stopTransfer()
                }
            }

            // Update subsystems
            robot.update()
            io.update()

            // Also allow gamepad control for convenience
            // D-pad up/down: adjust distance
            if (gamepad1.dpad_up) {
                tuningDistance += 0.01
            }
            if (gamepad1.dpad_down) {
                tuningDistance = (tuningDistance - 0.01).coerceAtLeast(0.1)
            }

            // Bumpers: adjust flywheel
            if (gamepad1.right_bumper) {
                flywheelTarget = (flywheelTarget + 1.0).coerceAtMost(628.0)
            }
            if (gamepad1.left_bumper) {
                flywheelTarget = (flywheelTarget - 1.0).coerceAtLeast(50.0)
            }

            // A: shoot from gamepad
            if (gamepad1.a) {
                shootRequested = true
            }

            // X: save point from gamepad
            if (gamepad1.x) {
                tuner.addPoint(tuningDistance, flywheelTarget, hoodAngleDeg)
                dataStore.save()
                while (gamepad1.x && opModeIsActive()) { idle() }
            }

            // Telemetry
            val flywheelVel = io.flywheelVelocity()
            val flywheelRPM = flywheelVel * 60.0 / (2.0 * PI)

            telemetry.addLine("=== SHOT TUNING ===")
            telemetry.addData("Web UI", "http://192.168.43.1:8080")
            telemetry.addData("Server", if (server.running) "RUNNING" else "STOPPED")
            telemetry.addLine("")
            telemetry.addData("Distance", "%.2f m (dpad up/down)", tuningDistance)
            telemetry.addData("FW Target", "%.0f rad/s (bumpers)", flywheelTarget)
            telemetry.addData("FW Actual", "%.0f rad/s", flywheelVel)
            telemetry.addData("FW RPM", "%.0f", flywheelRPM)
            telemetry.addData("FW Power", "%.3f", io.flywheel)
            telemetry.addData("Hood Angle", "%.1f°", hoodAngleDeg)
            telemetry.addData("Hood Servo", "%.3f", robot.hood.currentServoPosition)
            telemetry.addData("Balls", robot.beamBreak.ballCount)
            telemetry.addData("Data Points", tuner.pointCount())
            telemetry.addLine("")
            telemetry.addLine("GP1: dpad=dist, bumper=fw, A=shoot, X=save")
            telemetry.update()

            false
        }

        server.stop()
        robot.close()
    }
}
