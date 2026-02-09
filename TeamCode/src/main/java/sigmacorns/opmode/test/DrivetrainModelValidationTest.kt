package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.Vector2d
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.subsystem.DriveController
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MECANUM_DT
import sigmacorns.sim.MecanumDynamics
import sigmacorns.sim.MecanumState
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.time.Duration
import kotlin.time.DurationUnit

@TeleOp(name = "Drivetrain Model Validation", group = "test")
class DrivetrainModelValidationTest : SigmaOpMode() {
    private val dynamics = MecanumDynamics(drivetrainParameters)
    private val driveController = DriveController()

    override fun runOpMode() {
        telemetry.addLine("Drivetrain model validation ready")
        telemetry.addLine("LB: zero pose, RB: toggle scripted drive")
        telemetry.update()

        waitForStart()

        val maxSpeed = dynamics.maxSpeed()
        var modelState = MecanumState(Pose2d(), Pose2d())
        var modelInitialized = false
        var lastCommand = DoubleArray(4)

        val actualTrace = ArrayDeque<Vector2d>()
        val predictedTrace = ArrayDeque<Vector2d>()

        var autoEnabled = true
        var prevAutoToggle = false
        var prevZeroButton = false
        var logCounter = 0

        rerunSink("DrivetrainModelValidationTest").use { rerun ->
            ioLoop { state, dt ->
                if (!modelInitialized) {
                    modelState = state.mecanumState
                    modelInitialized = true
                } else {
                    val dtSeconds = dt.toDouble(DurationUnit.SECONDS)
                    if (dtSeconds > 1e-6) {
                        modelState = dynamics.integrate(dtSeconds, MECANUM_DT, lastCommand, modelState)
                    }
                }

                val zeroPressed = gamepad1.left_bumper
                if (zeroPressed && !prevZeroButton) {
                    io.setPosition(Pose2d())
                    modelInitialized = false
                }
                prevZeroButton = zeroPressed

                val autoToggle = gamepad1.right_bumper
                if (autoToggle && !prevAutoToggle) {
                    autoEnabled = !autoEnabled
                }
                prevAutoToggle = autoToggle

                val normalizedCommand = if (autoEnabled) {
                    scriptedCommand(state.timestamp)
                } else {
                    manualCommand()
                }

                val boundedCommand = clampPose(normalizedCommand, 0.75)

                lastCommand = driveController.drive(boundedCommand, io)

                val poseError = state.driveTrainPosition - modelState.pos
                val positionErrorNorm = hypot(poseError.v.x, poseError.v.y)
                val headingErrorRad = normalizeAngle(poseError.rot)
                val velocityError = state.driveTrainVelocity - modelState.vel
                val velocityErrorNorm = hypot(velocityError.v.x, velocityError.v.y)

                telemetry.addData("mode", if (autoEnabled) "scripted sine sweep" else "manual")
                telemetry.addData("pos err (cm)", "%.1f", positionErrorNorm * 100.0)
                telemetry.addData("heading err (deg)", "%.2f", Math.toDegrees(headingErrorRad))
                telemetry.addData("vel err (cm/s)", "%.1f", velocityErrorNorm * 100.0)
                telemetry.addData("command", "%.2f, %.2f, %.2f", boundedCommand.v.x, boundedCommand.v.y, boundedCommand.rot)
                telemetry.update()

                actualTrace.addSample(Vector2d(state.driveTrainPosition.v))
                predictedTrace.addSample(Vector2d(modelState.pos.v))

                rerun.logState(state)
                rerun.logInputs(io)

                if (rerun.isConnected && logCounter++ % 5 == 0) {
                    if (actualTrace.isNotEmpty()) {
                        rerun.logLineStrip("drivetrain/actual", actualTrace.toList())
                    }
                    if (predictedTrace.isNotEmpty()) {
                        rerun.logLineStrip("drivetrain/predicted", predictedTrace.toList())
                    }
                }

                false
            }
        }
    }

    private fun manualCommand(): Pose2d {
        return Pose2d(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble(),
        )
    }

    private fun scriptedCommand(timestamp: Duration): Pose2d {
        val t = timestamp.toDouble(DurationUnit.SECONDS)
        val vx = 0.7 * sin(2.0 * PI * t / 6.0)
        val vy = 0.6 * sin(2.0 * PI * t / 5.0 + PI / 2.0)
        val omega = 0.5 * sin(2.0 * PI * t / 4.0)
        return Pose2d(vx, vy, 0.0)
    }

    private fun clampPose(pose: Pose2d, limit: Double): Pose2d {
        val scale = maxOf(
            maxOf(abs(pose.v.x), abs(pose.v.y)),
            abs(pose.rot),
        )
        return if (scale <= limit || scale <= 1e-6) {
            pose
        } else {
            val ratio = limit / scale
            Pose2d(pose.v.x * ratio, pose.v.y * ratio, pose.rot * ratio)
        }
    }

    private fun normalizeAngle(angle: Double): Double {
        if (!angle.isFinite()) return 0.0
        var wrapped = angle
        while (wrapped > PI) wrapped -= 2.0 * PI
        while (wrapped < -PI) wrapped += 2.0 * PI
        return wrapped
    }

    private fun ArrayDeque<Vector2d>.addSample(sample: Vector2d, limit: Int = 400) {
        addLast(sample)
        while (size > limit) {
            removeFirst()
        }
    }
}