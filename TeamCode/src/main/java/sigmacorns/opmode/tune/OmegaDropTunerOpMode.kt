package sigmacorns.opmode.tune

import com.google.gson.GsonBuilder
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import java.io.File
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * Measures flywheel omega drop per shot across a range of omega targets.
 *
 * Controls:
 *   X            — fire one ball (spins up first if needed, then transfers)
 *   Y            — increase target omega by 20 rad/s
 *   B            — decrease target omega by 20 rad/s
 *   A            — save samples to file
 *
 * Telemetry shows pre-shot omega, post-shot omega, and measured drop fraction.
 * Saves all samples to /sdcard/FIRST/omega_drop_samples.json on A press.
 */
@TeleOp(name = "Omega Drop Tuner", group = "Tune")
class OmegaDropTunerOpMode : SigmaOpMode() {

    companion object {
        const val SETTLE_TOLERANCE_RAD_S = 5.0
        const val SETTLE_WINDOW_S = 0.5
        const val POST_SHOT_DELAY_LOOPS = 4
        val SAVE_FILE = "/sdcard/FIRST/omega_drop_samples.json"
        const val OMEGA_STEP = 20.0
        const val OMEGA_MIN = 100.0
        const val OMEGA_MAX = 700.0
        const val HOOD_STEP = 5.0
    }

    data class DropSample(
        val targetOmega: Double,
        val hoodAngleDeg: Double,
        val preOmega: Double,
        val postOmega: Double,
        val dropFraction: Double
    )

    private val samples = mutableListOf<DropSample>()

    private enum class Phase { IDLE, SPINNING_UP, SETTLING, FIRED, RECORDING }

    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = false)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = false)
        robot.aimFlywheel = false
        robot.shooter.autoAdjust = false

        var targetOmega = 400.0
        var hoodAngleDeg = ShooterConfig.minAngleDeg
        var phase = Phase.IDLE
        var settleTimer = 0.0
        var preOmega = 0.0
        var postLoops = 0

        var lastX = false; var lastY = false; var lastB = false; var lastA = false
        var lastLB = false; var lastRB = false

        telemetry.addLine("Omega Drop Tuner — waiting for start")
        telemetry.update()
        waitForStart()

        ioLoop { _, dt ->
            val curOmega = io.flywheelVelocity()
            val dtSec = dt.inWholeMilliseconds / 1000.0

            // ── Target adjustment ──────────────────────────────────────
            if (gamepad1.y && !lastY) targetOmega = (targetOmega + OMEGA_STEP).coerceAtMost(OMEGA_MAX)
            if (gamepad1.b && !lastB) targetOmega = (targetOmega - OMEGA_STEP).coerceAtLeast(OMEGA_MIN)
            lastY = gamepad1.y; lastB = gamepad1.b
            if (gamepad1.right_bumper && !lastRB) hoodAngleDeg = (hoodAngleDeg + HOOD_STEP).coerceAtMost(ShooterConfig.maxAngleDeg)
            if (gamepad1.left_bumper && !lastLB)  hoodAngleDeg = (hoodAngleDeg - HOOD_STEP).coerceAtLeast(ShooterConfig.minAngleDeg)
            lastRB = gamepad1.right_bumper; lastLB = gamepad1.left_bumper
            robot.shooter.manualHoodAngle = Math.toRadians(hoodAngleDeg)

            // ── Save ───────────────────────────────────────────────────
            if (gamepad1.a && !lastA && samples.isNotEmpty()) {
                try {
                    val gson = GsonBuilder().setPrettyPrinting().create()
                    File(SAVE_FILE).writeText(gson.toJson(samples))
                    telemetry.addLine("Saved ${samples.size} samples to $SAVE_FILE")
                } catch (e: Exception) {
                    telemetry.addLine("Save failed: ${e.message}")
                }
            }
            lastA = gamepad1.a

            // ── State machine ──────────────────────────────────────────
            when (phase) {
                Phase.IDLE -> {
                    robot.shooter.flywheelTarget = 0.0
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                    if (gamepad1.x && !lastX) {
                        phase = Phase.SPINNING_UP
                        settleTimer = 0.0
                    }
                }
                Phase.SPINNING_UP -> {
                    robot.shooter.flywheelTarget = targetOmega
                    val err = abs(curOmega - targetOmega)
                    if (err < SETTLE_TOLERANCE_RAD_S) {
                        settleTimer += dtSec
                        if (settleTimer >= SETTLE_WINDOW_S) {
                            phase = Phase.SETTLING
                            settleTimer = 0.0
                        }
                    } else {
                        settleTimer = 0.0
                    }
                }
                Phase.SETTLING -> {
                    robot.shooter.flywheelTarget = targetOmega
                    // One more loop at speed to latch pre-omega cleanly
                    preOmega = curOmega
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                    phase = Phase.FIRED
                    postLoops = 0
                }
                Phase.FIRED -> {
                    robot.shooter.flywheelTarget = targetOmega
                    postLoops++
                    if (postLoops >= POST_SHOT_DELAY_LOOPS) {
                        phase = Phase.RECORDING
                    }
                }
                Phase.RECORDING -> {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                    val postOmega = curOmega
                    val drop = if (preOmega > 1.0) (preOmega - postOmega) / preOmega else 0.0
                    samples.add(DropSample(targetOmega, hoodAngleDeg, preOmega, postOmega, drop))
                    phase = Phase.IDLE
                    robot.shooter.flywheelTarget = 0.0
                }
            }
            lastX = gamepad1.x

            robot.shooter.update(dt)
            robot.intakeTransfer.update(dt, io.time())

            // ── Telemetry ──────────────────────────────────────────────
            telemetry.addData("Phase", phase)
            telemetry.addData("Target omega (Y+/B-)", "${targetOmega.roundToInt()} rad/s")
            telemetry.addData("Hood angle (LB-/RB+)", "%.1f°".format(hoodAngleDeg))
            telemetry.addData("Actual omega", "${curOmega.roundToInt()} rad/s")
            telemetry.addData("Settle error", "%.1f rad/s".format(abs(curOmega - targetOmega)))
            if (samples.isNotEmpty()) {
                val last = samples.last()
                telemetry.addLine("--- Last shot ---")
                telemetry.addData("Pre-shot omega", "%.1f rad/s".format(last.preOmega))
                telemetry.addData("Post-shot omega", "%.1f rad/s".format(last.postOmega))
                telemetry.addData("Drop fraction", "%.3f (%.1f%%)".format(last.dropFraction, last.dropFraction * 100))
            }
            if (samples.size >= 3) {
                val avgDrop = samples.takeLast(5).map { it.dropFraction }.average()
                telemetry.addData("Avg drop (last 5)", "%.3f".format(avgDrop))
            }
            telemetry.addData("Samples", samples.size)
            telemetry.addLine("[X] shoot  [Y/B] omega  [LB/RB] hood  [A] save")
            telemetry.update()

            false
        }
    }
}
