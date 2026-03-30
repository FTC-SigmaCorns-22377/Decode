package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import sigmacorns.constants.flywheelMotor
import sigmacorns.subsystem.Flywheel
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.tune.FlywheelDeadbeatConfig
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.time.Duration.Companion.milliseconds

/**
 * Test OpMode for shooter experiments.
 *
 * Lets you adjust flywheel RPM independently to see how different RPMs
 * affect ball trajectory. Uses the blocker servo to release balls into
 * the flywheel, and beam break sensors to detect ball presence.
 *
 * WORKFLOW:
 * 1. Hold LEFT TRIGGER to run intake and bring balls in
 * 2. Adjust target RPM using left stick or D-pad
 * 3. Press A to enable flywheel at target RPM
 * 4. Press RIGHT TRIGGER to SHOOT - opens blocker to release ball
 *
 * CONTROLS:
 * - Left stick Y: Adjust flywheel RPM (up = faster)
 * - A button: Toggle flywheel on/off at current RPM
 * - B button: Reset RPM to zero and disable flywheel
 * - D-pad up/down: Fine adjust RPM (+/- 50)
 * - Right stick Y: Manual flywheel power override
 * - LEFT TRIGGER: Hold to run intake
 * - RIGHT TRIGGER: Hold to SHOOT - opens blocker servo
 * - Y button: Hold to eject (reverse intake)
 */
@TeleOp(name = "Shooter Experiment Test", group = "Test")
class ShooterExperimentTest : SigmaOpMode() {

    /** Target flywheel velocity in RPM */
    private var targetRPM: Double = 0.0

    /** Whether flywheel is actively spinning */
    private var flywheelEnabled: Boolean = false

    private val maxRPM = 6000.0
    private val minRPM = 0.0

    private val rpmContinuousRate = 1000.0  // RPM per second at full stick
    private val rpmFineRate = 200.0         // RPM per second in fine mode

    // Blocker servo positions
    private val BLOCKER_CLOSED = 0.0   // blocking - ball held
    private val BLOCKER_OPEN = 1.0     // open - ball released to flywheel

    private lateinit var flywheel: Flywheel
    private lateinit var gm1: Gamepad

    // Debounce flags
    private var aPressed = false
    private var bPressed = false
    private var dpadUpPressed = false
    private var dpadDownPressed = false

    // Voltage compensation
    private var dVoltage = 1.0

    override fun runOpMode() {
        gm1 = gamepad1

        flywheel = Flywheel(
            motor = flywheelMotor,
            inertia = FlywheelDeadbeatConfig.inertia,
            io = io,
            lag = FlywheelDeadbeatConfig.lagMs.milliseconds
        )

        telemetry.addLine("=== SHOOTER EXPERIMENT TEST ===")
        telemetry.addLine("")
        telemetry.addLine("Left stick Y: Adjust RPM")
        telemetry.addLine("A: Toggle flywheel on/off")
        telemetry.addLine("B: Reset all to zero")
        telemetry.addLine("D-pad up/down: Fine RPM (+/- 50)")
        telemetry.addLine("Right stick Y: Manual power override")
        telemetry.addLine("")
        telemetry.addLine("LEFT TRIGGER: Hold to intake")
        telemetry.addLine("RIGHT TRIGGER: Hold to SHOOT")
        telemetry.addLine("Y: Hold to eject")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            dVoltage = 12.0 / io.voltage()

            val dtSeconds = dt.inWholeMilliseconds / 1000.0

            // === FLYWHEEL RPM CONTROL ===

            // Continuous adjustment via left stick
            val stickY = gm1.left_stick_y.toDouble()
            if (stickY.absoluteValue > 0.05) {
                targetRPM += -stickY * rpmContinuousRate * dtSeconds
                targetRPM = targetRPM.coerceIn(minRPM, maxRPM)
            }

            // Fine adjustment via D-pad
            if (gm1.dpad_up && !dpadUpPressed) {
                targetRPM = (targetRPM + 50.0).coerceAtMost(maxRPM)
            }
            dpadUpPressed = gm1.dpad_up

            if (gm1.dpad_down && !dpadDownPressed) {
                targetRPM = (targetRPM - 50.0).coerceAtLeast(minRPM)
            }
            dpadDownPressed = gm1.dpad_down

            // === TOGGLE / RESET ===

            if (gm1.a && !aPressed) {
                flywheelEnabled = !flywheelEnabled
            }
            aPressed = gm1.a

            if (gm1.b && !bPressed) {
                targetRPM = 0.0
                flywheelEnabled = false
            }
            bPressed = gm1.b

            // === INTAKE CONTROL ===
            val intaking = gm1.left_trigger > 0.1
            val ejecting = gm1.y

            io.intake = when {
                ejecting -> 1.0    // reverse to eject
                intaking -> -1.0   // forward to intake
                else -> 0.0
            }

            // === SHOOT CONTROL (Right trigger = open blocker) ===
            val isShooting = gm1.right_trigger > 0.5
            io.blocker = if (isShooting) BLOCKER_OPEN else BLOCKER_CLOSED

            // === APPLY FLYWHEEL CONTROL ===
            val manualOverride = gm1.right_stick_y.toDouble().absoluteValue > 0.1

            if (manualOverride) {
                io.flywheel = -gm1.right_stick_y.toDouble() * dVoltage
            } else if (flywheelEnabled || isShooting) {
                val targetRadPerSec = targetRPM * 2.0 * PI / 60.0
                flywheel.target = targetRadPerSec
                flywheel.hold = false
                flywheel.update(io.flywheelVelocity(), dt)
            } else {
                io.flywheel = 0.0
            }

            // === TELEMETRY OUTPUT ===
            val actualRPM = io.flywheelVelocity() * 60.0 / (2.0 * PI)
            val targetRadPerSec = targetRPM * 2.0 * PI / 60.0
            val errorRPM = targetRPM - actualRPM

            telemetry.addLine("========================================")
            telemetry.addLine("    SHOOTER EXPERIMENT VALUES")
            telemetry.addLine("========================================")
            telemetry.addLine("")
            telemetry.addData(">>> TARGET RPM", "%.0f".format(targetRPM))
            telemetry.addData(">>> ACTUAL RPM", "%.0f".format(actualRPM))
            telemetry.addData(">>> ERROR RPM", "%.0f".format(errorRPM))
            telemetry.addData(">>> TARGET rad/s", "%.1f".format(targetRadPerSec))
            telemetry.addLine("")
            telemetry.addLine("----------------------------------------")
            telemetry.addData("Flywheel", if (flywheelEnabled) "ON" else "OFF")
            telemetry.addData("Motor Power", "%.2f".format(io.flywheel))
            telemetry.addData("Battery", "%.1f V".format(io.voltage()))
            telemetry.addData("Manual Override", if (manualOverride) "YES" else "NO")
            telemetry.addLine("")
            telemetry.addLine("--- BALL DETECTION ---")
            telemetry.addData("Beam Break 1", if (io.beamBreak1()) "BALL" else "empty")
            telemetry.addData("Beam Break 2", if (io.beamBreak2()) "BALL" else "empty")
            telemetry.addData("Beam Break 3", if (io.beamBreak3()) "BALL" else "empty")
            telemetry.addLine("")
            telemetry.addData("Blocker", if (isShooting) "OPEN" else "CLOSED")
            telemetry.addData("Intake Power", "%.2f".format(io.intake))
            telemetry.addData("Shooting", if (isShooting) "** FIRING **" else "ready")
            telemetry.addLine("----------------------------------------")
            telemetry.addLine("A=flywheel | B=reset")
            telemetry.addLine("LT=intake | RT=SHOOT | Y=eject")
            telemetry.addLine("RStick=manual power")
            telemetry.update()

            false // continue loop
        }
    }
}
