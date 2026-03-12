package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.constants.flywheelMotor
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.Flywheel
import kotlin.math.PI
import kotlin.time.Duration.Companion.milliseconds

/**
 * Test OpMode for shooter experiments.
 *
 * Lets you adjust flywheel RPM and hood arc angle independently,
 * displaying both values in telemetry for camera system integration.
 *
 * CONTROLS:
 * - Left stick Y: Adjust flywheel RPM (up = faster)
 * - Right stick Y: Adjust hood arc angle (up = increase)
 * - A button: Toggle flywheel on/off at current RPM
 * - B button: Reset both to zero
 * - D-pad up/down: Fine adjust RPM (+/- 50)
 * - D-pad left/right: Fine adjust arc angle (+/- 1 degree)
 * - Right bumper: Hold to use fine control mode (slower stick response)
 * - LEFT BUMPER: HOLD TO SHOOT (runs intake to feed ball)
 * - X button: Toggle feed direction (forward/reverse)
 * - Y button: Hold to eject (reverse intake)
 */
@TeleOp(name = "Shooter Experiment Test", group = "Test")
class ShooterExperimentTest : SigmaOpMode() {

    // ===== CONFIGURABLE PARAMETERS =====

    /** Target flywheel velocity in RPM */
    private var targetRPM: Double = 0.0

    /** Hood arc angle in degrees */
   private var arcAngleDegrees: Double = 0.0

    /** Whether flywheel is actively spinning *
    private var flywheelEnabled: Boolean = false

    /** Feed direction: 1.0 = forward, -1.0 = reverse */
    private var feedDirection: Double = 1.0

    // ===== LIMITS =====
    private val maxRPM = 6000.0
    private val minRPM = 0.0
    private val maxArcAngle = 90.0  // degrees - adjust to your hood's range
    private val minArcAngle = 0.0

    // ===== CONTROL RATES =====
    private val rpmContinuousRate = 1000.0      // RPM per second at full stick
    private val rpmFineRate = 200.0             // RPM per second in fine mode
    private val arcContinuousRate = 30.0        // degrees per second at full stick
    private val arcFineRate = 5.0               // degrees per second in fine mode

    private lateinit var flywheel: Flywheel

    // Debounce flags
    private var aPressed = false
    private var bPressed = false
    private var dpadUpPressed = false
    private var dpadDownPressed = false
    private var dpadLeftPressed = false
    private var dpadRightPressed = false
    private var xPressed = false

    override fun runOpMode() {
        flywheel = Flywheel(
            motor = flywheelMotor,
            inertia = 0.7,
            io = io,
            lag = 10.milliseconds
        )

        telemetry.addLine("=== SHOOTER EXPERIMENT TEST ===")
        telemetry.addLine("")
        telemetry.addLine("Left stick Y: Adjust RPM")
        telemetry.addLine("Right stick Y: Adjust arc angle")
        telemetry.addLine("A: Toggle flywheel on/off")
        telemetry.addLine("B: Reset all to zero")
        telemetry.addLine("D-pad: Fine adjustments")
        telemetry.addLine("Right bumper: Fine control mode")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            val dtSeconds = dt.inWholeMilliseconds / 1000.0
            val fineMode = gamepad1.right_bumper

            // === FLYWHEEL RPM CONTROL ===

            // Continuous adjustment via left stick
            val rpmRate = if (fineMode) rpmFineRate else rpmContinuousRate
            targetRPM += -gamepad1.left_stick_y * rpmRate * dtSeconds
            targetRPM = targetRPM.coerceIn(minRPM, maxRPM)

            // Fine adjustment via D-pad
            if (gamepad1.dpad_up && !dpadUpPressed) {
                targetRPM = (targetRPM + 50.0).coerceAtMost(maxRPM)
            }
            dpadUpPressed = gamepad1.dpad_up

            if (gamepad1.dpad_down && !dpadDownPressed) {
                targetRPM = (targetRPM - 50.0).coerceAtLeast(minRPM)
            }
            dpadDownPressed = gamepad1.dpad_down

            // === ARC ANGLE CONTROL ===

            // Continuous adjustment via right stick
            val arcRate = if (fineMode) arcFineRate else arcContinuousRate
            arcAngleDegrees += -gamepad1.right_stick_y * arcRate * dtSeconds
            arcAngleDegrees = arcAngleDegrees.coerceIn(minArcAngle, maxArcAngle)

            // Fine adjustment via D-pad
            if (gamepad1.dpad_right && !dpadRightPressed) {
                arcAngleDegrees = (arcAngleDegrees + 1.0).coerceAtMost(maxArcAngle)
            }
            dpadRightPressed = gamepad1.dpad_right

            if (gamepad1.dpad_left && !dpadLeftPressed) {
                arcAngleDegrees = (arcAngleDegrees - 1.0).coerceAtLeast(minArcAngle)
            }
            dpadLeftPressed = gamepad1.dpad_left

            // === TOGGLE / RESET ===

            // A button: toggle flywheel
            if (gamepad1.a && !aPressed) {
                flywheelEnabled = !flywheelEnabled
            }
            aPressed = gamepad1.a

            // B button: reset all
            if (gamepad1.b && !bPressed) {
                targetRPM = 0.0
                arcAngleDegrees = 0.0
                flywheelEnabled = false
            }
            bPressed = gamepad1.b

            // X button: toggle feed direction
            if (gamepad1.x && !xPressed) {
                feedDirection *= -1.0
            }
            xPressed = gamepad1.x

            // === SHOOT CONTROL ===
            // Left bumper = SHOOT (feed ball into flywheel)
            // Y button = EJECT (reverse intake)
            val shooting = gamepad1.left_bumper
            val ejecting = gamepad1.y

            io.intake = when {
                shooting -> feedDirection  // Feed into flywheel
                ejecting -> -1.0           // Eject
                else -> 0.0
            }

            // === APPLY FLYWHEEL CONTROL ===

            val targetRadPerSec = if (flywheelEnabled) targetRPM * 2.0 * PI / 60.0 else 0.0
            flywheel.target = targetRadPerSec
            flywheel.update(io.flywheelVelocity(), dt)

            // === TELEMETRY OUTPUT ===
            // These values are what you'd read for your camera system

            val actualRPM = io.flywheelVelocity() * 60.0 / (2.0 * PI)

            telemetry.addLine("========================================")
            telemetry.addLine("    SHOOTER EXPERIMENT VALUES")
            telemetry.addLine("========================================")
            telemetry.addLine("")
            telemetry.addData(">>> TARGET RPM", "%.0f".format(targetRPM))
            telemetry.addData(">>> ACTUAL RPM", "%.0f".format(actualRPM))
            telemetry.addData(">>> ARC ANGLE (deg)", "%.1f".format(arcAngleDegrees))
            telemetry.addLine("")
            telemetry.addLine("----------------------------------------")
            telemetry.addData("Flywheel", if (flywheelEnabled) "ON" else "OFF")
            telemetry.addData("Motor Power", "%.2f".format(io.flywheel))
            telemetry.addLine("")
            telemetry.addData(">>> SHOOTING", if (shooting) "** FIRING **" else "ready")
            telemetry.addData("Feed Direction", if (feedDirection > 0) "FORWARD" else "REVERSE")
            telemetry.addData("Intake Power", "%.2f".format(io.intake))
            telemetry.addLine("")
            telemetry.addData("Battery", "%.1f V".format(io.voltage()))
            telemetry.addData("Mode", if (fineMode) "FINE" else "NORMAL")
            telemetry.addLine("----------------------------------------")
            telemetry.addLine("A=flywheel | B=reset | LB=SHOOT")
            telemetry.addLine("X=flip feed | Y=eject | RB=fine")
            telemetry.update()

            false // continue loop
        }
    }
}
