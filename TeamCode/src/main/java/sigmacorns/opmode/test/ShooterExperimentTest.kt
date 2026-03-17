package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import sigmacorns.constants.flywheelMotor
import sigmacorns.control.subsystem.Flywheel
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.tune.FlywheelDeadbeatConfig
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.time.Duration.Companion.milliseconds

/**
 * Test OpMode for shooter experiments.
 *
 * Lets you adjust flywheel RPM independently to see how different RPMs
 * affect ball trajectory. The spindexer/transfer system handles ball
 * feeding end-to-end.
 *
 * WORKFLOW:
 * 1. Hold LEFT TRIGGER to run intake and bring balls into spindexer
 * 2. Adjust target RPM using left stick or D-pad
 * 3. Press A to enable flywheel at target RPM
 * 4. Press RIGHT TRIGGER to SHOOT - activates transfer to fire ball through flywheel
 *
 * CONTROLS:
 * - Left stick Y: Adjust flywheel RPM (up = faster)
 * - A button: Toggle flywheel on/off at current RPM
 * - B button: Reset RPM to zero and disable flywheel
 * - D-pad up/down: Fine adjust RPM (+/- 50)
 * - Right stick Y: Manual flywheel power override
 * - LEFT TRIGGER: Hold to run intake (feeds balls into spindexer)
 * - RIGHT TRIGGER: Hold to SHOOT - activates transfer to fire ball
 * - LEFT BUMPER: Nudge spindexer CCW
 * - RIGHT BUMPER: Nudge spindexer CW
 * - Y button: Hold to eject (reverse intake)
 */
@TeleOp(name = "Shooter Experiment Test", group = "Test")
class ShooterExperimentTest : SigmaOpMode() {

    // ===== CONFIGURABLE PARAMETERS =====

    /** Target flywheel velocity in RPM */
    private var targetRPM: Double = 0.0

    /** Whether flywheel is actively spinning */
    private var flywheelEnabled: Boolean = false

    // ===== LIMITS =====
    private val maxRPM = 6000.0
    private val minRPM = 0.0

    // ===== CONTROL RATES =====
    private val rpmContinuousRate = 1000.0      // RPM per second at full stick
    private val rpmFineRate = 200.0             // RPM per second in fine mode

    // Subsystems
    private lateinit var flywheel: Flywheel
    private lateinit var spindexerLogic: SpindexerLogic

    // Gamepad references
    private lateinit var gm1: Gamepad

    // Debounce flags
    private var aPressed = false
    private var bPressed = false
    private var dpadUpPressed = false
    private var dpadDownPressed = false
    private var wasIntaking = false
    private var wasShooting = false
    private var wasLeftBumper = false
    private var wasRightBumper = false

    // Voltage compensation
    private var dVoltage = 1.0

    override fun runOpMode() {
        // Get gamepad reference
        gm1 = gamepad1

        // Create flywheel controller using deadbeat control
        flywheel = Flywheel(
            motor = flywheelMotor,
            inertia = FlywheelDeadbeatConfig.inertia,
            io = io,
            lag = FlywheelDeadbeatConfig.lagMs.milliseconds
        )

        // Create spindexer logic WITHOUT flywheel - we'll control flywheel directly
        // This prevents SpindexerLogic from overriding our manual flywheel control
        spindexerLogic = SpindexerLogic(io, null)

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
        telemetry.addLine("Bumpers: Nudge spindexer L/R")
        telemetry.addLine("Y: Hold to eject")
        telemetry.update()

        waitForStart()

        ioLoop { state, dt ->
            // Update voltage compensation
            dVoltage = 12.0 / io.voltage()

            val dtSeconds = dt.inWholeMilliseconds / 1000.0
            val fineMode = gm1.left_bumper && gm1.right_bumper  // Both bumpers = fine mode

            // === FLYWHEEL RPM CONTROL ===

            // Continuous adjustment via left stick
            val stickY = gm1.left_stick_y.toDouble()
            if (stickY.absoluteValue > 0.05) {
                val rpmRate = if (fineMode) rpmFineRate else rpmContinuousRate
                targetRPM += -stickY * rpmRate * dtSeconds
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

            // A button: toggle flywheel
            if (gm1.a && !aPressed) {
                flywheelEnabled = !flywheelEnabled
            }
            aPressed = gm1.a

            // B button: reset all
            if (gm1.b && !bPressed) {
                targetRPM = 0.0
                flywheelEnabled = false
            }
            bPressed = gm1.b

            // === INTAKE CONTROL ===
            // Left trigger = run intake to bring balls into spindexer
            val intaking = gm1.left_trigger > 0.1
            if (intaking && !wasIntaking) {
                spindexerLogic.startIntaking()
            } else if (!intaking && wasIntaking) {
                spindexerLogic.stopIntaking()
            }
            wasIntaking = intaking

            // Y button = eject (reverse intake manually)
            val ejecting = gm1.y
            if (ejecting) {
                io.intake = 1.0  // Reverse to eject
            }

            // === SPINDEXER NUDGE ===
            // Bumpers nudge spindexer (only when not both pressed for fine mode)
            if (!fineMode) {
                if (gm1.left_bumper && !wasLeftBumper) {
                    spindexerLogic.nudge(true)  // CCW
                }
                wasLeftBumper = gm1.left_bumper

                if (gm1.right_bumper && !wasRightBumper) {
                    spindexerLogic.nudge(false)  // CW
                }
                wasRightBumper = gm1.right_bumper
            }

            // === SHOOT CONTROL (Right trigger) ===
            val isShooting = gm1.right_trigger > 0.5
            spindexerLogic.shootingRequested = isShooting

            if (isShooting && !wasShooting) {
                // Set the shot velocity to our target RPM
                val targetRadPerSec = targetRPM * 2.0 * PI / 60.0
                spindexerLogic.shotVelocity = targetRadPerSec
                spindexerLogic.shoot()
            }
            wasShooting = isShooting

            // === UPDATE SPINDEXER LOGIC ===
            // This handles FSM, spindexer motor, transfer
            // Since we passed null for flywheel, it uses PID internally but we override below
            spindexerLogic.update(dt)

            // === APPLY FLYWHEEL CONTROL ===
            // We control flywheel directly, overriding SpindexerLogic's PID output

            // Manual override with right stick
            val manualOverride = gm1.right_stick_y.toDouble().absoluteValue > 0.1

            if (manualOverride) {
                // Direct power control via right stick
                io.shooter = -gm1.right_stick_y.toDouble() * dVoltage
            } else if (flywheelEnabled || isShooting) {
                // Use deadbeat controller for precise RPM
                val targetRadPerSec = targetRPM * 2.0 * PI / 60.0
                flywheel.target = targetRadPerSec
                flywheel.hold = false  // Use deadbeat control for precise velocity
                flywheel.update(io.flywheelVelocity(), dt)
            } else {
                // Flywheel off - coast to stop
                io.shooter = 0.0
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
            telemetry.addData("Motor Power", "%.2f".format(io.shooter))
            telemetry.addData("Battery", "%.1f V".format(io.voltage()))
            telemetry.addData("Manual Override", if (manualOverride) "YES" else "NO")
            telemetry.addLine("")
            telemetry.addLine("--- SPINDEXER/SHOOTER STATE ---")
            telemetry.addData("FSM State", spindexerLogic.currentState.name)
            telemetry.addData("Spindexer Pos", "%.1f deg".format(Math.toDegrees(spindexerLogic.spindexer.curRotation)))
            telemetry.addData("Spindexer Target", "%.1f deg".format(Math.toDegrees(spindexerLogic.spindexerRotation)))
            telemetry.addData("Transfer", "%.2f".format(io.transfer))
            telemetry.addLine("")

            // Ball tracking
            val ballCount = spindexerLogic.spindexerState.count { it != null }
            val ballDisplay = spindexerLogic.spindexerState.mapIndexed { i, ball ->
                when (ball) {
                    null -> "[ ]"
                    else -> "[${ball.name.first()}]"
                }
            }.joinToString(" ")
            telemetry.addData("Balls", "$ballCount/3  $ballDisplay")
            telemetry.addData("Color Sensor", if (io.colorSensorDetectsBall()) "BALL DETECTED" else "empty")
            telemetry.addLine("")

            telemetry.addData("Intake Power", "%.2f".format(io.intake))
            telemetry.addData("Shooting", if (isShooting) "** FIRING **" else "ready")
            telemetry.addLine("----------------------------------------")
            telemetry.addLine("A=flywheel | B=reset")
            telemetry.addLine("LT=intake | RT=SHOOT | Y=eject")
            telemetry.addLine("Bumpers=nudge | RStick=manual power")
            telemetry.update()

            false // continue loop
        }
    }
}
