package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.constants.flywheelMotor
import sigmacorns.control.subsystem.Flywheel
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.tune.FlywheelDeadbeatConfig
import kotlin.math.PI
import kotlin.time.Duration.Companion.milliseconds

/**
 * Test OpMode for shooter experiments.
 *
 * Lets you adjust flywheel RPM independently to see how different RPMs
 * affect ball trajectory. The spindexer/transfer system handles ball
 * feeding end-to-end.
 *
 * WORKFLOW:
 * 1. Hold LEFT BUMPER to run intake and bring balls into spindexer
 * 2. Adjust target RPM using left stick or D-pad
 * 3. Press A to enable flywheel at target RPM
 * 4. Press X to SHOOT - activates transfer to fire ball through flywheel
 *
 * CONTROLS:
 * - Left stick Y: Adjust flywheel RPM (up = faster)
 * - A button: Toggle flywheel on/off at current RPM
 * - B button: Reset RPM to zero and disable flywheel
 * - D-pad up/down: Fine adjust RPM (+/- 50)
 * - Right bumper: Hold for fine control mode (slower stick response)
 * - LEFT BUMPER: Hold to run intake (feeds balls into spindexer)
 * - X button: SHOOT - activates transfer to fire ball
 * - Y button: Hold to eject (reverse intake)
 * - Left/Right triggers: Nudge spindexer CCW/CW
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

    // Debounce flags
    private var aPressed = false
    private var bPressed = false
    private var dpadUpPressed = false
    private var dpadDownPressed = false
    private var xPressed = false
    private var wasIntaking = false
    private var wasLeftTrigger = false
    private var wasRightTrigger = false

    override fun runOpMode() {
        // Create flywheel controller using deadbeat control
        flywheel = Flywheel(
            motor = flywheelMotor,
            inertia = FlywheelDeadbeatConfig.inertia,
            io = io,
            lag = FlywheelDeadbeatConfig.lagMs.milliseconds
        )

        // Create spindexer logic with the flywheel for coordinated control
        spindexerLogic = SpindexerLogic(io, flywheel)

        telemetry.addLine("=== SHOOTER EXPERIMENT TEST ===")
        telemetry.addLine("")
        telemetry.addLine("Left stick Y: Adjust RPM")
        telemetry.addLine("A: Toggle flywheel on/off")
        telemetry.addLine("B: Reset all to zero")
        telemetry.addLine("D-pad up/down: Fine RPM (+/- 50)")
        telemetry.addLine("Right bumper: Fine control mode")
        telemetry.addLine("")
        telemetry.addLine("LEFT BUMPER: Hold to intake")
        telemetry.addLine("X: SHOOT (transfer fires ball)")
        telemetry.addLine("Y: Hold to eject")
        telemetry.addLine("Triggers: Nudge spindexer L/R")
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

            // === TOGGLE / RESET ===

            // A button: toggle flywheel
            if (gamepad1.a && !aPressed) {
                flywheelEnabled = !flywheelEnabled
            }
            aPressed = gamepad1.a

            // B button: reset all
            if (gamepad1.b && !bPressed) {
                targetRPM = 0.0
                flywheelEnabled = false
            }
            bPressed = gamepad1.b

            // === INTAKE CONTROL ===
            // Left bumper = run intake to bring balls into spindexer
            val intaking = gamepad1.left_bumper
            if (intaking && !wasIntaking) {
                spindexerLogic.startIntaking()
            } else if (!intaking && wasIntaking) {
                spindexerLogic.stopIntaking()
            }
            wasIntaking = intaking

            // Y button = eject (reverse intake manually)
            if (gamepad1.y) {
                io.intake = 1.0  // Reverse to eject
            }

            // === SPINDEXER NUDGE ===
            // Triggers nudge spindexer to manually position balls
            val leftTrigger = gamepad1.left_trigger > 0.5
            val rightTrigger = gamepad1.right_trigger > 0.5

            if (leftTrigger && !wasLeftTrigger) {
                spindexerLogic.nudge(true)  // CCW
            }
            wasLeftTrigger = leftTrigger

            if (rightTrigger && !wasRightTrigger) {
                spindexerLogic.nudge(false)  // CW
            }
            wasRightTrigger = rightTrigger

            // === SHOOT CONTROL (X button) ===
            // X button: Fire the ball through the shooter
            if (gamepad1.x && !xPressed) {
                // Set the shot velocity to our target RPM
                val targetRadPerSec = targetRPM * 2.0 * PI / 60.0
                spindexerLogic.shotVelocity = targetRadPerSec
                spindexerLogic.shootingRequested = true
                spindexerLogic.shoot()
            }
            xPressed = gamepad1.x

            // Clear shooting request when X is released
            if (!gamepad1.x) {
                spindexerLogic.shootingRequested = false
            }

            // === APPLY FLYWHEEL CONTROL ===
            // When flywheel is enabled but not shooting, spin up to target
            if (flywheelEnabled && spindexerLogic.currentState != SpindexerLogic.State.SHOOTING) {
                val targetRadPerSec = targetRPM * 2.0 * PI / 60.0
                flywheel.target = targetRadPerSec
                flywheel.hold = true  // Use simple feedforward for spinup
                flywheel.update(io.flywheelVelocity(), dt)
            } else if (!flywheelEnabled && spindexerLogic.currentState != SpindexerLogic.State.SHOOTING) {
                // Flywheel disabled and not shooting - coast to stop
                flywheel.target = 0.0
                flywheel.hold = true
                flywheel.update(io.flywheelVelocity(), dt)
            }
            // During shooting, SpindexerLogic controls the flywheel

            // Update spindexer logic (handles FSM, spindexer motor, transfer, and flywheel during shooting)
            spindexerLogic.update(dt)

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
            telemetry.addData("Mode", if (fineMode) "FINE" else "NORMAL")
            telemetry.addLine("----------------------------------------")
            telemetry.addLine("A=flywheel | B=reset | X=SHOOT")
            telemetry.addLine("LB=intake | Y=eject | RB=fine")
            telemetry.addLine("Triggers=nudge spindexer")
            telemetry.update()

            false // continue loop
        }
    }
}
