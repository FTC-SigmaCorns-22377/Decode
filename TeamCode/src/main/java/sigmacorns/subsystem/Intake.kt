package sigmacorns.subsystem

import sigmacorns.Robot
import kotlin.time.Duration

/**
 * Intake subsystem: controls the intake motor for collecting and ejecting balls.
 *
 * States are mutually exclusive: intaking, reversing, or idle.
 * When intaking, the blocker is engaged and transfer is stopped to prevent
 * accidentally shooting balls. The BeamBreak system auto-stops intaking
 * when 3 balls are held.
 */
class Intake(val robot: Robot) {

    var isRunning = false
    var isReversing = false

    companion object {
        const val INTAKE_POWER = 1.0
        const val OUTTAKE_POWER = -1.0
    }

    /** Start intaking: engage blocker and turn off transfer so we don't accidentally shoot. */
    fun startIntake() {
        if (robot.beamBreak.isFull) return
        isRunning = true
        isReversing = false
        robot.transfer.engageBlocker()
        robot.transfer.isRunning = false
    }

    /** Stop the intake motor. */
    fun stopIntake() {
        isRunning = false
        isReversing = false
    }

    /** Reverse the intake to spit balls out. */
    fun startReverse() {
        isReversing = true
        isRunning = false
    }

    /** Stop reversing the intake. */
    fun stopReverse() {
        isReversing = false
    }

    /**
     * Called every loop. Sets motor power based on state flags.
     * Does NOT set power when transfer is running (transfer owns the motor then).
     */
    fun update(dt: Duration) {
        // Transfer takes priority - it uses the intake motor to feed balls to shooter
        if (robot.transfer.isRunning) return

        robot.io.intake = when {
            isReversing -> OUTTAKE_POWER
            isRunning -> INTAKE_POWER
            else -> 0.0
        }
    }
}
