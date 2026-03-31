package sigmacorns.subsystem

import kotlinx.coroutines.delay
import sigmacorns.io.SigmaIO
import kotlin.time.Duration

/**
 * Unified intake-transfer subsystem.
 *
 * The intake and internal transfer are one continuous mechanical system driven
 * by two motors sharing a single [SigmaIO.intake] power output. A blocker servo
 * at the end of the transfer path gates ball entry into the shooter.
 *
 * Motor priority: TRANSFERRING > REVERSING > INTAKING > IDLE.
 */
class IntakeTransfer(val io: SigmaIO) {

    enum class State {
        IDLE,
        INTAKING,
        REVERSING,
        TRANSFERRING
    }

    var state: State = State.IDLE

    /** Set by coordinator from BeamBreak state. */
    var isFull: Boolean = false

    companion object {
        const val INTAKE_POWER = 1.0
        const val OUTTAKE_POWER = -1.0
        const val TRANSFER_POWER = 1.0
        const val BLOCKER_ENGAGED = 0.0
        const val BLOCKER_DISENGAGED = 1.0
        const val BLOCKER_MOVE_DELAY_MS = 300L
    }

    fun startIntake() {
        if (isFull) return
        state = State.INTAKING
    }

    fun stopIntake() {
        if (state == State.INTAKING || state == State.REVERSING) {
            state = State.IDLE
        }
    }

    fun startReverse() {
        state = State.REVERSING
    }

    fun stopReverse() {
        if (state == State.REVERSING) {
            state = State.IDLE
        }
    }

    fun engageBlocker() {
        io.blocker = BLOCKER_ENGAGED
    }

    fun disengageBlocker() {
        io.blocker = BLOCKER_DISENGAGED
    }

    /**
     * Begin transferring balls to the shooter.
     * Disengages blocker first and waits for it to physically move.
     */
    suspend fun startTransfer() {
        disengageBlocker()
        delay(BLOCKER_MOVE_DELAY_MS)
        state = State.TRANSFERRING
    }

    /** Stop transferring. Re-engages blocker. */
    suspend fun stopTransfer() {
        state = State.IDLE
        engageBlocker()
    }

    /**
     * Called every loop. Sets motor power based on state.
     */
    fun update(dt: Duration) {
        io.intake = when (state) {
            State.TRANSFERRING -> TRANSFER_POWER
            State.REVERSING -> OUTTAKE_POWER
            State.INTAKING -> INTAKE_POWER
            State.IDLE -> 0.0
        }
    }
}
