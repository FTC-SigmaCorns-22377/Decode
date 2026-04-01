package sigmacorns.subsystem

import sigmacorns.io.SigmaIO
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

/**
 * Unified intake-transfer subsystem as a state machine.
 *
 * The intake and internal transfer are one continuous mechanical system driven
 * by two motors sharing a single [SigmaIO.intake] power output. A blocker servo
 * at the end of the transfer path gates ball entry into the shooter.
 *
 * All IO writes happen exclusively in [update]. External code (logic layer,
 * opmodes) may only set [state] or call the transition methods — never write
 * to IO directly.
 *
 * The blocker is fully state-driven:
 * - Engaged (blocking) in IDLE, INTAKING, REVERSING
 * - Disengaged (open) in TRANSFERRING, READY_TO_SHOOT
 *
 * When transitioning to TRANSFERRING, the motor waits [BLOCKER_MOVE_DELAY]
 * for the blocker servo to physically open before driving balls through.
 */
class IntakeTransfer(val io: SigmaIO) {

    enum class State {
        /** Motor off, blocker engaged. */
        IDLE,
        /** Motor forward (intake), blocker engaged. */
        INTAKING,
        /** Motor reverse (eject), blocker engaged. */
        REVERSING,
        /** Motor forward (transfer) after blocker delay, blocker disengaged. */
        TRANSFERRING,
        /** Motor off, blocker disengaged. Used by auto-shoot zone detection. */
        READY_TO_SHOOT
    }

    var state: State = State.IDLE
        set(value) {
            if (field != value) {
                // When entering a blocker-disengaged state, start the delay timer
                val wasDisengaged = field == State.TRANSFERRING || field == State.READY_TO_SHOOT
                val willDisengage = value == State.TRANSFERRING || value == State.READY_TO_SHOOT
                if (willDisengage && !wasDisengaged) {
                    blockerReady = false
                    blockerDisengagedAt = lastUpdateTime
                }
                field = value
            }
        }

    companion object {
        const val INTAKE_POWER = 1.0
        const val OUTTAKE_POWER = -1.0
        const val TRANSFER_POWER = 1.0
        const val BLOCKER_ENGAGED = 0.0
        const val BLOCKER_DISENGAGED = 1.0
        val BLOCKER_MOVE_DELAY = 300.milliseconds
    }

    /** Whether the blocker has had enough time to physically open. */
    var blockerReady: Boolean = true
        private set

    private var blockerDisengagedAt: Duration = Duration.ZERO
    private var lastUpdateTime: Duration = Duration.ZERO

    // -- Update (sole owner of IO writes) --

    /**
     * Called every loop. Drives blocker servo and motor power from state.
     * @param time absolute time from [SigmaIO.time], used for blocker delay.
     */
    fun update(dt: Duration, time: Duration) {
        lastUpdateTime = time

        // Blocker servo: disengaged only in TRANSFERRING and READY_TO_SHOOT
        val blockerDisengaged = state == State.TRANSFERRING || state == State.READY_TO_SHOOT
        io.blocker = if (blockerDisengaged) BLOCKER_DISENGAGED else BLOCKER_ENGAGED

        // Track blocker readiness (delay after disengaging)
        if (blockerDisengaged && !blockerReady) {
            if (time - blockerDisengagedAt >= BLOCKER_MOVE_DELAY) {
                blockerReady = true
            }
        }
        if (!blockerDisengaged) {
            blockerReady = true
        }

        // Motor power
        io.intake = when (state) {
            State.TRANSFERRING -> if (blockerReady) TRANSFER_POWER else 0.0
            State.REVERSING -> OUTTAKE_POWER
            State.INTAKING -> INTAKE_POWER
            State.IDLE, State.READY_TO_SHOOT -> 0.0
        }
    }
}
