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
 * Motor priority: transfer > reverse > intake > idle.
 */
class IntakeTransfer(val io: SigmaIO) {

    var isIntaking = false
    var isReversing = false
    var isTransferring = false

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
        isIntaking = true
        isReversing = false
    }

    fun stopIntake() {
        isIntaking = false
        isReversing = false
    }

    fun startReverse() {
        isReversing = true
        isIntaking = false
    }

    fun stopReverse() {
        isReversing = false
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
        isTransferring = true
    }

    /** Stop transferring. Re-engages blocker. */
    suspend fun stopTransfer() {
        isTransferring = false
        engageBlocker()
    }

    /**
     * Called every loop. Sets motor power based on state flags.
     * Priority: transfer > reverse > intake > idle.
     */
    fun update(dt: Duration) {
        io.intake = when {
            isTransferring -> TRANSFER_POWER
            isReversing -> OUTTAKE_POWER
            isIntaking -> INTAKE_POWER
            else -> 0.0
        }
    }
}
