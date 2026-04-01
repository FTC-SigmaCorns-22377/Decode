package sigmacorns.subsystem

import sigmacorns.io.SigmaIO

/**
 * Continuously polls the three beam break sensors to track how many balls
 * are held in the robot. Ball slot 1 is the slot closest to the shooter.
 */
class BeamBreak(val io: SigmaIO) {

    // 3 slots: null = empty, true = ball present
    val slots: BooleanArray = booleanArrayOf(false, false, false)

    /** Number of balls currently held. */
    val ballCount: Int get() = slots.count { it == true }

    /** True when all 3 slots are occupied. */
    val isFull: Boolean get() = ballCount >= 3

    /**
     * Call every loop iteration. Reads beam break sensors from IO and
     * updates the ball slot list.
     */
    fun update() {
        slots[0] = io.beamBreak1()
        slots[1] = io.beamBreak2()
        slots[2] = io.beamBreak3()
    }
}
