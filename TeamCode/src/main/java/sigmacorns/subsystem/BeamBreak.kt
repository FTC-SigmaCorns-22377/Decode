package sigmacorns.subsystem

import sigmacorns.io.SigmaIO

/**
 * Continuously polls the three beam break sensors to track how many balls
 * are held in the robot. Each slot is either null (empty) or true (ball present).
 */
class BeamBreak(val io: SigmaIO) {

    // 3 slots: null = empty, true = ball present
    val slots: Array<Boolean?> = arrayOfNulls(3)

    /** Number of balls currently held. */
    val ballCount: Int get() = slots.count { it == true }

    /** True when all 3 slots are occupied. */
    val isFull: Boolean get() = ballCount >= 3

    /**
     * Call every loop iteration. Reads beam break sensors from IO and
     * updates the ball slot list.
     */
    fun update() {
        slots[0] = if (io.beamBreak1()) true else null
        slots[1] = if (io.beamBreak2()) true else null
        slots[2] = if (io.beamBreak3()) true else null
    }
}
