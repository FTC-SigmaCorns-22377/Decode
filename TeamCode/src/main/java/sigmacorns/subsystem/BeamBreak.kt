package sigmacorns.subsystem

import sigmacorns.io.SigmaIO
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

/**
 * Continuously polls the three beam break sensors to track how many balls
 * are held in the robot. Ball slot 1 is the slot closest to the shooter.
 *
 * Each slot has a hold-on cooldown: once a slot goes true it stays true for
 * [holdMs] milliseconds after the sensor clears, so brief gaps during transfer
 * don't cause ball count to drop mid-shot.
 */
class BeamBreak(val io: SigmaIO) {

    var holdMs: Long = 600L

    // 3 slots: true = ball present (including cooldown hold)
    val slots: BooleanArray = booleanArrayOf(false, false, false)

    /** Number of balls currently held. */
    val ballCount: Int get() = slots.count { it }

    /** True when all 3 slots are occupied. */
    val isFull: Boolean get() = ballCount >= 3

    private val holdExpiry: Array<Duration?> = arrayOfNulls(3)

    /**
     * Call every loop iteration. Reads beam break sensors from IO and
     * updates the ball slot list with per-slot hold cooldown.
     */
    fun update() {
        val now = io.time()
        val raw = booleanArrayOf(io.beamBreak1(), io.beamBreak2(), io.beamBreak3())
        for (i in 0..2) {
            if (raw[i]) {
                slots[i] = true
                holdExpiry[i] = null
            } else {
                val expiry = holdExpiry[i]
                if (slots[i] && expiry == null) {
                    // just went false — start the cooldown
                    holdExpiry[i] = now + holdMs.milliseconds
                    slots[i] = true
                } else if (expiry != null && now < expiry) {
                    slots[i] = true  // still within hold window
                } else {
                    slots[i] = false
                    holdExpiry[i] = null
                }
            }
        }
    }
}