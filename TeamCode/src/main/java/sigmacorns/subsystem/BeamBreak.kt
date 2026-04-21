package sigmacorns.subsystem

import sigmacorns.io.SigmaIO
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

/**
 * Continuously polls the three beam break sensors to track how many balls
 * are held in the robot. Slot 0 is the slot closest to the shooter,
 * slot 2 is the intake-end slot.
 *
 * Applies an asymmetric per-slot "hold-on" filter so ballCount does not dip
 * while a ball is actively traveling through the transfer path. Rising edges
 * (ball arrived) are instant; falling edges (sensor uncovered) are deferred
 * by [holdMs0] / [holdMs1] / [holdMs2] so consumers (burst counters, shooter
 * flywheel drop, shot planner, intake auto-stop) see a stable count.
 */
class BeamBreak(val io: SigmaIO) {

    /** Hold time for slot 0 (nearest shooter). Must survive a ball transiting out. */
    var holdMs0: Long = 120L
    /** Hold time for slot 1 (middle). Shorter — ball re-triggers slot 0 quickly. */
    var holdMs1: Long = 50L
    /** Hold time for slot 2 (intake end). 0 — isFull must respond instantly. */
    var holdMs2: Long = 0L

    val slots: BooleanArray = booleanArrayOf(false, false, false)

    /** Number of balls currently held (filtered). */
    val ballCount: Int get() = slots.count { it }

    /** True when all 3 slots are occupied (filtered). */
    val isFull: Boolean get() = ballCount >= 3

    private val holdExpiry: Array<Duration?> = arrayOfNulls(3)

    /**
     * Call every loop. Applies the asymmetric filter to raw sensor reads.
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
                    val hold = when (i) { 0 -> holdMs0; 1 -> holdMs1; else -> holdMs2 }
                    holdExpiry[i] = now + hold.milliseconds
                    slots[i] = true
                } else if (expiry != null && now < expiry) {
                    slots[i] = true
                } else {
                    slots[i] = false
                    holdExpiry[i] = null
                }
            }
        }
    }
}
