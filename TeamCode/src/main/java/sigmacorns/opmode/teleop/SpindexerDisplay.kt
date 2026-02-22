package sigmacorns.opmode.teleop

import sigmacorns.sim.Balls

/**
 * Builds large-format unicode block displays for the spindexer autosort telemetry.
 * Each slot is rendered as a 4×4 character block:
 *   Green  = ████  (filled)
 *   Purple = ▒▒▒▒  (checkerboard)
 *   Empty  = ____  (underscore)
 */
object SpindexerDisplay {
    private const val BLOCK_SIZE = 4
    private const val SLOT_COUNT = 3

    fun ballChar(ball: Balls?): Char = when (ball) {
        Balls.Green  -> '█'
        Balls.Purple -> 'P'
        Balls.Empty  -> '_'
        null         -> '_'
    }

    /** One text row for a set of slots, e.g. "████ ▒▒▒▒ ____" */
    private fun slotTextRow(balls: List<Balls?>): String =
        balls.joinToString(" ") { ball ->
            ballChar(ball).toString().repeat(BLOCK_SIZE)
        }

    /** 4 identical rows forming the block display for a set of slots. */
    fun buildSlotRows(balls: List<Balls?>): List<String> {
        val row = slotTextRow(balls)
        return List(BLOCK_SIZE) { row }
    }

    /** Arrow row: 4 arrow chars under the chosen slot index. */
    fun buildArrowRow(arrowIdx: Int): String {
        // Each slot is BLOCK_SIZE chars wide, separated by 1 space.
        // Slot i starts at position i * (BLOCK_SIZE + 1).
        val totalWidth = SLOT_COUNT * BLOCK_SIZE + (SLOT_COUNT - 1)
        val arr = CharArray(totalWidth) { ' ' }
        val start = arrowIdx * (BLOCK_SIZE + 1)
        for (i in start until start + BLOCK_SIZE) {
            arr[i] = '↑'
        }
        return String(arr)
    }

    /** Full display as a list of lines. */
    fun buildDisplay(
        motif: List<Balls?>,
        spindexerState: List<Balls?>,
        nextMotifIndex: Int,
        stateName: String
    ): List<String> = buildList {
        add("TARGET:")
        addAll(buildSlotRows(motif))
        add(buildArrowRow(nextMotifIndex))
        add("")
        add("SPINDEXER:")
        addAll(buildSlotRows(spindexerState))
        add("")
        add("State: $stateName")
    }
}
