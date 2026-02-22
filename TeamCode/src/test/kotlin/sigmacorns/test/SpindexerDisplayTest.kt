package sigmacorns.test

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import sigmacorns.opmode.teleop.SpindexerDisplay
import sigmacorns.sim.Balls

class SpindexerDisplayTest {

    @Test
    fun testGreenPurplePurpleMotif() {
        val lines = SpindexerDisplay.buildDisplay(
            motif = listOf(Balls.Green, Balls.Purple, Balls.Purple),
            spindexerState = listOf(Balls.Green, null, Balls.Purple),
            nextMotifIndex = 0,
            stateName = "IDLE"
        )
        lines.forEach { println(it) }

        assertEquals("TARGET:", lines[0])
        // 4 identical block rows for motif
        val expectedMotifRow = "████ ▒▒▒▒ ▒▒▒▒"
        for (i in 1..4) assertEquals(expectedMotifRow, lines[i])
        // Arrow under first slot
        assertEquals("↑↑↑↑          ", lines[5])
        // Blank separator
        assertEquals("", lines[6])
        assertEquals("SPINDEXER:", lines[7])
        // 4 identical block rows for spindexer
        val expectedSpindexerRow = "████ ____ ▒▒▒▒"
        for (i in 8..11) assertEquals(expectedSpindexerRow, lines[i])
        assertEquals("", lines[12])
        assertEquals("State: IDLE", lines[13])
    }

    @Test
    fun testArrowSlot0() {
        val row = SpindexerDisplay.buildArrowRow(0)
        assertEquals("↑↑↑↑          ", row)
    }

    @Test
    fun testArrowSlot1() {
        val row = SpindexerDisplay.buildArrowRow(1)
        assertEquals("     ↑↑↑↑     ", row)
    }

    @Test
    fun testArrowSlot2() {
        val row = SpindexerDisplay.buildArrowRow(2)
        assertEquals("          ↑↑↑↑", row)
    }

    @Test
    fun testAllEmpty() {
        val lines = SpindexerDisplay.buildDisplay(
            motif = listOf(null, null, null),
            spindexerState = listOf(null, null, null),
            nextMotifIndex = 1,
            stateName = "SHOOTING"
        )
        lines.forEach { println(it) }

        val expectedRow = "____ ____ ____"
        for (i in 1..4) assertEquals(expectedRow, lines[i])
        assertEquals("     ↑↑↑↑     ", lines[5])
        for (i in 8..11) assertEquals(expectedRow, lines[i])
        assertEquals("State: SHOOTING", lines[lines.size - 1])
    }

    @Test
    fun testAllGreen() {
        val lines = SpindexerDisplay.buildDisplay(
            motif = listOf(Balls.Green, Balls.Green, Balls.Green),
            spindexerState = listOf(Balls.Green, Balls.Green, Balls.Green),
            nextMotifIndex = 2,
            stateName = "FULL"
        )
        lines.forEach { println(it) }

        val expectedRow = "████ ████ ████"
        for (i in 1..4) assertEquals(expectedRow, lines[i])
        assertEquals("          ↑↑↑↑", lines[5])
        for (i in 8..11) assertEquals(expectedRow, lines[i])
        assertEquals("State: FULL", lines[lines.size - 1])
    }

    @Test
    fun testMixedPattern() {
        val lines = SpindexerDisplay.buildDisplay(
            motif = listOf(Balls.Purple, Balls.Green, Balls.Purple),
            spindexerState = listOf(null, Balls.Purple, Balls.Green),
            nextMotifIndex = 1,
            stateName = "MOVING_SHOOT"
        )
        lines.forEach { println(it) }

        assertEquals("▒▒▒▒ ████ ▒▒▒▒", lines[1])
        assertEquals("____ ▒▒▒▒ ████", lines[8])
        assertEquals("     ↑↑↑↑     ", lines[5])
        assertEquals("State: MOVING_SHOOT", lines[lines.size - 1])
    }

    @Test
    fun testLineCount() {
        val lines = SpindexerDisplay.buildDisplay(
            motif = listOf(Balls.Green, Balls.Purple, Balls.Purple),
            spindexerState = listOf(null, null, null),
            nextMotifIndex = 0,
            stateName = "IDLE"
        )
        // TARGET: + 4 blocks + arrow + blank + SPINDEXER: + 4 blocks + blank + State = 14
        assertEquals(14, lines.size)
    }
}
