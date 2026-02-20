package sigmacorns.control.subsystem

import sigmacorns.control.MotorRangeMapper
import sigmacorns.io.SigmaIO
import kotlin.time.Duration

class Spindexer(
    val range: MotorRangeMapper,
    val io: SigmaIO
) {
    var target: Double = 0.0
    var curRotation: Double = 0.0

    fun update(dt: Duration) {
        curRotation = range.tickToPos(io.spindexerPosition())
        // Set target position in ticks; motor runs in RUN_TO_POSITION mode
        io.spindexer = range.posToTick(target)
    }
}