package sigmacorns

import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.FlywheelState
import sigmacorns.sim.MecanumState
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

data class FieldState (
    var motif: Int //motif can only equal = 21 (GPP), 22 (PGP), or 23 (PPG)
) {
    constructor(): this(
        motif = 0
    ) {
    }
}