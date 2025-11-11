package sigmacorns


import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.FlywheelState
import sigmacorns.sim.MecanumState
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

val globalFieldState = FieldState()

data class FieldState (
    var motif: Int = 0, //motif can only equal = 21 (GPP), 22 (PGP), or 23 (PPG)
    var ramp: IntArray = IntArray(9)
)