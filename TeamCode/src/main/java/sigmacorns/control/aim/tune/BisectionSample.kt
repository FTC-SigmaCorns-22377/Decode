package sigmacorns.control.aim.tune

enum class ShotOutcome { SHORT, LONG, MADE, SKIPPED }

data class BisectionSample(
    val cellId: String,
    val distance: Double,       // m — actual distance at shot
    val hoodAngleDeg: Double,   // deg — actual commanded hood
    val omegaMeasured: Double,  // rad/s — peak flywheel ω during shot window
    val omegaCommanded: Double, // rad/s — target set by tuner
    val outcome: ShotOutcome,
    val timestampMs: Long
)

data class CellState(
    val cellId: String,
    val targetDistance: Double,
    val targetHoodDeg: Double,
    var omegaLo: Double,
    var omegaHi: Double,
    var nShots: Int = 0,
    var nMades: Int = 0,
    var converged: Boolean = false,
    var skipped: Boolean = false,
    var lastOmegaMade: Double? = null
) {
    val omegaMid: Double get() = 0.5 * (omegaLo + omegaHi)
    val bracketWidth: Double get() = omegaHi - omegaLo
}
