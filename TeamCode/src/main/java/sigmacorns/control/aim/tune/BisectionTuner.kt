package sigmacorns.control.aim.tune

import sigmacorns.constants.flywheelRadius
import sigmacorns.logic.AimConfig
import sigmacorns.subsystem.ShooterConfig

/**
 * Grid-based bisection tuner for the omega(phi, v) map.
 *
 * The grid is (distance × hoodAngle). Per cell:
 *  1. Bisect flywheel omega until the bracket narrows or a MADE is recorded.
 *  2. On MADE, keep shooting at the converged omega to collect N_MAKES_TARGET
 *     make-samples at slightly varying positions within the cell's distance window.
 *     Multiple makes per cell at real (not nominal) distances make the polynomial
 *     fit robust to position noise.
 *  3. Once N_MAKES_TARGET makes are recorded, advance to next cell.
 *
 * Measured omega (peak ω during shot window) is stored, not the commanded value.
 * Drag-aware v inversion (OmegaCoefFitter.distanceHoodToVExit) converts each
 * recorded (d, phi, omega) tuple to the (phi, v) basis the solver queries at runtime.
 */
class BisectionTuner(
    private val store: BisectionStore,
    private val getCurrentOmegaCoeffs: () -> FloatArray
) {
    companion object {
        val DISTANCES = listOf(1.25, 1.5, 2.0, 2.5, 3.0, 3.5)
        val HOOD_DEGS = listOf(
            ShooterConfig.minAngleDeg + 5.0,
            (ShooterConfig.minAngleDeg*3.0 + ShooterConfig.maxAngleDeg) / 4.0,
            (ShooterConfig.minAngleDeg + ShooterConfig.maxAngleDeg) / 2.0,
            (ShooterConfig.minAngleDeg + ShooterConfig.maxAngleDeg*3.0) / 4.0,
            ShooterConfig.maxAngleDeg - 5.0
        )
        private const val OMEGA_MAX = 628.0    // ~6000 rpm
        private const val CONVERGE_WIDTH = 15.0 // rad/s bracket width → bisect done
        const val N_MAKES_TARGET = 3           // makes required per cell before advancing
        private const val SEED_MARGIN = 0.25   // ±25% bracket around seeded omega

        fun cellId(d: Double, phi: Double) = "d${d.toInt()}p${phi.toInt()}"
    }

    init { ensureGrid() }

    private fun ensureGrid() {
        for (d in DISTANCES) for (phi in HOOD_DEGS) {
            val id = cellId(d, phi)
            if (store.getCell(id) == null) {
                val seed = seedOmega(d, phi)
                store.putCell(CellState(
                    cellId = id,
                    targetDistance = d,
                    targetHoodDeg = phi,
                    omegaLo = (seed * (1.0 - SEED_MARGIN)).coerceAtLeast(0.0),
                    omegaHi = (seed * (1.0 + SEED_MARGIN)).coerceAtMost(OMEGA_MAX)
                ))
            }
        }
    }

    private fun seedOmega(d: Double, hoodDeg: Double): Double {
        val phi = Math.toRadians(hoodDeg)
        val v = OmegaCoefFitter.distanceHoodToVExit(d, phi)
            ?: return d * 80.0
        val c = getCurrentOmegaCoeffs()
        val omega = c[0] + c[1]*v + c[2]*phi + c[3]*v*v + c[4]*phi*v + c[5]*phi*phi
        return omega.toDouble().coerceIn(100.0, OMEGA_MAX)
    }

    fun cells(): List<CellState> = store.getCells()
    fun activeCells(): List<CellState> = store.getCells().filter { !it.skipped }
    fun pendingCells(): List<CellState> = store.getCells().filter { !it.converged && !it.skipped }
    fun nextCell(): CellState? = pendingCells().firstOrNull()

    /**
     * Whether the cell is in "collecting makes" mode (bisection done, accumulating samples).
     * In this mode, omega stays at the converged bracket midpoint; we just need more makes.
     */
    fun isCollectingMakes(cell: CellState): Boolean =
        !cell.converged && !cell.skipped &&
        (cell.bracketWidth < CONVERGE_WIDTH || cell.nMades > 0)

    fun recordOutcome(
        cell: CellState,
        outcome: ShotOutcome,
        omegaMeasured: Double,
        omegaCommanded: Double,
        actualDistance: Double,
        actualHoodDeg: Double
    ) {
        val sample = BisectionSample(
            cellId = cell.cellId,
            distance = actualDistance,
            hoodAngleDeg = actualHoodDeg,
            omegaMeasured = omegaMeasured,
            omegaCommanded = omegaCommanded,
            outcome = outcome,
            timestampMs = System.currentTimeMillis()
        )
        store.addSample(sample)

        when (outcome) {
            ShotOutcome.SHORT -> {
                cell.omegaLo = omegaMeasured.coerceAtLeast(cell.omegaLo)
            }
            ShotOutcome.LONG -> {
                cell.omegaHi = omegaMeasured.coerceAtMost(cell.omegaHi)
            }
            ShotOutcome.MADE -> {
                cell.lastOmegaMade = omegaMeasured
                cell.nMades++
                if (cell.nMades >= N_MAKES_TARGET) cell.converged = true
            }
            ShotOutcome.SKIPPED -> cell.skipped = true
        }
        cell.nShots++
        if (cell.bracketWidth < CONVERGE_WIDTH && cell.nMades == 0) {
            // Bracket converged without a make yet — lock it and keep shooting to collect makes
            // Don't mark converged yet; wait for N_MAKES_TARGET makes
        }
        store.putCell(cell)
        store.save()
    }

    fun skipCell(cell: CellState) {
        cell.skipped = true
        store.putCell(cell)
        store.save()
    }

    fun resetCell(cell: CellState) {
        val seed = seedOmega(cell.targetDistance, cell.targetHoodDeg)
        cell.omegaLo = (seed * (1.0 - SEED_MARGIN)).coerceAtLeast(0.0)
        cell.omegaHi = (seed * (1.0 + SEED_MARGIN)).coerceAtMost(OMEGA_MAX)
        cell.nShots = 0
        cell.nMades = 0
        cell.converged = false
        cell.skipped = false
        cell.lastOmegaMade = null
        store.putCell(cell)
        // Remove all samples for this cell
        val remaining = store.getSamples().filter { it.cellId != cell.cellId }
        store.clear()
        remaining.forEach { store.addSample(it) }
        // Re-init all other cells from remaining
        store.save()
    }

    fun undoLastSample(cell: CellState) {
        store.removeLastSample()
        // Recompute bracket from remaining samples for this cell
        val seed = seedOmega(cell.targetDistance, cell.targetHoodDeg)
        cell.omegaLo = (seed * (1.0 - SEED_MARGIN)).coerceAtLeast(0.0)
        cell.omegaHi = (seed * (1.0 + SEED_MARGIN)).coerceAtMost(OMEGA_MAX)
        cell.converged = false
        cell.skipped = false
        cell.lastOmegaMade = null
        cell.nShots = 0
        cell.nMades = 0
        for (s in store.getSamples().filter { it.cellId == cell.cellId }) {
            when (s.outcome) {
                ShotOutcome.SHORT   -> { cell.omegaLo = s.omegaMeasured.coerceAtLeast(cell.omegaLo); cell.nShots++ }
                ShotOutcome.LONG    -> { cell.omegaHi = s.omegaMeasured.coerceAtMost(cell.omegaHi); cell.nShots++ }
                ShotOutcome.MADE    -> { cell.lastOmegaMade = s.omegaMeasured; cell.nMades++; cell.nShots++ }
                ShotOutcome.SKIPPED -> cell.skipped = true
            }
        }
        if (cell.nMades >= N_MAKES_TARGET) cell.converged = true
        store.putCell(cell)
        store.save()
    }

    fun madeSpeedPoints(): List<SpeedPoint> = store.madeSamples().map {
        SpeedPoint(it.distance, it.omegaMeasured, it.hoodAngleDeg)
    }

    fun allSamplesJson(): String {
        val sb = StringBuilder("[")
        val samples = store.getSamples()
        samples.forEachIndexed { i, s ->
            sb.append("""{"cellId":"${s.cellId}","distance":${s.distance},"hoodDeg":${s.hoodAngleDeg},"omegaMeasured":${s.omegaMeasured},"omegaCommanded":${s.omegaCommanded},"outcome":"${s.outcome}","ts":${s.timestampMs}}""")
            if (i < samples.size - 1) sb.append(',')
        }
        sb.append(']')
        return sb.toString()
    }

    fun cellsJson(): String {
        val sb = StringBuilder("[")
        val cells = store.getCells()
        cells.forEachIndexed { i, c ->
            val made = c.lastOmegaMade?.toString() ?: "null"
            sb.append("""{"id":"${c.cellId}","d":${c.targetDistance},"phi":${c.targetHoodDeg},"lo":${c.omegaLo},"hi":${c.omegaHi},"mid":${c.omegaMid},"shots":${c.nShots},"nMades":${c.nMades},"makesTarget":$N_MAKES_TARGET,"converged":${c.converged},"skipped":${c.skipped},"made":$made}""")
            if (i < cells.size - 1) sb.append(',')
        }
        sb.append(']')
        return sb.toString()
    }
}
