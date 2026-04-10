package sigmacorns.test

import org.joml.Vector2d
import org.junit.jupiter.api.Test
import sigmacorns.State
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.ltv.LTVClient
import sigmacorns.control.ltv.QpSolverType
import sigmacorns.control.trajopt.TrajoptLoader
import sigmacorns.io.RerunLogging
import sigmacorns.io.SimIO
import sigmacorns.opmode.SigmaOpMode
import java.io.File
import java.util.Random
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.system.measureNanoTime
import kotlin.time.Duration.Companion.nanoseconds

/**
 * Compares solve-time performance and closed-loop curve tracking across all
 * available QP solver modes (FISTA, NEON_IPM, and optionally HPIPM_OCP).
 *
 * Each solver runs an independent closed-loop simulation from the same initial
 * position with identical per-step motor disturbance noise, so trajectory
 * divergence is purely a function of solver quality rather than noise.
 *
 * Entity layout in Rerun:
 *   reference/trajectory                    — static blue reference path
 *   solver/{name}/trajectory                — static actual path for each solver
 *   solver/{name}/position                  — point cloud at each timestep
 *   metrics/solve_time_us/{name}            — per-step solve time (µs)
 *   metrics/pos_error/{name}                — distance to current window reference
 *   metrics/heading_error/{name}            — heading error to window reference
 *   control/{fl|bl|br|fr}/{name}            — motor commands per step
 *
 * Output: rerun/ltv_solver_timing.rrd
 *
 * Noise model: Gaussian u-noise with σ=NOISE_STDDEV added after clamping to
 * [-1,1], shared by all solvers (same seed → same noise sequence) so that
 * any trajectory difference is caused by solver output differences.
 */
class LTVSolverTimingTest {

    companion object {
        private const val SETTLE_STEPS  = 30
        private const val NOISE_STDDEV  = 0.05
        private const val NOISE_SEED    = 42L
    }

    /** Colors for each solver in ARGB. */
    private val solverColors = mapOf(
        QpSolverType.FISTA     to 0xFF00C800.toInt(),  // green
        QpSolverType.NEON_IPM  to 0xFFFFB400.toInt(),  // amber
        QpSolverType.HPIPM_OCP to 0xFF64C8FF.toInt(),  // cyan
    )

    private data class SolverRun(
        val type: QpSolverType,
        val solveTimes: List<Long>,         // nanoseconds per step
        val trajectory: List<Vector2d>,     // position after each step
        val posErrors: List<Double>,        // distance to window ref at each step
        val headingErrors: List<Double>,    // heading error at each step
        val timeAxis: List<Double>,         // sim_time seconds at each step
    )

    @Test
    fun testSolverTimingComparison() {
        SigmaOpMode.SIM = true
        SigmaOpMode.LIMELIGHT_CONNECTED = false

        val projectFile = TrajoptLoader.findProjectFile("simple")
            ?: error("No trajopt project file found in ${TrajoptLoader.robotTrajoptDir()}")
        val traj = TrajoptLoader.loadFirstTrajectory(projectFile)
            ?: error("No trajectory found in ${projectFile.name}")
        val initialSample = traj.getInitialSample()
            ?: error("Trajectory has no samples")

        println("Trajectory: '${traj.name}'  totalTime=${traj.totalTime}s")

        // Only benchmark solvers that are available in this build
        val solversToTest = listOf(QpSolverType.FISTA, QpSolverType.NEON_IPM, QpSolverType.HPIPM_OCP)
            .filter { it.isAvailable }

        println("Testing ${solversToTest.size} solvers: ${solversToTest.map { it.name }}")

        // Pre-generate disturbance noise table (same for every solver)
        val rng = Random(NOISE_SEED)
        val noiseTable = mutableListOf<DoubleArray>()

        // Run each solver independently to completion
        val results = solversToTest.map { type ->
            val ltv = LTVClient(drivetrainParameters, solverType = type)
            ltv.loadTrajectory(traj)
            val sim = SimIO()
            sim.setPosition(initialSample.pos)
            val state = State(sim)

            val solveTimes    = mutableListOf<Long>()
            val trajectory    = mutableListOf(Vector2d(sim.position().v.x, sim.position().v.y))
            val posErrors     = mutableListOf<Double>()
            val headingErrors = mutableListOf<Double>()
            val timeAxis      = mutableListOf<Double>()

            var settleStepsRemaining = SETTLE_STEPS
            var stepIdx = 0

            while (settleStepsRemaining > 0) {
                if (ltv.isComplete(sim.time())) settleStepsRemaining--

                state.update(sim)
                val elapsed = sim.time()

                val u: DoubleArray
                val solveNs = measureNanoTime { u = ltv.solve(state.mecanumState, elapsed) }
                solveTimes.add(solveNs)

                // Noise shared across solvers via pre-generated table
                val noise = if (stepIdx < noiseTable.size) {
                    noiseTable[stepIdx]
                } else {
                    val n = DoubleArray(4) { rng.nextGaussian() * NOISE_STDDEV }
                    noiseTable.add(n)
                    n
                }

                sim.driveFL = (u[0] + noise[0]).coerceIn(-1.0, 1.0)
                sim.driveBL = (u[1] + noise[1]).coerceIn(-1.0, 1.0)
                sim.driveBR = (u[2] + noise[2]).coerceIn(-1.0, 1.0)
                sim.driveFR = (u[3] + noise[3]).coerceIn(-1.0, 1.0)
                sim.update()

                val pos    = state.driveTrainPosition
                val winRef = ltv.getWindowRef(ltv.prevWindowIdx())!!
                timeAxis.add(sim.time().inWholeMilliseconds / 1000.0)
                posErrors.add(hypot(pos.v.x - winRef[0], pos.v.y - winRef[1]))
                headingErrors.add(abs(pos.rot - winRef[2]))
                trajectory.add(Vector2d(sim.position().v.x, sim.position().v.y))
                stepIdx++
            }

            ltv.close()
            SolverRun(type, solveTimes, trajectory, posErrors, headingErrors, timeAxis)
        }

        // Print timing summary table
        println("\n%-12s %9s %9s %9s %9s %9s".format("Solver", "Mean(µs)", "Med(µs)", "p95(µs)", "Max(µs)", "FinalErr"))
        println("-".repeat(62))
        for (run in results) {
            val sorted = run.solveTimes.sorted()
            val mean   = sorted.average() / 1_000.0
            val med    = sorted[sorted.size / 2] / 1_000.0
            val p95    = sorted[(sorted.size * 0.95).toInt().coerceAtMost(sorted.lastIndex)] / 1_000.0
            val max    = sorted.last() / 1_000.0
            val finalErr = run.posErrors.lastOrNull() ?: 0.0
            println("%-12s %9.1f %9.1f %9.1f %9.1f %9.4f".format(
                run.type.name, mean, med, p95, max, finalErr))
        }

        // Save to Rerun
        val outputDir = File(System.getProperty("user.dir")!!, "rerun")
        outputDir.mkdirs()
        val rrdPath = File(outputDir, "ltv_solver_timing.rrd").absolutePath
        val rr = RerunLogging.save("ltv_solver_timing", rrdPath)
        println("\nRecording to: $rrdPath")

        // Static reference path
        rr.logLineStripStatic(
            "reference/trajectory",
            traj.samples.map { Vector2d(it.x, it.y) },
        )

        // Set series colors before logging any data
        for (run in results) {
            val color = solverColors[run.type] ?: 0xFFFFFFFF.toInt()
            rr.logSeriesColor("metrics/solve_time_us/${run.type.name}", color)
            rr.logSeriesColor("metrics/pos_error/${run.type.name}",    color)
            rr.logSeriesColor("metrics/heading_error/${run.type.name}", color)
        }

        // Time-indexed logs
        val nSteps  = results.maxOf { it.timeAxis.size }
        val refRun  = results.maxBy { it.timeAxis.size }

        for (step in 0 until nSteps) {
            rr.setSimTime(refRun.timeAxis[step])

            for (run in results) {
                if (step >= run.timeAxis.size) continue
                val name  = run.type.name
                val color = solverColors[run.type] ?: 0xFFFFFFFF.toInt()

                rr.logScalar("metrics/solve_time_us/$name",
                    run.solveTimes[step].nanoseconds.inWholeMicroseconds.toDouble())
                rr.logScalar("metrics/pos_error/$name",     run.posErrors[step])
                rr.logScalar("metrics/heading_error/$name", run.headingErrors[step])

                val pos = run.trajectory.getOrNull(step + 1) ?: run.trajectory.last()
                rr.logPoints2D("solver/$name/position", listOf(pos), color, radius = 3f)
            }
        }

        // Static actual trajectories (logged once)
        for (run in results) {
            val color = solverColors[run.type] ?: 0xFFFFFFFF.toInt()
            rr.logLineStripStatic("solver/${run.type.name}/trajectory", run.trajectory, color)
        }

        rr.close()
        println("Done. Replay with:  rerun $rrdPath")
    }
}
