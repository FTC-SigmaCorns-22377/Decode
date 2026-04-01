package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.joml.Vector2d
import sigmacorns.State
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.ltv.LTVClient
import sigmacorns.control.ltv.QpSolverType
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.io.SimIO
import sigmacorns.opmode.SigmaOpMode
import java.util.Random
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.system.measureNanoTime
import kotlin.time.Duration.Companion.nanoseconds

/**
 * On-robot benchmark opmode for the NEON_IPM and FISTA LTV solvers.
 *
 * Runs entirely in simulation — no motors are driven.  Two independent SimIO
 * instances advance in lockstep each control cycle, one per solver, so you
 * get a direct comparison of:
 *   - Solve time (ARM Cortex-A53 + optional NEON acceleration)
 *   - Closed-loop curve tracking under identical Gaussian disturbance noise
 *
 * Results are logged to telemetry and saved to an .rrd file on the robot's
 * SD card (/sdcard/FIRST/ltv_solver_benchmark.rrd) for offline Rerun replay.
 *
 * Running: deploy as a TeleOp, enable it via the driver station, then press
 * START.  The opmode ends automatically when both simulations complete.
 */
@TeleOp(name = "LTV Solver Benchmark", group = "benchmark")
class LTVSolverBenchmark : SigmaOpMode(SimIO()) {

    companion object {
        private const val SETTLE_STEPS = 30
        private const val NOISE_STDDEV = 0.05
        private const val NOISE_SEED   = 42L
    }

    /** Per-solver bookkeeping updated each step. */
    private inner class SolverEntry(
        val type: QpSolverType,
        val sim: SimIO,
        val ltv: LTVClient,
        val color: Int,
    ) {
        val state = State(sim)

        // Rolling statistics
        var steps          = 0
        var sumNs          = 0L
        var maxNs          = 0L
        var sumPosErr      = 0.0
        var maxPosErr      = 0.0

        // Rerun trajectory points
        val path = mutableListOf<Vector2d>()

        var lastSolveUs  = 0.0
        var lastPosErr   = 0.0
        var lastHeadingErr = 0.0
        var settled      = false
        var settleRemaining = SETTLE_STEPS

        fun meanSolveUs() = if (steps > 0) sumNs / steps / 1_000.0 else 0.0
        fun maxSolveUs()  = maxNs / 1_000.0
        fun meanPosErr()  = if (steps > 0) sumPosErr / steps else 0.0
    }

    override fun runOpMode() {
        val projectFile = TrajoptLoader.findProjectFile("simple")
            ?: error("No trajopt project file in ${TrajoptLoader.robotTrajoptDir()}")
        val traj = TrajoptLoader.loadFirstTrajectory(projectFile)
            ?: error("No trajectory in project")
        val initialSample = traj.getInitialSample()
            ?: error("Trajectory has no samples")

        // Only include solvers available in this build
        val types = listOf(QpSolverType.FISTA, QpSolverType.NEON_IPM, QpSolverType.HPIPM_OCP)
            .filter { it.isAvailable }

        val colors = mapOf(
            QpSolverType.FISTA     to 0xFF00C800.toInt(),
            QpSolverType.NEON_IPM  to 0xFFFFB400.toInt(),
            QpSolverType.HPIPM_OCP to 0xFF64C8FF.toInt(),
        )

        // Initialise one SimIO + LTVClient per solver
        val entries = types.map { type ->
            val sim = SimIO()
            sim.setPosition(initialSample.pos)
            val ltv = LTVClient(drivetrainParameters, solverType = type)
            ltv.loadTrajectory(traj)
            SolverEntry(type, sim, ltv, colors[type] ?: 0xFFFFFFFF.toInt())
        }

        // Open Rerun recording (saved to SD card on robot, connected on sim host)
        val rr = rerunSink("ltv_solver_benchmark")
        rr.logLineStripStatic(
            "reference/trajectory",
            traj.samples.map { Vector2d(it.x, it.y) },
        )
        for (e in entries) {
            rr.logSeriesColor("metrics/solve_time_us/${e.type.name}", e.color)
            rr.logSeriesColor("metrics/pos_error/${e.type.name}", e.color)
            rr.logSeriesColor("metrics/heading_error/${e.type.name}", e.color)
        }

        telemetry.addLine("Solvers: ${types.joinToString { it.name }}")
        telemetry.addLine("Trajectory: ${traj.name}  (${traj.totalTime}s)")
        telemetry.update()

        waitForStart()

        // Shared noise table (generated once, consumed identically by every solver)
        val rng = Random(NOISE_SEED)
        val noiseTable = mutableListOf<DoubleArray>()

        var step = 0

        while (opModeIsActive() && entries.any { !it.settled }) {
            // Grow noise table as needed (all solvers share the same noise at step k)
            if (step >= noiseTable.size) {
                noiseTable.add(DoubleArray(4) { rng.nextGaussian() * NOISE_STDDEV })
            }
            val noise = noiseTable[step]

            for (e in entries) {
                if (e.settled) continue

                if (e.ltv.isComplete(e.sim.time())) {
                    e.settleRemaining--
                    if (e.settleRemaining <= 0) { e.settled = true; continue }
                }

                e.state.update(e.sim)
                val elapsed = e.sim.time()

                val u: DoubleArray
                val ns = measureNanoTime { u = e.ltv.solve(e.state.mecanumState, elapsed) }

                e.sim.driveFL = (u[0] + noise[0]).coerceIn(-1.0, 1.0)
                e.sim.driveBL = (u[1] + noise[1]).coerceIn(-1.0, 1.0)
                e.sim.driveBR = (u[2] + noise[2]).coerceIn(-1.0, 1.0)
                e.sim.driveFR = (u[3] + noise[3]).coerceIn(-1.0, 1.0)
                e.sim.update()

                val pos    = e.state.driveTrainPosition
                val winRef = e.ltv.getWindowRef(e.ltv.prevWindowIdx())!!
                val posErr = hypot(pos.v.x - winRef[0], pos.v.y - winRef[1])
                val hdgErr = abs(pos.rot - winRef[2])

                e.steps++
                e.sumNs    += ns
                e.maxNs     = maxOf(e.maxNs, ns)
                e.sumPosErr += posErr
                e.maxPosErr = maxOf(e.maxPosErr, posErr)
                e.lastSolveUs    = ns.nanoseconds.inWholeMicroseconds.toDouble()
                e.lastPosErr     = posErr
                e.lastHeadingErr = hdgErr
                e.path.add(Vector2d(e.sim.position().v.x, e.sim.position().v.y))
            }

            // Log to Rerun at the first active solver's sim time
            val refEntry = entries.firstOrNull { !it.settled } ?: entries.first()
            rr.setSimTime(refEntry.sim.time().inWholeMilliseconds / 1000.0)

            for (e in entries) {
                if (e.steps == 0) continue
                val name = e.type.name
                rr.logScalar("metrics/solve_time_us/$name", e.lastSolveUs)
                rr.logScalar("metrics/pos_error/$name",     e.lastPosErr)
                rr.logScalar("metrics/heading_error/$name", e.lastHeadingErr)
                e.path.lastOrNull()?.let { pt ->
                    rr.logPoints2D("solver/$name/position", listOf(pt), e.color, radius = 3f)
                }
            }

            // Telemetry update (every step — driver station shows live stats)
            telemetry.clear()
            for (e in entries) {
                telemetry.addLine("── ${e.type.name} ──")
                telemetry.addData("  solve", "%.0fµs (max %.0fµs)".format(
                    e.lastSolveUs, e.maxSolveUs()))
                telemetry.addData("  pos err", "%.4f m (mean %.4f m)".format(
                    e.lastPosErr, e.meanPosErr()))
            }
            telemetry.update()

            step++
        }

        // Log final static trajectories for Rerun replay
        for (e in entries) {
            rr.logLineStripStatic("solver/${e.type.name}/trajectory", e.path, e.color)
        }

        // Print final summary to logs
        println("\n%-12s %9s %9s %9s %9s".format("Solver", "MeanSolve", "MaxSolve", "MeanErr", "MaxErr"))
        println("-".repeat(55))
        for (e in entries) {
            println("%-12s %8.1fµs %8.1fµs %8.4fm %8.4fm".format(
                e.type.name,
                e.meanSolveUs(), e.maxSolveUs(),
                e.meanPosErr(), e.maxPosErr,
            ))
        }

        rr.close()

        // Close LTVClients
        entries.forEach { it.ltv.close() }
    }
}
