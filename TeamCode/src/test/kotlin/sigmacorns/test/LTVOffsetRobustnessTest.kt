package sigmacorns.test

import org.joml.Vector2d
import org.joml.Vector3d
import org.junit.jupiter.api.Test
import sigmacorns.State
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.ltv.LTVClient
import sigmacorns.control.ltv.QpSolverType
import sigmacorns.control.mpc.TrajoptLoader
import sigmacorns.control.mpc.TrajoptTrajectory
import sigmacorns.io.RerunLogging
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.opmode.test.MPCTest
import java.io.File
import kotlin.math.hypot
import kotlin.math.max
import kotlin.system.measureNanoTime
import kotlin.time.Duration.Companion.nanoseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit
import kotlin.time.measureTime

/**
 * Tests HPIPM_OCP LTV robustness across a 5×5×5 grid of starting offsets.
 *
 * All 125 simulations run in lockstep so every timestep logs a single point
 * cloud showing where each trial's robot is right now, letting you scrub
 * through time and watch all trajectories evolve simultaneously.
 *
 * Entity layout:
 *   reference/trajectory                    — static blue reference path
 *   cases/positions                         — point cloud of all 125 positions (time-indexed)
 *   cases/trajectory/{label}               — static actual trajectory per trial
 *   cases/errors/pos/{label}               — position error over sim_time per trial
 *   cases/errors/heading/{label}           — heading error over sim_time per trial
 *
 * Colors: hue = heading offset, brightness = position offset magnitude.
 *         All three logged items for a trial share the same color.
 *
 * Output: rerun/ltv_offset_test.rrd
 */
class LTVOffsetRobustnessTest {

    data class OffsetCase(val dx: Double, val dy: Double, val dHeadingDeg: Double) {
        val label: String get() {
            fun Double.fmt() = "%.2f".format(this).replace("-", "n").replace(".", "p")
            val hStr = dHeadingDeg.toInt().toString().replace("-", "n")
            return "dx${dx.fmt()}_dy${dy.fmt()}_dh${hStr}deg"
        }
    }

    /** HSV→ARGB. h/s/v all in [0,1]. */
    private fun hsv(h: Float, s: Float, v: Float): Int {
        val h6 = h * 6f
        val i = h6.toInt()
        val f = h6 - i
        val p = v * (1 - s); val q = v * (1 - f * s); val t = v * (1 - (1 - f) * s)
        val (r, g, b) = when (i % 6) {
            0 -> Triple(v, t, p); 1 -> Triple(q, v, p); 2 -> Triple(p, v, t)
            3 -> Triple(p, q, v); 4 -> Triple(t, p, v); else -> Triple(v, p, q)
        }
        return (0xFF shl 24) or ((r * 255).toInt() shl 16) or ((g * 255).toInt() shl 8) or (b * 255).toInt()
    }

    /**
     * Color by initial offset:
     *   hue        = heading offset  −120°..+120° → 0..0.83  (red→yellow→green→cyan→blue→magenta)
     *   saturation = position magnitude  0..√2 m  → 0.35..1.0
     *                (base of 0.35 keeps colors discernible even at zero position offset)
     *   value      = 1.0 (always full brightness)
     */
    private fun offsetColor(case: OffsetCase): Int {
        val hue = ((case.dHeadingDeg + 120.0) / 240.0).toFloat().coerceIn(0f, 1f)
        val posFraction = (hypot(case.dx, case.dy) / Math.sqrt(2.0)).toFloat().coerceIn(0f, 1f)
        val saturation = 0.35f + 0.65f * posFraction
        return hsv(hue, saturation, 1f)
    }

    private class TrialResult(
        val case: OffsetCase,
        val color: Int,
        // trajectory[i] = position at step i (including initial position at index 0)
        val trajectory: List<Vector2d>,
        // timeAxis[i] = sim time at step i (after sim.update()); same length as posErrors
        val timeAxis: List<Double>,
        val posErrors: List<Double>,
        val headingErrors: List<Double>,
    )

    @Test
    fun testLTVOffsetRobustness() {
        SigmaOpMode.SIM = true
        SigmaOpMode.LIMELIGHT_CONNECTED = false

        val projectFile = MPCTest.findProjectFile()
            ?: error("No trajopt project file found in ${TrajoptLoader.robotTrajoptDir()}")
        val traj = TrajoptLoader.loadFirstTrajectory(projectFile)
            ?: error("No trajectory found in ${projectFile.name}")
        val initialSample = traj.getInitialSample()
            ?: error("Trajectory has no samples")

        println("Trajectory: '${traj.name}'  totalTime=${traj.totalTime}s")

        val outputDir = File(System.getProperty("user.dir")!!, "rerun")
        outputDir.mkdirs()
        val rrdPath = File(outputDir, "ltv_offset_test.rrd").absolutePath
        val rr = RerunLogging.save("ltv_offset_test", rrdPath)
        println("Recording to: $rrdPath")

        // Static reference trajectory
        rr.logLineStripStatic("reference/trajectory", traj.samples.map { Vector2d(it.x, it.y) })

        // Precompute HPIPM windows once, save to temp file for fast per-trial reload.
        print("Precomputing HPIPM_OCP windows... ")
        val tempWindows = LTVClient(drivetrainParameters, solverType = QpSolverType.HPIPM_OCP)
        tempWindows.loadTrajectory(traj)
        val numWindows = tempWindows.numWindows()
        println("done ($numWindows windows, dt=${tempWindows.dt})")

        val tempFile = java.io.File.createTempFile("ltv_windows", ".bin")
        try {
            tempWindows.saveWindows(tempFile.absolutePath)
            tempWindows.close()

            // Build 5×5×5 = 125 offset cases
            val cases = listOf(-1.0, -0.3, 0.0, 0.3, 1.0).flatMap { dx ->
                listOf(-1.0, -0.3, 0.0, 0.3, 1.0).flatMap { dy ->
                    listOf(-120.0, -20.0, 0.0, 20.0, 120.0).map { dh ->
                        OffsetCase(dx, dy, dh)
                    }
                }
            }

            // Run each trial sequentially to completion with a fresh LTVClient.
            // trajectory[0] = initial position; trajectory[i+1] = position after step i.
            // timeAxis[i] / posErrors[i] / headingErrors[i] are recorded AFTER sim.update()
            // so they align with trajectory[i+1].
            println("Running ${cases.size} trials sequentially...")
            val results = cases.mapIndexed { caseIdx, case ->
                val ltv = LTVClient.fromPrecomputed(tempFile.absolutePath, QpSolverType.HPIPM_OCP)
                val sim = SimIO()
                sim.setPosition(Pose2d(
                    initialSample.x + case.dx,
                    initialSample.y + case.dy,
                    initialSample.heading + Math.toRadians(case.dHeadingDeg),
                ))
                val state = State(sim)

                val trajectory    = mutableListOf(Vector2d(sim.position().v.x, sim.position().v.y))
                val timeAxis      = mutableListOf<Double>()
                val posErrors     = mutableListOf<Double>()
                val headingErrors = mutableListOf<Double>()

                var maxSolveTime = 0L
                var sumSolveTime = 0L
                var n = 0
                var t = 0.0

                while (!ltv.isComplete(sim.time())) {
                    state.update(sim)
                    val elapsed = sim.time()

                    val u: DoubleArray
                    val solveTime = measureNanoTime {
                        u = ltv.solve(state.mecanumState, elapsed)
                    }

                    maxSolveTime = max(solveTime,maxSolveTime)
                    sumSolveTime += solveTime
                    n++

                    sim.driveFL = u[0]; sim.driveBL = u[1]
                    sim.driveBR = u[2]; sim.driveFR = u[3]
                    sim.update()

                    t = sim.time().toDouble(DurationUnit.SECONDS)
                    val pos = state.driveTrainPosition
                    val winRef = ltv.getWindowRef(ltv.prevWindowIdx())!!
                    timeAxis.add(t)
                    posErrors.add(hypot(pos.v.x - winRef[0], pos.v.y - winRef[1]))
                    headingErrors.add(pos.rot - winRef[2])
                    trajectory.add(Vector2d(sim.position().v.x, sim.position().v.y))
                }
                val meanSolveTime = sumSolveTime/n

                println("trial ${case.label} solve times: " +
                        "mean=${meanSolveTime.nanoseconds.inWholeMicroseconds}μs " +
                        "max=${maxSolveTime.nanoseconds.inWholeMicroseconds}μs " +
                        "sum=${sumSolveTime.nanoseconds.inWholeMicroseconds}μs " +
                        "simTime=${sim.time().inWholeMicroseconds}μs")

                ltv.close()
                if ((caseIdx + 1) % 25 == 0) println("  ${caseIdx + 1}/${cases.size} done")

                TrialResult(case, offsetColor(case), trajectory, timeAxis, posErrors, headingErrors)
            }

            // Iterate over all steps up to the longest trial.
            // Trials that finish early hold their final position in the point cloud.
            val nSteps = results.maxOf { it.timeAxis.size }
            val longestResult = results.maxBy { it.timeAxis.size }

            println("Logging ${results.size} trials to Rerun (max $nSteps steps)...")
            for (result in results) {
                rr.logSeriesColor("cases/errors/pos/${result.case.label}", result.color)
                rr.logSeriesColor("cases/errors/heading/${result.case.label}", result.color)
            }

            for (step in 0 until nSteps) {
                rr.setSimTime(longestResult.timeAxis[step])

                val cloudPts    = ArrayList<Vector3d>(results.size)
                val cloudColors = IntArray(results.size)

                for ((i, result) in results.withIndex()) {
                    // Clamp to last recorded position once a trial completes
                    val trajIdx = (step + 1).coerceAtMost(result.trajectory.lastIndex)
                    val pos = result.trajectory[trajIdx]
                    cloudPts.add(Vector3d(pos.x, pos.y, 0.0))
                    cloudColors[i] = result.color
                    // Only log scalars while the trial is still running
                    if (step < result.timeAxis.size) {
                        rr.logScalar("cases/errors/pos/${result.case.label}", result.posErrors[step])
                        rr.logScalar("cases/errors/heading/${result.case.label}", result.headingErrors[step])
                    }
                }
                rr.logPoints3DWithColors("cases/positions", cloudPts, cloudColors, radius = 0.03f)
            }

            for (result in results) {
                rr.logLineStripStatic("cases/trajectory/${result.case.label}", result.trajectory, result.color)
            }
        } finally {
            tempFile.delete()
        }

        rr.close()
        println("Done. Saved to $rrdPath")
    }
}
