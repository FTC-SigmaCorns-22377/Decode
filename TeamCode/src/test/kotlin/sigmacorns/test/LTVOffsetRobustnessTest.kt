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
import sigmacorns.io.SIM_UPDATE_TIME
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

    companion object {
        /** Number of extra solver steps after the trajectory completes, to verify controlled settling. */
        const val SETTLE_STEPS = 30
    }

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
        val solveTimes: List<Long>, // individual solve times in nanoseconds
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
            val ltv = LTVClient(drivetrainParameters, solverType = QpSolverType.HPIPM_OCP)
            ltv.loadTrajectory(traj)
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
            val solveTimes    = mutableListOf<Long>()

            var maxSolveTime = 0L
            var sumSolveTime = 0L
            var n = 0
            var t = 0.0

            // Settling window: continue solving for SETTLE_STEPS past trajectory completion
            // so the controller actively brakes to zero velocity.
            var settleStepsRemaining = SETTLE_STEPS

            while (settleStepsRemaining > 0) {
                if (ltv.isComplete(sim.time())) settleStepsRemaining--

                state.update(sim)
                val elapsed = sim.time()

                val u: DoubleArray
                val solveTime = measureNanoTime {
                    u = ltv.solve(state.mecanumState, elapsed)
                }

                solveTimes.add(solveTime)
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

            TrialResult(case, offsetColor(case), trajectory, timeAxis, posErrors, headingErrors, solveTimes)
        }

        // Aggregate solve time percentiles across all trials
        val allSolveTimes = results.flatMap { it.solveTimes }.sorted()
        fun percentile(sorted: List<Long>, p: Double): Long {
            val idx = ((p / 100.0) * (sorted.size - 1)).toInt().coerceIn(0, sorted.lastIndex)
            return sorted[idx]
        }
        println("\nSolve time percentiles across ${allSolveTimes.size} total solves:")
        println("  p25 = ${percentile(allSolveTimes, 25.0).nanoseconds.inWholeMicroseconds}μs")
        println("  p50 = ${percentile(allSolveTimes, 50.0).nanoseconds.inWholeMicroseconds}μs")
        println("  p95 = ${percentile(allSolveTimes, 95.0).nanoseconds.inWholeMicroseconds}μs")
        println("  p99 = ${percentile(allSolveTimes, 99.0).nanoseconds.inWholeMicroseconds}μs")
        println("  max = ${allSolveTimes.last().nanoseconds.inWholeMicroseconds}μs")

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

        rr.close()
        println("Done. Saved to $rrdPath")
    }

    // ── Model-mismatch + external-force robustness test ────────────────────

    /**
     * @param gainScale  multiplicative factor on all motor outputs (1.0 = nominal)
     * @param forceN     magnitude of constant external force in Newtons
     * @param forceAngleDeg direction of external force in field frame (degrees)
     * @param torqueNm   constant external torque in N·m
     */
    private data class MismatchCase(
        val gainScale: Double,
        val forceN: Double,
        val forceAngleDeg: Double,
        val torqueNm: Double,
    ) {
        val label: String get() {
            fun Double.fmt() = "%.2f".format(this).replace("-", "n").replace(".", "p")
            val aStr = forceAngleDeg.toInt().toString().replace("-", "n")
            return "g${gainScale.fmt()}_f${forceN.fmt()}_a${aStr}_t${torqueNm.fmt()}"
        }
    }

    private class MismatchTrialResult(
        val case: MismatchCase,
        val color: Int,
        val trajectory: List<Vector2d>,
        val timeAxis: List<Double>,
        val posErrors: List<Double>,
        val headingErrors: List<Double>,
        val solveTimes: List<Long>,
    )

    /**
     * Color by total disturbance severity.
     * Combines gain deviation, force magnitude, and torque into a single 0..1 fraction,
     * then maps green (no disturbance) → red (max disturbance).
     */
    private fun mismatchColor(
        case: MismatchCase,
        maxGainDev: Double,
        maxForce: Double,
        maxTorque: Double,
    ): Int {
        val gainFrac  = if (maxGainDev > 0) Math.abs(case.gainScale - 1.0) / maxGainDev else 0.0
        val forceFrac = if (maxForce > 0) case.forceN / maxForce else 0.0
        val torqFrac  = if (maxTorque > 0) Math.abs(case.torqueNm) / maxTorque else 0.0
        val frac = ((gainFrac + forceFrac + torqFrac) / 3.0).toFloat().coerceIn(0f, 1f)
        val hue = 0.33f * (1f - frac) // green → red
        return hsv(hue, 0.9f, 1f)
    }

    /**
     * Tests LTV robustness against combined model mismatch and constant external
     * disturbance forces across all trajectories in the project.
     *
     * Disturbance axes:
     *   - Gain mismatch: u *= gainScale  (actuator over/under-performance)
     *   - External force: constant field-frame force (e.g. inclined surface, collision)
     *   - External torque: constant heading torque (e.g. asymmetric weight)
     *
     * Every combination of gain × force × direction × torque is tested on each
     * trajectory, starting from the correct pose.
     *
     * Entity layout (per trajectory, namespaced by traj name):
     *   {traj}/reference/trajectory                  — static blue reference path
     *   {traj}/cases/positions                       — point cloud of all positions (time-indexed)
     *   {traj}/cases/trajectory/{label}             — static actual trajectory per trial
     *   {traj}/cases/errors/pos/{label}             — position error over sim_time per trial
     *   {traj}/cases/errors/heading/{label}         — heading error over sim_time per trial
     *
     * Colors: green = no disturbance, red = severe disturbance.
     *
     * Output: rerun/ltv_mismatch_test.rrd
     */
    @Test
    fun testLTVModelMismatch() {
        SigmaOpMode.SIM = true
        SigmaOpMode.LIMELIGHT_CONNECTED = false

        val projectFile = MPCTest.findProjectFile()
            ?: error("No trajopt project file found in ${TrajoptLoader.robotTrajoptDir()}")
        val trajectories = TrajoptLoader.loadAllTrajectories(projectFile)
        require(trajectories.isNotEmpty()) { "No trajectories found in ${projectFile.name}" }

        println("Loaded ${trajectories.size} trajectories: ${trajectories.map { it.name }}")

        val outputDir = File(System.getProperty("user.dir")!!, "rerun")
        outputDir.mkdirs()
        val rrdPath = File(outputDir, "ltv_mismatch_test.rrd").absolutePath
        val rr = RerunLogging.save("ltv_mismatch_test", rrdPath)
        println("Recording to: $rrdPath")

        // Disturbance grid
        val gainScales     = listOf(0.7, 0.85, 1.0, 1.15, 1.3)
        val forceMagnitudes = listOf(0.0, 1.0, 3.0, 5.0)            // Newtons
        val forceAngles    = listOf(0.0, 90.0, 180.0, 270.0)        // degrees, field frame
        val torques        = listOf(0.0, 0.2, 0.5)                   // N·m

        val maxGainDev = gainScales.maxOf { Math.abs(it - 1.0) }
        val maxForce   = forceMagnitudes.max()
        val maxTorque  = torques.maxOf { Math.abs(it) }

        // Build all cases; skip force angle variation when force = 0
        val casesTemplate = gainScales.flatMap { g ->
            forceMagnitudes.flatMap { f ->
                val angles = if (f == 0.0) listOf(0.0) else forceAngles
                angles.flatMap { a ->
                    torques.map { t -> MismatchCase(g, f, a, t) }
                }
            }
        }

        val mass       = drivetrainParameters.weight
        val rotInertia = drivetrainParameters.rotInertia
        val simDt      = SIM_UPDATE_TIME.toDouble(DurationUnit.SECONDS)

        println("${casesTemplate.size} disturbance cases × ${trajectories.size} trajectories " +
                "= ${casesTemplate.size * trajectories.size} total trials")

        val allSolveTimes = mutableListOf<Long>()
        var trialCount = 0

        for (traj in trajectories) {
            val initialSample = traj.getInitialSample()
                ?: error("Trajectory '${traj.name}' has no samples")
            val ns = traj.name.replace(" ", "_")

            println("\n── Trajectory '${traj.name}' (totalTime=${traj.totalTime}s) ──")
            rr.logLineStripStatic("$ns/reference/trajectory", traj.samples.map { Vector2d(it.x, it.y) })

            println("Running ${casesTemplate.size} disturbance trials...")
            val results = casesTemplate.mapIndexed { caseIdx, case ->
                val ltv = LTVClient(drivetrainParameters, solverType = QpSolverType.HPIPM_OCP)
                ltv.loadTrajectory(traj)
                val sim = SimIO()
                sim.setPosition(Pose2d(initialSample.x, initialSample.y, initialSample.heading))
                val state = State(sim)

                // Precompute constant force accelerations (field frame)
                val forceAngleRad = Math.toRadians(case.forceAngleDeg)
                val axExt = case.forceN * Math.cos(forceAngleRad) / mass       // m/s² in x
                val ayExt = case.forceN * Math.sin(forceAngleRad) / mass       // m/s² in y
                val alphaExt = case.torqueNm / rotInertia                       // rad/s²

                val trajectory    = mutableListOf(Vector2d(sim.position().v.x, sim.position().v.y))
                val timeAxis      = mutableListOf<Double>()
                val posErrors     = mutableListOf<Double>()
                val headingErrors = mutableListOf<Double>()
                val solveTimes    = mutableListOf<Long>()

                var maxSolveTime = 0L
                var sumSolveTime = 0L
                var n = 0
                var settleStepsRemaining = SETTLE_STEPS

                while (settleStepsRemaining > 0) {
                    if (ltv.isComplete(sim.time())) settleStepsRemaining--

                    state.update(sim)
                    val elapsed = sim.time()

                    val u: DoubleArray
                    val solveTime = measureNanoTime {
                        u = ltv.solve(state.mecanumState, elapsed)
                    }

                    solveTimes.add(solveTime)
                    maxSolveTime = max(solveTime, maxSolveTime)
                    sumSolveTime += solveTime
                    n++

                    // Gain mismatch
                    sim.driveFL = (u[0] * case.gainScale).coerceIn(-1.0, 1.0)
                    sim.driveBL = (u[1] * case.gainScale).coerceIn(-1.0, 1.0)
                    sim.driveBR = (u[2] * case.gainScale).coerceIn(-1.0, 1.0)
                    sim.driveFR = (u[3] * case.gainScale).coerceIn(-1.0, 1.0)

                    // External force/torque: impulse applied after physics step
                    val ds = sim.robot.drivetrainState
                    sim.robot.drivetrainState.vel = Pose2d(
                        ds.vel.v.x + axExt * simDt,
                        ds.vel.v.y + ayExt * simDt,
                        ds.vel.rot + alphaExt * simDt,
                    )
                    sim.update()

                    val t = sim.time().toDouble(DurationUnit.SECONDS)
                    val pos = state.driveTrainPosition
                    val winRef = ltv.getWindowRef(ltv.prevWindowIdx())!!
                    timeAxis.add(t)
                    posErrors.add(hypot(pos.v.x - winRef[0], pos.v.y - winRef[1]))
                    headingErrors.add(pos.rot - winRef[2])
                    trajectory.add(Vector2d(sim.position().v.x, sim.position().v.y))
                }

                ltv.close()
                allSolveTimes.addAll(solveTimes)
                trialCount++
                if (trialCount % 50 == 0) println("  $trialCount trials done")

                val color = mismatchColor(case, maxGainDev, maxForce, maxTorque)
                MismatchTrialResult(case, color, trajectory, timeAxis, posErrors, headingErrors, solveTimes)
            }

            // Log to Rerun under this trajectory's namespace
            val nSteps = results.maxOf { it.timeAxis.size }
            val longestResult = results.maxBy { it.timeAxis.size }

            println("Logging ${results.size} trials to Rerun (max $nSteps steps)...")
            for (result in results) {
                rr.logSeriesColor("$ns/cases/errors/pos/${result.case.label}", result.color)
                rr.logSeriesColor("$ns/cases/errors/heading/${result.case.label}", result.color)
            }

            for (step in 0 until nSteps) {
                rr.setSimTime(longestResult.timeAxis[step])

                val cloudPts    = ArrayList<Vector3d>(results.size)
                val cloudColors = IntArray(results.size)

                for ((i, result) in results.withIndex()) {
                    val trajIdx = (step + 1).coerceAtMost(result.trajectory.lastIndex)
                    val pos = result.trajectory[trajIdx]
                    cloudPts.add(Vector3d(pos.x, pos.y, 0.0))
                    cloudColors[i] = result.color
                    if (step < result.timeAxis.size) {
                        rr.logScalar("$ns/cases/errors/pos/${result.case.label}", result.posErrors[step])
                        rr.logScalar("$ns/cases/errors/heading/${result.case.label}", result.headingErrors[step])
                    }
                }
                rr.logPoints3DWithColors("$ns/cases/positions", cloudPts, cloudColors, radius = 0.03f)
            }

            for (result in results) {
                rr.logLineStripStatic("$ns/cases/trajectory/${result.case.label}", result.trajectory, result.color)
            }
        }

        // Aggregate percentile summary across all trajectories
        val sorted = allSolveTimes.sorted()
        fun percentile(s: List<Long>, p: Double): Long {
            val idx = ((p / 100.0) * (s.size - 1)).toInt().coerceIn(0, s.lastIndex)
            return s[idx]
        }
        println("\nSolve time percentiles across ${sorted.size} total solves ($trialCount trials):")
        println("  p25 = ${percentile(sorted, 25.0).nanoseconds.inWholeMicroseconds}μs")
        println("  p50 = ${percentile(sorted, 50.0).nanoseconds.inWholeMicroseconds}μs")
        println("  p95 = ${percentile(sorted, 95.0).nanoseconds.inWholeMicroseconds}μs")
        println("  p99 = ${percentile(sorted, 99.0).nanoseconds.inWholeMicroseconds}μs")
        println("  max = ${sorted.last().nanoseconds.inWholeMicroseconds}μs")

        rr.close()
        println("Done. Saved to $rrdPath")
    }
}
