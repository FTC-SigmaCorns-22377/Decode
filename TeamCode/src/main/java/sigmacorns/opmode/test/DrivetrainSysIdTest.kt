package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.State
import sigmacorns.io.RerunLogging
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import java.io.File
import java.io.FileWriter
import java.util.Locale
import kotlin.time.Duration
import kotlin.time.DurationUnit

@TeleOp(name = "Drivetrain SysId", group = "test")
class DrivetrainSysIdTest : SigmaOpMode() {
    private enum class Axis {
        FORWARD,
        STRAFE,
        ROTATE;

        fun next(): Axis = values()[(ordinal + 1) % values().size]
        fun previous(): Axis = values()[(ordinal - 1 + values().size) % values().size]
        fun label(): String = when (this) {
            FORWARD -> "forward"
            STRAFE -> "strafe"
            ROTATE -> "rotate"
        }
    }

    private enum class TestType {
        QUASI_STATIC,
        STEP;

        fun next(): TestType = values()[(ordinal + 1) % values().size]
        fun previous(): TestType = values()[(ordinal - 1 + values().size) % values().size]
        fun label(): String = when (this) {
            QUASI_STATIC -> "quasi-static ramp"
            STEP -> "dynamic step"
        }
    }

    private enum class Direction {
        POSITIVE,
        NEGATIVE;

        fun toggled(): Direction = if (this == POSITIVE) NEGATIVE else POSITIVE
        fun multiplier(): Double = if (this == POSITIVE) 1.0 else -1.0
        fun label(axis: Axis): String {
            return when (axis) {
                Axis.FORWARD -> if (this == POSITIVE) "forward" else "reverse"
                Axis.STRAFE -> if (this == POSITIVE) "left" else "right"
                Axis.ROTATE -> if (this == POSITIVE) "ccw" else "cw"
            }
        }
    }

    private data class SysIdSample(
        val timeSeconds: Double,
        val command: Double,
        val wheelPowers: DoubleArray,
        val position: Pose2d,
        val velocity: Pose2d,
        val acceleration: Pose2d,
    )

    private data class SysIdRun(
        val axis: Axis,
        val type: TestType,
        val direction: Direction,
        val totalDuration: Double,
        var elapsed: Double = 0.0,
        var progress: Double = 0.0,
        var lastCommand: Double = 0.0,
        var baseTimestamp: Double? = null,
        val samples: MutableList<SysIdSample> = mutableListOf(),
    )

    override fun runOpMode() {
        telemetry.addLine("SysId ready: select axis/type, press A to run, B to abort")
        telemetry.addLine("D-pad ↑/↓ axis • ←/→ profile • X flips direction")
        telemetry.update()

        waitForStart()

        var selectedAxis = Axis.FORWARD
        var selectedType = TestType.QUASI_STATIC
        var selectedDirection = Direction.POSITIVE
        var currentRun: SysIdRun? = null

        var prevDpadUp = false
        var prevDpadDown = false
        var prevDpadLeft = false
        var prevDpadRight = false
        var prevA = false
        var prevB = false
        var prevX = false

        rerunSink("DrivetrainSysId").use { rerun ->
            ioLoop { state, dt ->
                val dpadUpPressed = edge(gamepad1.dpad_up, prevDpadUp).also { prevDpadUp = gamepad1.dpad_up }
                val dpadDownPressed = edge(gamepad1.dpad_down, prevDpadDown).also { prevDpadDown = gamepad1.dpad_down }
                val dpadLeftPressed = edge(gamepad1.dpad_left, prevDpadLeft).also { prevDpadLeft = gamepad1.dpad_left }
                val dpadRightPressed = edge(gamepad1.dpad_right, prevDpadRight).also { prevDpadRight = gamepad1.dpad_right }
                val aPressed = edge(gamepad1.a, prevA).also { prevA = gamepad1.a }
                val bPressed = edge(gamepad1.b, prevB).also { prevB = gamepad1.b }
                val xPressed = edge(gamepad1.x, prevX).also { prevX = gamepad1.x }

                if (currentRun == null) {
                    if (dpadUpPressed) selectedAxis = selectedAxis.previous()
                    if (dpadDownPressed) selectedAxis = selectedAxis.next()
                    if (dpadLeftPressed) selectedType = selectedType.previous()
                    if (dpadRightPressed) selectedType = selectedType.next()
                    if (xPressed) selectedDirection = selectedDirection.toggled()
                }

                if (aPressed && currentRun == null) {
                    currentRun = createRun(selectedAxis, selectedType, selectedDirection)
                    telemetry.log().add(
                        "Starting ${selectedType.label()} ${selectedAxis.label()} sweep (${selectedDirection.label(selectedAxis)})",
                    )
                }

                if (bPressed && currentRun != null) {
                    stopRun(currentRun!!, aborted = true)
                    currentRun = null
                    telemetry.log().add("SysId run aborted")
                }

                val run = currentRun
                if (run != null) {
                    val finished = updateRun(run, state, dt)
                    if (finished) {
                        stopRun(run, aborted = false)
                        currentRun = null
                    }
                } else {
                    zeroDrive()
                }

                telemetry.addData("axis", selectedAxis.label())
                telemetry.addData("profile", selectedType.label())
                telemetry.addData("direction", selectedDirection.label(selectedAxis))
                if (currentRun != null) {
                    telemetry.addData("status", "running")
                    telemetry.addData("progress", "%.0f%%", currentRun!!.progress * 100.0)
                    telemetry.addData("command", "%.3f", currentRun!!.lastCommand)
                    telemetry.addData("samples", currentRun!!.samples.size)
                } else {
                    telemetry.addData("status", "idle")
                    telemetry.addLine("Press A to capture data")
                }
                telemetry.update()

                rerun.logState(state)
                rerun.logInputs(io)

                false
            }
        }
    }

    private fun createRun(axis: Axis, type: TestType, direction: Direction): SysIdRun {
        val totalDuration = when (type) {
            TestType.QUASI_STATIC -> QUASI_STATIC_RAMP_DURATION + QUASI_STATIC_HOLD_DURATION + QUASI_STATIC_COAST_DURATION
            TestType.STEP -> STEP_PRE_DELAY + STEP_HOLD_DURATION + STEP_POST_DURATION
        }
        return SysIdRun(axis, type, direction, totalDuration)
    }

    private fun updateRun(run: SysIdRun, state: State, dt: Duration): Boolean {
        val dtSeconds = dt.toDouble(DurationUnit.SECONDS)
        run.elapsed += dtSeconds
        run.progress = (run.elapsed / run.totalDuration).coerceIn(0.0, 1.0)

        val commandMagnitude = when (run.type) {
            TestType.QUASI_STATIC -> quasiStaticProfile(run.elapsed)
            TestType.STEP -> stepProfile(run.elapsed)
        }

        val command = commandMagnitude * run.direction.multiplier()

        val wheelPattern = wheelPattern(run.axis)
        val wheelPowers = DoubleArray(4) { index ->
            wheelPattern[index] * command
        }

        applyWheelPowers(wheelPowers)

        run.lastCommand = command

        val absoluteTimestamp = state.timestamp.toDouble(DurationUnit.SECONDS)
        if (run.baseTimestamp == null) {
            run.baseTimestamp = absoluteTimestamp
        }
        val relativeTime = absoluteTimestamp - (run.baseTimestamp ?: absoluteTimestamp)

        run.samples.add(
            SysIdSample(
                timeSeconds = relativeTime,
                command = command,
                wheelPowers = wheelPowers.copyOf(),
                position = Pose2d(
                    state.driveTrainPosition.v.x,
                    state.driveTrainPosition.v.y,
                    state.driveTrainPosition.rot,
                ),
                velocity = Pose2d(
                    state.driveTrainVelocity.v.x,
                    state.driveTrainVelocity.v.y,
                    state.driveTrainVelocity.rot,
                ),
                acceleration = Pose2d(
                    state.driveTrainAcceleration.v.x,
                    state.driveTrainAcceleration.v.y,
                    state.driveTrainAcceleration.rot,
                ),
            ),
        )

        val finished = run.elapsed >= run.totalDuration
        if (finished) {
            zeroDrive()
        }

        return finished
    }

    private fun quasiStaticProfile(elapsed: Double): Double {
        return when {
            elapsed < QUASI_STATIC_RAMP_DURATION -> {
                val slope = QUASI_STATIC_MAX_POWER / QUASI_STATIC_RAMP_DURATION
                slope * elapsed
            }
            elapsed < QUASI_STATIC_RAMP_DURATION + QUASI_STATIC_HOLD_DURATION -> {
                QUASI_STATIC_MAX_POWER
            }
            elapsed < QUASI_STATIC_RAMP_DURATION + QUASI_STATIC_HOLD_DURATION + QUASI_STATIC_COAST_DURATION -> {
                0.0
            }
            else -> 0.0
        }
    }

    private fun stepProfile(elapsed: Double): Double {
        return when {
            elapsed < STEP_PRE_DELAY -> 0.0
            elapsed < STEP_PRE_DELAY + STEP_HOLD_DURATION -> STEP_POWER
            elapsed < STEP_PRE_DELAY + STEP_HOLD_DURATION + STEP_POST_DURATION -> 0.0
            else -> 0.0
        }
    }

    private fun wheelPattern(axis: Axis): DoubleArray {
        return when (axis) {
            Axis.FORWARD -> doubleArrayOf(1.0, 1.0, 1.0, 1.0)
            Axis.STRAFE -> doubleArrayOf(1.0, -1.0, 1.0, -1.0)
            Axis.ROTATE -> doubleArrayOf(-1.0, -1.0, 1.0, 1.0)
        }
    }

    private fun applyWheelPowers(powers: DoubleArray) {
        io.driveFL = powers[0].coerceIn(-1.0, 1.0)
        io.driveBL = powers[1].coerceIn(-1.0, 1.0)
        io.driveBR = powers[2].coerceIn(-1.0, 1.0)
        io.driveFR = powers[3].coerceIn(-1.0, 1.0)
    }

    private fun stopRun(run: SysIdRun, aborted: Boolean) {
        zeroDrive()
        if (run.samples.isEmpty()) {
            return
        }
        val baseDir = File(rerunLocation(), "sysid/drivetrain")
        if (!baseDir.exists()) {
            baseDir.mkdirs()
        }

        val suffix = if (aborted) "aborted" else "complete"
        val fileName = "drivetrain_${run.axis.label()}_${run.type.name.lowercase()}_${run.direction.label(run.axis)}_${suffix}_${System.currentTimeMillis()}.csv"
        val outputFile = File(baseDir, fileName)

        try {
            FileWriter(outputFile).use { writer ->
                writer.appendLine("time_seconds,axis,type,direction,command,driveFL,driveBL,driveBR,driveFR,vel_x,vel_y,omega,acc_x,acc_y,alpha,pos_x,pos_y,heading")
                val locale = Locale.US
                run.samples.forEach { sample ->
                    val line = String.format(
                        locale,
                        "%f,%s,%s,%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                        sample.timeSeconds,
                        run.axis.label(),
                        run.type.name.lowercase(),
                        run.direction.label(run.axis),
                        sample.command,
                        sample.wheelPowers[0],
                        sample.wheelPowers[1],
                        sample.wheelPowers[2],
                        sample.wheelPowers[3],
                        sample.velocity.v.x,
                        sample.velocity.v.y,
                        sample.velocity.rot,
                        sample.acceleration.v.x,
                        sample.acceleration.v.y,
                        sample.acceleration.rot,
                        sample.position.v.x,
                        sample.position.v.y,
                        sample.position.rot,
                    )
                    writer.appendLine(line)
                }
            }
            telemetry.log().add("SysId data saved: ${outputFile.canonicalPath}")
        } catch (ex: Exception) {
            telemetry.log().add("Failed to write SysId data: ${ex.message}")
        }
    }

    private fun zeroDrive() {
        io.driveFL = 0.0
        io.driveBL = 0.0
        io.driveBR = 0.0
        io.driveFR = 0.0
    }

    private fun edge(current: Boolean, previous: Boolean): Boolean = current && !previous

    private companion object {
        private const val QUASI_STATIC_MAX_POWER = 0.6
        private const val QUASI_STATIC_RAMP_DURATION = 6.0
        private const val QUASI_STATIC_HOLD_DURATION = 1.0
        private const val QUASI_STATIC_COAST_DURATION = 1.5

        private const val STEP_POWER = 0.7
        private const val STEP_PRE_DELAY = 0.8
        private const val STEP_HOLD_DURATION = 2.5
        private const val STEP_POST_DURATION = 1.2
    }
}
