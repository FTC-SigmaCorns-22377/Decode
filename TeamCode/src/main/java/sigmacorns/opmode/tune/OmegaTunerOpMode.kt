package sigmacorns.opmode.tune

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.control.aim.tune.BisectionStore
import sigmacorns.control.aim.tune.BisectionTuner
import sigmacorns.control.aim.tune.OmegaCoefFitter
import sigmacorns.control.aim.tune.OmegaTunerWebServer
import sigmacorns.control.aim.tune.ShotDataStore
import sigmacorns.control.aim.tune.ShotOutcome
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import kotlin.math.PI
import kotlin.math.abs

/**
 * Omega-map bisection tuner.
 *
 * Procedure per cell:
 *  1. Drive until distance display shows you're within the green zone for the target distance.
 *  2. X to shoot (flywheel is pre-spun automatically).
 *  3. Watch the ball: LB = short, RB = long, A = made.
 *  4. After [N_MAKES_TARGET] makes, cell completes; web UI auto-advances.
 *  5. Repeat for all 18 cells. Press Start when done to refit coefficients.
 *
 * Bisection converges quickly because it's seeded from the current polynomial fit.
 * Multiple makes per cell at slightly varied positions give a noise-robust fit.
 *
 * Controls:
 *   Left stick        drive
 *   Right stick X     rotate
 *   X                 shoot
 *   LB                result: SHORT
 *   RB                result: LONG
 *   A                 result: MADE
 *   Y                 skip current cell
 *   Back              undo last shot result
 *   D-pad L/R         navigate cells manually
 *   Start             reset current cell (clears samples, restores seed bracket)
 *
 * Web UI: robot-ip:8083
 */
@TeleOp(name = "Omega Bisect Tuner", group = "Tune")
class OmegaTunerOpMode : SigmaOpMode() {

    companion object {
        private const val SHOT_WINDOW_MS = 2500L
        /** ±m around target before we show IN POSITION. */
        const val POS_TOLERANCE_M = 0.08
        /** Distance at which we warn "too close / too far" and refuse to shoot. */
        const val POS_HARD_LIMIT_M = 0.20
    }

    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = true)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = false)
        robot.aimFlywheel = false
        robot.shooter.autoAdjust = false

        val store = BisectionStore()
        store.load()

        val dataStore = ShotDataStore()
        dataStore.load()
        var currentCoeffs = OmegaCoefFitter.fit(dataStore.getPoints())

        val tuner = BisectionTuner(store)

        val webServer = OmegaTunerWebServer(
            port = 8083,
            tuner = tuner,
            store = store,
            getCoeffs = { currentCoeffs },
            getTargetDistance = { robot.aim.targetDistance },
            getActualOmega = { io.flywheelVelocity() }
        )
        webServer.start()

        // ── State ─────────────────────────────────────────────────────
        var cellIndex = 0
        var shotActive = false
        var shotStartMs = 0L
        var peakOmegaDuringShot = 0.0
        var awaitingOutcome = false
        var lastSlot2 = false

        // Debounce
        var lastX = false; var lastA = false
        var lastLB = false; var lastRB = false
        var lastY = false; var lastBack = false
        var lastStart = false
        var lastDpadL = false; var lastDpadR = false

        fun currentCell() = tuner.cells().getOrNull(cellIndex)

        fun autoAdvance() {
            val next = tuner.cells().indexOfFirst { !it.converged && !it.skipped }
            if (next >= 0) cellIndex = next
        }

        telemetry.addLine("Omega Bisect Tuner — waiting for start")
        telemetry.addLine("Connect browser to <robot-ip>:8083")
        telemetry.update()
        waitForStart()

        ioLoop { _, _ ->
            robot.drive.fieldCentricHeading = robot.aim.autoAim.fusedPose.rot - 0.5 * PI
            robot.drive.update(gamepad1, io)

            val cell = currentCell()
            val actualDist = robot.aim.targetDistance
            val distErr = if (cell != null) actualDist - cell.targetDistance else 0.0
            val inPosition = cell != null && abs(distErr) < POS_TOLERANCE_M
            val inHardLimit = cell != null && abs(distErr) > POS_HARD_LIMIT_M
            val actualOmega = io.flywheelVelocity()

            // ── Track peak omega during shot window ───────────────────
            if (shotActive && actualOmega > peakOmegaDuringShot)
                peakOmegaDuringShot = actualOmega

            // ── Detect ball exit via slot[2] falling edge ─────────────
            val slot2 = robot.beamBreak.slots[2]
            if (shotActive && lastSlot2 && !slot2) {
                awaitingOutcome = true
            }
            lastSlot2 = slot2

            // ── Shot timeout ──────────────────────────────────────────
            if (shotActive && System.currentTimeMillis() - shotStartMs > SHOT_WINDOW_MS) {
                shotActive = false
                awaitingOutcome = true
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }

            // ── Shooter control ───────────────────────────────────────
            if (cell != null && !cell.converged && !cell.skipped) {
                robot.shooter.manualHoodAngle = Math.toRadians(cell.targetHoodDeg)
                val targetOmega = cell.omegaMid
                if (shotActive) {
                    robot.shooter.flywheelTarget = targetOmega
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                } else if (!awaitingOutcome) {
                    // Pre-spin while driver positions
                    robot.shooter.flywheelTarget = targetOmega
                }
            }

            // ── Shoot (X) ─────────────────────────────────────────────
            if (gamepad1.x && !lastX && !shotActive && !awaitingOutcome &&
                cell != null && !cell.converged && !cell.skipped
            ) {
                if (inHardLimit) {
                    // Refuse — too far from target. Telemetry will say so.
                } else {
                    shotActive = true
                    shotStartMs = System.currentTimeMillis()
                    peakOmegaDuringShot = 0.0
                    robot.shooter.flywheelTarget = cell.omegaMid
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                }
            }
            lastX = gamepad1.x

            // ── Outcome buttons ────────────────────────────────────────
            if (awaitingOutcome && cell != null) {
                val commanded = cell.omegaMid
                val measured = peakOmegaDuringShot.takeIf { it > 10.0 } ?: commanded

                fun record(outcome: ShotOutcome) {
                    tuner.recordOutcome(
                        cell = cell,
                        outcome = outcome,
                        omegaMeasured = measured,
                        omegaCommanded = commanded,
                        actualDistance = actualDist,
                        actualHoodDeg = Math.toDegrees(robot.shooter.computedHoodAngle)
                    )
                    awaitingOutcome = false
                    shotActive = false
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                    if (cell.converged) autoAdvance()
                }

                if (gamepad1.left_bumper && !lastLB)  record(ShotOutcome.SHORT)
                if (gamepad1.right_bumper && !lastRB) record(ShotOutcome.LONG)
                if (gamepad1.a && !lastA)             record(ShotOutcome.MADE)
            }
            lastLB = gamepad1.left_bumper
            lastRB = gamepad1.right_bumper
            lastA = gamepad1.a

            // ── Skip (Y) ──────────────────────────────────────────────
            if (gamepad1.y && !lastY && cell != null) {
                tuner.skipCell(cell)
                autoAdvance()
                awaitingOutcome = false; shotActive = false
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }
            lastY = gamepad1.y

            // ── Undo last result (Back) ────────────────────────────────
            if (gamepad1.back && !lastBack && cell != null && !shotActive) {
                tuner.undoLastSample(cell)
                awaitingOutcome = false
            }
            lastBack = gamepad1.back

            // ── Navigate cells (D-pad L/R) ────────────────────────────
            if (gamepad1.dpad_left && !lastDpadL)
                cellIndex = (cellIndex - 1).coerceAtLeast(0)
            lastDpadL = gamepad1.dpad_left
            if (gamepad1.dpad_right && !lastDpadR)
                cellIndex = (cellIndex + 1).coerceAtMost((tuner.cells().size - 1).coerceAtLeast(0))
            lastDpadR = gamepad1.dpad_right

            // ── Reset current cell (Start) ────────────────────────────
            if (gamepad1.start && !lastStart && cell != null) {
                tuner.resetCell(cell)
                awaitingOutcome = false
                shotActive = false
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }
            lastStart = gamepad1.start

            // ── Robot update ──────────────────────────────────────────
            robot.update()
            io.update()

            // ── Telemetry ─────────────────────────────────────────────
            telemetry.addLine("=== OMEGA BISECT TUNER  [${cellIndex + 1}/${tuner.cells().size}] ===")
            if (cell != null) {
                // Position indicator
                val arrow = when {
                    inPosition -> ">>> IN POSITION <<<"
                    distErr > 0 -> "DRIVE CLOSER  [%.2fm too far]".format(distErr)
                    else        -> "DRIVE BACK    [%.2fm too close]".format(-distErr)
                }
                telemetry.addData("Position", arrow)
                telemetry.addData("Target d", "%.2fm  actual=%.2fm", cell.targetDistance, actualDist)
                telemetry.addData("Hood", "%.1f°", cell.targetHoodDeg)

                // Bracket + makes progress
                val collecting = tuner.isCollectingMakes(cell)
                telemetry.addLine("")
                telemetry.addData(
                    "Omega bracket",
                    "[%.0f, %.0f]  mid=%.0f  w=%.0f %s".format(
                        cell.omegaLo, cell.omegaHi, cell.omegaMid, cell.bracketWidth,
                        if (collecting) "(locked)" else "(bisecting)"
                    )
                )
                telemetry.addData(
                    "Makes",
                    "%d / %d".format(cell.nMades, BisectionTuner.N_MAKES_TARGET)
                )

                telemetry.addLine("")
                telemetry.addLine(when {
                    cell.converged      -> "DONE — advancing to next cell"
                    cell.skipped        -> "SKIPPED"
                    awaitingOutcome     -> ">>> LB=short  RB=long  A=made <<<"
                    shotActive          -> "SHOOTING...  peak_ω=%.0f rad/s".format(peakOmegaDuringShot)
                    inHardLimit         -> "TOO FAR FROM TARGET — reposition first"
                    !inPosition         -> "(drive to target distance)"
                    else                -> "X to shoot"
                })
            } else {
                telemetry.addLine("All cells complete! Press Start to refit.")
            }

            telemetry.addLine("")
            telemetry.addData("FW", "%.0f rad/s (target %.0f)".format(actualOmega, cell?.omegaMid ?: 0.0))
            telemetry.addData("Pending", tuner.pendingCells().size)
            telemetry.addData("Made samples", store.madeSamples().size)
            telemetry.addLine("Y=skip  Back=undo  Dpad=navigate  Start=reset cell")
            telemetry.update()

            false
        }

        webServer.stop()
        robot.close()
    }
}
