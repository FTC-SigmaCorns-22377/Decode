package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.State
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.ltv.LTVClient
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumState
import kotlin.math.hypot
import kotlin.system.measureNanoTime
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

/**
 * Drives a 1m square using solveWaypoint, stopping fully at each corner before
 * advancing. Starts at (0, 0, 0°) and visits (1,0) → (1,1) → (0,1) → (0,0).
 *
 * Heading is held at 0° throughout. Uses lqrRef=true (zero-velocity arrival).
 *
 * tRemaining is seeded from the solver's own forward-simulated ETA each loop,
 * giving a self-consistent, model-derived horizon that shrinks naturally as the
 * robot approaches. A safety timeout per leg guards against non-convergence.
 */
@TeleOp(group = "test")
class WaypointSquareTest : SigmaOpMode() {

    // Position arrival threshold (metres)
    private val POS_TOL = 0.05
    // Velocity arrival threshold (m/s xy magnitude)
    private val VEL_TOL = 0.05
    // Initial tRemaining for the first call of each leg (before any ETA is available)
    private val INITIAL_T_REMAINING = 4.seconds
    // Minimum tRemaining passed to the solver (keeps N_eff >= 2 steps)
    private val MIN_T_REMAINING = 0.2.seconds

    override fun runOpMode() {
        // 1m square corners, heading held at 0° throughout
        val corners = listOf(
            Pose2d(1.0, 0.0, 0.0),
            Pose2d(1.0, 1.0, 0.0),
            Pose2d(0.0, 1.0, 0.0),
            Pose2d(0.0, 0.0, 0.0),
        )

        val robot = Robot(io,false)

        val state = State(io)
        io.setPosition(Pose2d(0.0, 0.0, 0.0))

        waitForStart()
        io.setPosition(Pose2d(0.0, 0.0, 0.0))

        for ((idx, corner) in corners.withIndex()) {
            val target      = MecanumState(vel = Pose2d(), pos = corner)
            val legStart    = io.time()

            // tRemaining for the first call — ETA not yet available
            var tRemaining: Duration = INITIAL_T_REMAINING

            while (opModeIsActive()) {
                state.update(io)

                var u = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
                val solveNs = measureNanoTime {
                    u = robot.ltv.solveWaypoint(state.mecanumState, target, tRemaining, lqrRef = true)
                }

                // Update tRemaining from the solver's own forward-simulated ETA.
                // Clamp from below so the horizon never collapses to a single step.
                tRemaining = robot.ltv.prevWaypointEta().coerceAtLeast(MIN_T_REMAINING)

                val voltage = io.voltage()
                io.driveFL = u[0] * 12.0 / voltage
                io.driveBL = u[1] * 12.0 / voltage
                io.driveBR = u[2] * 12.0 / voltage
                io.driveFR = u[3] * 12.0 / voltage

                val posErr = hypot(
                    state.driveTrainPosition.v.x - corner.v.x,
                    state.driveTrainPosition.v.y - corner.v.y,
                )
                val velMag = hypot(
                    state.driveTrainVelocity.v.x,
                    state.driveTrainVelocity.v.y,
                )

                telemetry.addData("leg", "${idx + 1} / ${corners.size}  →  (%.2f, %.2f)".format(corner.v.x, corner.v.y))
                telemetry.addData("pos", "(%.3f, %.3f, %.1f°)".format(
                    state.driveTrainPosition.v.x,
                    state.driveTrainPosition.v.y,
                    Math.toDegrees(state.driveTrainPosition.rot),
                ))
                telemetry.addData("posErr",    "%.3f m".format(posErr))
                telemetry.addData("velMag",    "%.3f m/s".format(velMag))
                telemetry.addData("eta",       "%.2f s".format(tRemaining.toDouble(DurationUnit.SECONDS)))
                telemetry.addData("legElapsed","%.2f s".format((io.time() - legStart).toDouble(DurationUnit.SECONDS)))
                telemetry.addData("solveTime", "%.2f ms".format(solveNs / 1_000_000.0))
                telemetry.update()

                io.update()

                if (SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)

                val arrived  = posErr < POS_TOL && velMag < VEL_TOL
                if(idx == 3) {
                    if(arrived) {
                        robot.ltv.holdPos(io, target.pos)
                    }
                    continue
                }
                if (arrived) break
            }

            // Stop all motors
            io.driveFL = 0.0
            io.driveBL = 0.0
            io.driveBR = 0.0
            io.driveFR = 0.0
            io.update()
        }
    }
}
