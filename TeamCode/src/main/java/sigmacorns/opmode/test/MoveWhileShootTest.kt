package sigmacorns.opmode.test

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.control.Robot
import sigmacorns.control.aim.MoveWhileShootConfig
import sigmacorns.control.aim.MoveWhileShootSolver
import sigmacorns.control.aim.MuzzleSpeedMapping
import sigmacorns.constants.drivetrainParameters
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.MecanumDynamics
import sigmacorns.sim.MecanumState
import org.joml.Vector2d
import org.joml.Vector3d
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.hypot

/**
 * Test OpMode for MoveWhileShoot system.
 *
 * Lightweight version: computes the shot solution for the current robot state
 * at T=0 (no trajectory prediction, no binary search). Just solves the 3D
 * kinematics for "if I shot right now" and applies the result.
 */
@TeleOp(name = "MoveWhileShoot Test", group = "Test")
class MoveWhileShootTest : SigmaOpMode() {

    @Configurable
    object MWSTestConfig {
        @JvmField var mwsEnabled = true
    }

    private lateinit var robot: Robot

    // Profiling
    private var mwsSolveTimeMs = 0.0
    private var loopTimeMs = 0.0
    private var loopCount = 0L
    private val EMA_ALPHA = 0.1

    override fun runOpMode() {
        robot = Robot(io, true)
        robot.init(Pose2d(0.0, 0.0, PI / 2.0), true)
        robot.startApriltag()

        val cfg = MoveWhileShootConfig
        val goalPosition2d = robot.aim.goalPosition
        val goalPosition3d = Vector3d(goalPosition2d.x, goalPosition2d.y, cfg.goalHeight)
        val launchOffset = Vector3d(cfg.launchOffsetX, cfg.launchOffsetY, cfg.launchOffsetZ)

        val shootingZone = object : MoveWhileShootSolver.ShootingZone {
            override fun contains(pos: Vector2d): Boolean {
                val dx = pos.x - goalPosition2d.x
                val dy = pos.y - goalPosition2d.y
                val distance = hypot(dx, dy)
                return distance in 1.0..3.5 && pos.y < goalPosition2d.y
            }

            override fun closestTo(pos: Vector2d): Vector2d {
                val dx = pos.x - goalPosition2d.x
                val dy = pos.y - goalPosition2d.y
                val distance = hypot(dx, dy)
                val clampedDist = distance.coerceIn(1.0, 3.5)
                val scale = if (distance > 0.01) clampedDist / distance else 1.0
                val result = Vector2d(
                    goalPosition2d.x + dx * scale,
                    goalPosition2d.y + dy * scale
                )
                if (result.y >= goalPosition2d.y) {
                    result.y = goalPosition2d.y - 0.1
                }
                return result
            }
        }

        // Build muzzle speed mapping from tuner data
        val dz = goalPosition3d.z - launchOffset.z
        val mapping = MuzzleSpeedMapping.buildFromTuner(
            tunerPoints = robot.aim.adaptiveTuner.getPointsSorted(),
            dz = dz,
            alpha = cfg.launchAngle,
            g = cfg.gravity
        )

        val solver = MoveWhileShootSolver(
            validShootingZone = shootingZone,
            muzzleSpeedMapping = mapping,
            goalPosition3d = goalPosition3d,
            launchOffset = launchOffset,
            mecanumDynamics = MecanumDynamics(drivetrainParameters),
        )

        var wasShooting = false

        telemetry.addLine("MoveWhileShoot Test ready (lightweight)")
        telemetry.addLine("Drive around — shoots automatically when feasible")
        telemetry.update()

        try {
            waitForStart()
            if (isStopRequested) return

            var lastLoopNanos = System.nanoTime()

            ioLoop { state, dt ->
                val loopStartNanos = System.nanoTime()
                val loopDtMs = (loopStartNanos - lastLoopNanos) / 1e6
                lastLoopNanos = loopStartNanos
                loopTimeMs = if (loopCount == 0L) loopDtMs else loopTimeMs * (1 - EMA_ALPHA) + loopDtMs * EMA_ALPHA

                val fusedPose = robot.aim.autoAim.fusedPose
                val odomVelocity = io.velocity()

                val hybridState = MecanumState(
                    vel = odomVelocity,
                    pos = fusedPose
                )

                // Single-shot solve: compute solution at T=transferTime with current state
                val mwsSolveStart = System.nanoTime()
                val solution = if (MWSTestConfig.mwsEnabled) {
                    solver.computeSolutionAtTime(
                        T = cfg.transferTime,
                        predictedState = hybridState,
                        currentFlywheelSpeed = state.flywheelSpeed
                    )
                } else {
                    null
                }
                val mwsSolveElapsedMs = (System.nanoTime() - mwsSolveStart) / 1e6
                mwsSolveTimeMs = if (loopCount == 0L) mwsSolveElapsedMs
                    else mwsSolveTimeMs * (1 - EMA_ALPHA) + mwsSolveElapsedMs * EMA_ALPHA
                loopCount++

                // Apply turret targeting
                if (solution != null) {
                    robot.aim.turret.fieldRelativeMode = true
                    robot.aim.turret.fieldTargetAngle = fusedPose.rot + solution.turretAngle
                    robot.logic.shotVelocity = solution.flywheelSpeed.coerceAtLeast(0.0)
                    robot.aimTurret = false
                    robot.aimFlywheel = false
                } else {
                    robot.aim.applyAutoAimTarget()
                }

                // Manual shoot takes priority over auto-shoot
                val manualShoot = gamepad1.right_trigger.absoluteValue > 0.1
                val isReady = solution?.feasible == true && solution.inShootingZone
                if (manualShoot) {
                    robot.logic.shootingRequested = true
                    robot.logic.shoot()
                } else {
                    robot.logic.shootingRequested = isReady
                }
                wasShooting = isReady || manualShoot

                robot.drive.update(gamepad1, io)
                robot.update()

                // Telemetry
                telemetry.addLine("=== PROFILING ===")
                telemetry.addData("Loop Time (ms)", "%.1f", loopTimeMs)
                telemetry.addData("MWS Solve (ms)", "%.2f", mwsSolveTimeMs)
                telemetry.addData("MWS Solve Now (ms)", "%.2f", mwsSolveElapsedMs)
                telemetry.addData("Loop Hz", "%.0f", if (loopTimeMs > 0) 1000.0 / loopTimeMs else 0.0)

                telemetry.addLine("")
                telemetry.addLine("=== MOVE-WHILE-SHOOT ===")
                telemetry.addData("MWS Enabled", MWSTestConfig.mwsEnabled)
                telemetry.addData("Tuner Points", robot.aim.adaptiveTuner.pointCount())

                if (solution != null) {
                    telemetry.addData("Turret Angle (deg)", "%.1f", Math.toDegrees(solution.turretAngle))
                    telemetry.addData("Flywheel Speed (rad/s)", "%.0f", solution.flywheelSpeed)
                    telemetry.addData("Time of Flight (s)", "%.3f", solution.timeOfFlight)
                    telemetry.addData("In Shooting Zone", solution.inShootingZone)
                    telemetry.addData("Feasible", solution.feasible)
                    telemetry.addData("Ready", isReady)
                } else {
                    telemetry.addData("MWS Status", "No solution / disabled")
                }

                telemetry.addLine("")
                telemetry.addLine("=== STATE ===")
                telemetry.addData("Position (m)", "%.2f, %.2f", fusedPose.v.x, fusedPose.v.y)
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(fusedPose.rot))
                telemetry.addData("Speed (m/s)", "%.2f", odomVelocity.v.length())
                telemetry.addData("Distance to Goal (m)", "%.2f", robot.aim.targetDistance)

                telemetry.addLine("")
                telemetry.addLine("=== SUBSYSTEMS ===")
                telemetry.addData("Spindexer", robot.logic.currentState.name)
                telemetry.addData("Turret (deg)", "%.1f", Math.toDegrees(robot.aim.turret.pos))
                telemetry.addData("Tags", robot.aim.autoAim.detectedTagCount)

                telemetry.update()
                false
            }
        } finally {
            robot.close()
        }
    }
}
