package sigmacorns.opmode.auto

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import sigmacorns.Robot
import sigmacorns.State
import sigmacorns.control.trajopt.TrajoptLoader
import sigmacorns.control.trajopt.TrajoptTrajectory
import sigmacorns.io.HardwareIO
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.logic.BallChaseAutoFSM
import sigmacorns.logic.BallTrackingSystem
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.vision.BallDetectionProcessor
import sigmacorns.vision.tracker.TrackerConfig
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

@Autonomous(name = "Auto 12 Ball Far", group = "Competition")
class Auto12Far: SigmaOpMode() {
    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir)
            .find { it.nameWithoutExtension == "12ballfar" }
            ?: throw IllegalStateException("12ballfar.json not found in $robotDir")

        val traj1 = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 1")!!
        val traj2 = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 2")!!
        val traj3 = TrajoptLoader.loadTrajectory(projectFile, "Trajectory 3")!!

        val initialSample = traj1.getInitialSample()
            ?: throw IllegalStateException("Trajectory has no samples")

        io.setPosition(initialSample.pos)

        val robot = Robot(io, blue = false)
        robot.init(initialSample.pos, apriltagTracking = false)
        robot.aimTurret = true
        robot.aimFlywheel = false

        // --- Vision pipeline (skip portal in SIM; tracker is sim-safe) ---
        val processor = if (!SIM) BallDetectionProcessor() else null
        val portal: VisionPortal? = if (!SIM && processor != null) {
            VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
                .addProcessor(processor)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(Size(1280, 720))
                .build()
        } else null
        val hardware = io as? HardwareIO
        if (processor != null) {
            hardware?.ballDetectionProvider = {
                processor.detectedBalls to processor.lastCaptureTimeSec
            }
        }

        // --- Tracker + chase FSM ---
        val tracking = BallTrackingSystem(config = TrackerConfig.loadDefault(), io = io)
        // FSM is only driven through CHASE in this opmode; we set
        // `enabled = false` before it can transition to DRIVE_TO_SHOOT_ZONE,
        // so `shootingZone` and `flywheelShootSpeed` are never consulted.
        val fsm = BallChaseAutoFSM(
            tracking = tracking,
            drivetrain = robot.drive,
            io = io,
            shootingZone = Pose2d(0.0, 0.0, 0.0),
            maxSpeed = 1.0,
            arrivalRadiusM = 0.18,
            chasePullInRadiusM = 0.35,
            flywheelShootSpeed = 0.0,
        )

        val state = State(
            0.0, io.position(), Pose2d(), Pose2d(), 0.0, 0.0, 0.seconds,
        )

        waitForStart()

        try {
            // Phase 1: Shoot all preloads using aimFlywheel
            robot.aimFlywheel = true
            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            var transferStartTime: Duration? = null

            while (opModeIsActive()) {
                state.update(io)
                robot.update()
                io.update()

                robot.aim.shotRequested = true

                // Track transfer start time
                if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING && transferStartTime == null) {
                    transferStartTime = io.time()
                }

                // Only exit if balls are gone AND 2 seconds have passed since transfer started
                if (robot.beamBreak.ballCount == 0 && transferStartTime != null &&
                    (io.time() - transferStartTime) >= 2.seconds) { // 2 seconds in nanoseconds
                    break
                }

                robot.aimFlywheel = false
            }

            robot.intakeTransfer.state = IntakeTransfer.State.IDLE

            // Phase 2: Run 3 trajectories with intake/shoot cycles
            val trajectories = listOf(traj1, traj2, traj3)

            for (traj in trajectories) {
                if (!opModeIsActive()) break

                // Reset chase state before each trajectory so stale tracks
                // and a leftover-enabled FSM from the previous run can't
                // bleed into this one.
                fsm.enabled = false
                tracking.reset()

                runTrajectoryWithIntakeShoot(robot, state, traj, traj == traj3, fsm, tracking)
            }

            io.driveFL = 0.0
            io.driveBL = 0.0
            io.driveBR = 0.0
            io.driveFR = 0.0
            robot.shooter.flywheelTarget = 0.0
            io.update()
        } finally {
            hardware?.ballDetectionProvider = null
            portal?.close()
            processor?.release()
        }
    }

    private fun runTrajectoryWithIntakeShoot(
        robot: Robot,
        state: State,
        traj: TrajoptTrajectory,
        last: Boolean,
        fsm: BallChaseAutoFSM,
        tracking: BallTrackingSystem,
    ) {
        robot.ltv.loadTrajectory(traj)
        var startTime = io.time()
        var transferStartTime: Duration? = null

        // Intake-window bounds (same indexing rule as the legacy code:
        // last trajectory uses waypoints [0,1]; others use [1,2]).
        val tIntakeStart = traj.waypointTimes.getOrNull(if (last) 0 else 1) ?: Double.MAX_VALUE
        val tIntakeEnd = traj.waypointTimes.getOrNull(if (last) 1 else 2) ?: Double.MAX_VALUE
        val chaseTimeoutSec = (tIntakeEnd - tIntakeStart) + 2.0

        var chaseDoneForThisTraj = false

        while (opModeIsActive()) {
            state.update(io)
            val elapsed = io.time() - startTime
            val elapsedSeconds = elapsed.toDouble(DurationUnit.SECONDS)

            // ----- Chase phase: vision-driven, replaces the timed intake window -----
            if (!chaseDoneForThisTraj && elapsedSeconds >= tIntakeStart) {
                val chaseStart = io.time()
                // Engage FSM. Inside this loop we deliberately DO NOT call
                // robot.update() — the FSM writes io.intake/io.blocker/io.flywheel
                // directly and any subsystem update would clobber it.
                fsm.enabled = true

                while (opModeIsActive()) {
                    state.update(io)
                    val tSec = state.timestamp.toDouble(DurationUnit.SECONDS)
                    tracking.update(tSec)
                    fsm.update()
                    io.update()

                    if (SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)

                    val held = BallChaseAutoFSM.countHeldFromBeamBreaks(io)
                    val chaseElapsedSec = (io.time() - chaseStart).toDouble(DurationUnit.SECONDS)

                    telemetry.addData("phase", "CHASE/${fsm.phase}")
                    telemetry.addData("chase_t", "%.2f / %.2f s".format(chaseElapsedSec, chaseTimeoutSec))
                    telemetry.addData("held", held)
                    val tgt = tracking.targetBallField
                    if (tgt != null) {
                        telemetry.addData("target", "(%.2f, %.2f)".format(tgt.v.x, tgt.v.y))
                    } else {
                        telemetry.addLine("no target")
                    }
                    telemetry.update()

                    if (held >= 3) break
                    if (chaseElapsedSec >= chaseTimeoutSec) break
                }

                // Disable FSM and run one more update so its idleAll() zeros
                // the drivetrain / intake / flywheel before LTV takes back over.
                fsm.enabled = false
                fsm.update()
                io.update()

                // Reset intake/transfer state so the post-intake branch's
                // aimFlywheel / shotRequested logic behaves exactly as today.
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE

                // Time-skip rejoin: shift startTime so the next iteration sees
                // elapsedSeconds == tIntakeEnd, jumping into the post-intake
                // (drive-to-shoot) leg of the trajectory regardless of how long
                // the chase actually took.
                startTime = io.time() - tIntakeEnd.seconds

                chaseDoneForThisTraj = true
                continue
            }

            // ----- LTV trajectory follow (pre-intake and post-intake) -----
            if (elapsedSeconds <= traj.totalTime) {
                val u = robot.ltv.solve(state.mecanumState, elapsed)
                io.driveFL = u[0]
                io.driveBL = u[1]
                io.driveBR = u[2]
                io.driveFR = u[3]
            } else {
                io.driveFL = 0.0
                io.driveBL = 0.0
                io.driveBR = 0.0
                io.driveFR = 0.0
            }

            // Pre-chase / post-chase intake + flywheel decisions.
            // Note: the original "inIntakeZone" branch is gone — that window
            // is now handled by the chase loop above. Anything before
            // tIntakeStart keeps intake idle; after tIntakeEnd the original
            // shoot logic fires unchanged.
            if (elapsedSeconds >= tIntakeEnd) {
                robot.aimFlywheel = true
                if (elapsedSeconds >= 0.9 * traj.totalTime) {
                    // Shoot at last waypoint (only after 90% of trajectory)
                    robot.aim.shotRequested = true
                }
            } else {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                robot.aimFlywheel = false
            }

            robot.update()

            // Track transfer start time
            if (robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING && transferStartTime == null) {
                transferStartTime = io.time()
            }

            telemetry.addData("elapsed", "%.2f / %.2f s".format(elapsedSeconds, traj.totalTime))
            telemetry.addData("state", robot.intakeTransfer.state)
            telemetry.addData("flywheel", if (robot.shooter.flywheelTarget > 0) "SPINNING" else "OFF")
            telemetry.addData("pos", "(%.3f, %.3f, %.1f°)".format(
                state.driveTrainPosition.v.x,
                state.driveTrainPosition.v.y,
                Math.toDegrees(state.driveTrainPosition.rot),
            ))
            telemetry.update()

            io.update()

            if (SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)

            // Exit only after trajectory completes AND 2 seconds have passed since transfer (if transfer happened)
            val canExit = if (transferStartTime != null) {
                (io.time() - transferStartTime) >= 2.seconds // 2 seconds in nanoseconds
            } else {
                true
            }

            if (elapsedSeconds > traj.totalTime + 1.5 && canExit) break
        }
    }
}