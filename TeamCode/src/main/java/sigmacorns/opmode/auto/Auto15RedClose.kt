package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import sigmacorns.Robot
import sigmacorns.State
import sigmacorns.control.trajopt.TrajoptLoader
import sigmacorns.control.trajopt.TrajoptTrajectory
import sigmacorns.io.SIM_UPDATE_TIME
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

@Autonomous(name = "Auto 15 Red Close", group = "Competition")
class Auto15RedClose : SigmaOpMode() {
    override fun runOpMode() {
        val projectFile = TrajoptLoader.findProjectFile("red-15-close-1")
            ?: throw IllegalStateException("red-15-close-1.json not found in trajopt dir")

        val pickupFirstRow  = TrajoptLoader.loadTrajectory(projectFile, "PickupFirstRow")!!
        val descoreGate     = TrajoptLoader.loadTrajectory(projectFile, "DescoreGate")!!
        val shootFirstRow   = TrajoptLoader.loadTrajectory(projectFile, "ShootFirstRow")!!
        val shootSecondRow  = TrajoptLoader.loadTrajectory(projectFile, "ShootSecondRow")!!
        val gateShoot1      = TrajoptLoader.loadTrajectory(projectFile, "GateShoot1")!!
        val gateShoot2      = TrajoptLoader.loadTrajectory(projectFile, "GateShoot2")!!

        val initialSample = pickupFirstRow.getInitialSample()
            ?: throw IllegalStateException("PickupFirstRow has no samples")

        io.setPosition(initialSample.pos)

        val robot = Robot(io, blue = false, useNativeAim = true)
        robot.init(initialSample.pos, apriltagTracking = false)
        robot.aimTurret = true
        robot.aimFlywheel = true
        robot.intakeCoordinator.autoShootEnabled = false

        val state = State(io)

        waitForStart()

        val trajectories = listOf(
            pickupFirstRow,
            descoreGate,
            shootFirstRow,
            shootSecondRow,
            gateShoot1,
            gateShoot2,
        )

        for (traj in trajectories) {
            if (!opModeIsActive()) break
            runTrajectory(robot, state, traj)
        }

        io.driveFL = 0.0
        io.driveBL = 0.0
        io.driveBR = 0.0
        io.driveFR = 0.0
        robot.shooter.flywheelTarget = 0.0
        io.update()
    }

    private fun runTrajectory(robot: Robot, state: State, traj: TrajoptTrajectory) {
        robot.ltv.loadTrajectory(traj)
        val startTime = io.time()
        val firedEvents = BooleanArray(traj.eventMarkers.size)

        while (opModeIsActive()) {
            state.update(io)
            val elapsed = io.time() - startTime
            val elapsedSeconds = elapsed.toDouble(DurationUnit.SECONDS)

            // Drive
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

            // Maintain idle flywheel speed between prespin/burst windows so NativeAutoAim
            // has a warm flywheel to spin up from when approaching a shooting zone.
            // NativeAutoAim overrides this with targetOmega when prespin or burst is active.
            robot.shooter.flywheelTarget = 240.0

            // Fire event markers by timestamp
            traj.eventMarkers.forEachIndexed { idx, event ->
                if (!firedEvents[idx] && elapsedSeconds >= event.timestamp) {
                    firedEvents[idx] = true
                    when (event.name) {
                        "shoot" -> robot.aim.shotRequested = true
                        "intake_on", "intake_in" -> robot.intakeCoordinator.startIntake()
                        "intake_off" -> robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                    }
                }
            }

            // Clear shot request once all balls have exited
            if (robot.aim.shotRequested && robot.beamBreak.ballCount == 0) {
                robot.aim.shotRequested = false
            }

            robot.update()
            io.update()

            if (SIM) sleep(SIM_UPDATE_TIME.inWholeMilliseconds)

            telemetry.addData("traj", traj.name)
            telemetry.addData("elapsed", "%.2f / %.2f s".format(elapsedSeconds, traj.totalTime))
            telemetry.addData("balls", robot.beamBreak.ballCount)
            telemetry.addData("state", robot.intakeTransfer.state)
            telemetry.addData("shotReq", robot.aim.shotRequested)
            telemetry.addData("readyToShoot", robot.aim.readyToShoot)
            telemetry.update()

            if (elapsedSeconds > traj.totalTime + 1.5) break
        }
    }
}