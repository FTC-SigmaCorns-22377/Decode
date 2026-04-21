package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.Limelight
import sigmacorns.io.HardwareIO
import sigmacorns.logic.BallChaseAutoFSM
import sigmacorns.logic.BallTrackingSystem
import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.TrackerConfig
import kotlin.time.DurationUnit

/**
 * Ball-chase opmode with an auto-cycle state machine.
 *
 * Right bumper HELD = auto-chase ON: tracker picks the nearest confirmed ball,
 * the robot drives straight at it with the intake running. When the 3 beam
 * breaks report a full hopper, the robot drives to a shooting zone near the
 * goal, spins up, disengages the blocker and fires. When the hopper empties,
 * it goes back to chasing. Release right bumper at any time to reclaim
 * manual drive immediately — the FSM stops commanding subsystems and zeros
 * intake / flywheel / blocker outputs.
 *
 * Requires Limelight pipeline [Limelight.BALL_PIPELINE] to be configured as
 * a color detector tuned for the DECODE artifact. Camera intrinsics +
 * extrinsics are loaded from config/ball_tracker.json — calibrate + verify
 * before trusting the position estimates.
 */
@TeleOp(name = "Ball Chase TeleOp", group = "Chase")
class BallChaseTeleOp : SigmaOpMode() {

    override fun runOpMode() {
        val robot = Robot(io, blue = false, useNativeAim = false)
        robotRef = robot
        robot.init(Pose2d(), apriltagTracking = false)

        (io as? HardwareIO)?.limelight?.pipelineSwitch(Limelight.BALL_PIPELINE)

        val config = TrackerConfig.loadDefault()
        val tracking = BallTrackingSystem(config = config, io = io)

        // Shooting zone: 1.0 m out, 0.6 m to the side of the red goal.
        val goal = FieldLandmarks.redGoalPosition
        val zoneX = goal.x - 1.0
        val zoneY = goal.y - 0.6
        val shootingZone = Pose2d(zoneX, zoneY, kotlin.math.atan2(goal.y - zoneY, goal.x - zoneX))

        val fsm = BallChaseAutoFSM(
            tracking = tracking,
            drivetrain = robot.drive,
            io = io,
            shootingZone = shootingZone,
            maxSpeed = 1.0,
            arrivalRadiusM = 0.18,
            chasePullInRadiusM = 0.35,
            flywheelShootSpeed = 1.0,
        )

        telemetry.addLine("Ball Chase TeleOp ready. Hold RB for auto-cycle; release for manual.")
        telemetry.update()

        waitForStart()

        ioLoop { state, _ ->
            val t = state.timestamp.toDouble(DurationUnit.SECONDS)

            tracking.update(t)

            val wantAuto = gamepad1.right_bumper
            fsm.enabled = wantAuto
            if (wantAuto) {
                fsm.update()
            } else {
                robot.drive.update(gamepad1, io)
            }

            telemetry.addData("mode", if (wantAuto) fsm.phase else "MANUAL")
            telemetry.addData("tracks", tracking.tracker.tracks.size)
            val tgt = tracking.target
            if (tgt != null) {
                val p = tgt.positionAt(t)
                val pose = io.position()
                val d = kotlin.math.hypot(p.x - pose.v.x, p.y - pose.v.y)
                telemetry.addData("target", "#${tgt.id} (hits=${tgt.hits})  d=%.2fm".format(d))
            } else {
                telemetry.addLine("no target")
            }
            telemetry.addData("held", BallChaseAutoFSM.countHeldFromBeamBreaks(io))
            telemetry.update()

            false
        }

        (io as? HardwareIO)?.limelight?.pipelineSwitch(Limelight.APRILTAG_PIPELINE)
    }
}
