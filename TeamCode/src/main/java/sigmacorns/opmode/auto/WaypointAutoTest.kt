package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.launch
import sigmacorns.Robot
import sigmacorns.control.trajopt.AutoAuto
import sigmacorns.control.trajopt.AutoPOI
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode

@Autonomous
class WaypointAutoTest: SigmaOpMode() {
    override fun runOpMode() {
        val sequence = listOf(
            AutoPOI.PRELOAD,
            AutoPOI.SPIKE_FAR,
            AutoPOI.LAUNCH_FAR,
            AutoPOI.SPIKE_MED,
            AutoPOI.LAUNCH_FAR
        )

        val initialPos = Pose2d(0.30, -1.524, 0.0)

        val auto = AutoAuto(
            false,
            initialPos,
            0.0,
            0.90,
            { 1.5 },
            sequence
        )

        val robot = Robot(io, blue = true)
        robot.init(initialPos, apriltagTracking = !SIM)
        robot.aimTurret = true
        robot.aimFlywheel = true

        waitForStart()

        val f = robot.scope.launch {
            auto.waypoints.map {
                auto.run(robot,it)
            }
        }

        ioLoop { x, dt ->
            robot.update()

            f.isCompleted
        }
    }
}