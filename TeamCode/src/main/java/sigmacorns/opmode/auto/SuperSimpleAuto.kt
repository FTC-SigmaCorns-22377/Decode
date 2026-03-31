package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import sigmacorns.State
import sigmacorns.subsystem.Drivetrain
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

@Autonomous
class SuperSimpleAuto: SigmaOpMode() {
    override fun runOpMode() {
        val drivetrain = Drivetrain()
        waitForStart()

        val startDriveTime = io.time()
        ioLoop { state: State, dt: Duration ->
            drivetrain.drive(Pose2d(0.5, 0.0, 0.0),io)

            val res = io.time() - startDriveTime > 0.5.seconds

            if(res) {
                io.driveBR = 0.0
                io.driveBL = 0.0
                io.driveFR = 0.0
                io.driveFL = 0.0
            }
            res
        }
    }
}