package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import sigmacorns.opmode.SigmaOpMode

@Autonomous
class MoveForward: SigmaOpMode() {
    override fun runOpMode() {
        waitForStart()
        io.driveFL = 1.0
        io.driveBL = 1.0
        io.driveBR = 1.0
        io.driveFR = 1.0
        io.update()

        sleep(500)

        io.driveFL = 0.0
        io.driveBL = 0.0
        io.driveBR = 0.0
        io.driveFR = 0.0
        io.update()
    }
}