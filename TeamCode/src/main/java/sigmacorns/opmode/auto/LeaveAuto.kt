package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import sigmacorns.opmode.PosePersistence
import sigmacorns.opmode.SigmaOpMode
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "Leave Auto", group = "Competition")
class LeaveAuto : SigmaOpMode() {
    override fun runOpMode() {
        waitForStart()

        val start = io.time()
        ioLoop { _, _ ->
            io.driveFL = 0.5
            io.driveBL = 0.5
            io.driveBR = 0.5
            io.driveFR = 0.5
            io.time() - start >= 1.seconds
        }

        io.driveFL = 0.0
        io.driveBL = 0.0
        io.driveBR = 0.0
        io.driveFR = 0.0
        PosePersistence.save(storageDir(), io.position())
        io.update()
    }
}
