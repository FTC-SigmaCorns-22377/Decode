package sigmacorns.opmode.auto

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import sigmacorns.FieldState
import sigmacorns.opmode.SigmaOpMode


@Autonomous
class MoveForward: SigmaOpMode() {

    override fun runOpMode() {
        val limelight = hardwareMap.get<Limelight3A?>(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(10) // This sets how often we ask Limelight for data (100 times per second)
        limelight.start() // This tells Limelight to start looking!

        val fieldState = FieldState()

        var motifDetected : Boolean = false

        waitForStart()
//        io.driveFL = 1.0
//        io.driveBL = 1.0
//        io.driveBR = 1.0
//        io.driveFR = 1.0
//        io.update()
//
//        sleep(500)

        io.driveFL = 0.0
        io.driveBL = 0.0
        io.driveBR = 0.0
        io.driveFR = 0.0
        io.update()

        while(opModeIsActive())
        {
            if (!motifDetected)
            {
                val result = limelight.getLatestResult()
                if (result != null && result.isValid()) {
                    val fiducials: MutableList<FiducialResult> = result.getFiducialResults()
                    for (fiducial in fiducials) {
                        val id = fiducial.getFiducialId() // The ID number of the fiducial
                        if ((id == 21) ||
                            (id == 22) ||
                            (id == 23))
                        {
                            motifDetected = true
                            fieldState.motif = id
                        }
                        telemetry.addData("Motif ID", fieldState.motif )
                    }
                }
            }

        }
    }
}