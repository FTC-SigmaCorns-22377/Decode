package sigmacorns.opmode.auto

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import sigmacorns.FieldState
import sigmacorns.globalFieldState
import sigmacorns.opmode.SigmaOpMode



fun aaa() {
    val list = listOf<Double?>(null,null,null)

    if(list.all { it == null}) {}

    if(list[0] == null && list[1] == null && list[2] == null) {}
    fun f(item: Double?): Boolean { return item == null }
    val f = { item: Double? -> item == null }
    if(list.all(f)) {}

    if(list.all({ item -> item == null })) {}
    if(list.all { item -> item == null } ) {}
    if(list.all { it == null } ) {}
}


@Autonomous
class MoveForward: SigmaOpMode() {

    override fun runOpMode() {
        val limelight = hardwareMap.get<Limelight3A?>(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(10) // This sets how often we ask Limelight for data (100 times per second)
        limelight.start() // This tells Limelight to start looking!

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
                            globalFieldState.motif = id
                        }
                        telemetry.addData("Motif ID", globalFieldState.motif)
                        telemetry.update()
                    }
                }
            }

        }
    }
}