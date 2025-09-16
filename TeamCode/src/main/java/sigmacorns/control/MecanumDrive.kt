package sigmacorns.control

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

class MecanumDrive : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val FL = hardwareMap.get<DcMotor>(DcMotor::class.java, "FL")
        val FR = hardwareMap.get<DcMotor>(DcMotor::class.java, "FR")
        val BL = hardwareMap.get<DcMotor>(DcMotor::class.java, "BL")
        val BR = hardwareMap.get<DcMotor>(DcMotor::class.java, "BR")

        FL.setDirection(DcMotorSimple.Direction.REVERSE)

        while (opModeIsActive()) {
            val vx = -gamepad1.left_stick_y.toDouble()
            val vy = gamepad1.right_stick_x.toDouble()
            val vw = gamepad1.right_stick_x.toDouble()

            val vFL = vx + vy - vw
            val vFR = vx - vy + vw
            val vBL = vx - vy - vw
            val vBR = vx + vy - vw

            FL.setPower(vFL)
            FR.setPower(vFR)
            BL.setPower(vBL)
            BR.setPower(vBR)
        }
    }
}

