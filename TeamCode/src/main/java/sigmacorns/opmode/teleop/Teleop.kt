package sigmacorns.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import sigmacorns.io.SigmaIO
import sigmacorns.opmode.SigmaOpMode

class Teleop(io: SigmaIO): SigmaOpMode(io) {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val FL = hardwareMap.get<DcMotor>(DcMotor::class.java, "FL")
        val FR = hardwareMap.get<DcMotor>(DcMotor::class.java, "FR")
        val BL = hardwareMap.get<DcMotor>(DcMotor::class.java, "BL")
        val BR = hardwareMap.get<DcMotor>(DcMotor::class.java, "BR")
        val flywheel = hardwareMap.get<DcMotor>(DcMotor::class.java, "FlyWheel")

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

            val isAPressed = gamepad1.a
            val isBPressed = gamepad1.b
            if (isAPressed)
                flywheel.setPower(0.5)
            else if (isBPressed)
                flywheel.setPower(0.25)
            else
                flywheel.setPower(0.0)


        }
    }
}