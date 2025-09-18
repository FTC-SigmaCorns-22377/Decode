package sigmacorns.io

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap

// import odometry from some library
class HardwareIO(hardwareMap: HardwareMap) {
    //drive motor declarations
    private val driveFL: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"driveFL")
    private val driveBL: DcMotor? = hardwareMap.tryGet(DcMotor::class.java, "driveBL")
    private val driveFR: DcMotor? = hardwareMap.tryGet(DcMotor::class.java, "driveFR")
    private val driveBR: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"driveBR")

    //shooter
    private val flyWheel: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"flyWheel")
    private val guidingArm: CRServo? = hardwareMap.tryGet(CRServo::class.java,"guidingArm")

    //intake
    private val intake:DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"intake")

    //sensors
    private val colorSensor: ColorRangeSensor? = hardwareMap.tryGet(ColorRangeSensor::class.java, "color")
    val limelight: Limelight3A? = hardwareMap.tryGet(Limelight3A::class.java, "limeLight")
    val imu: IMU? = hardwareMap.tryGet(IMU::class.java,"imu")
    
}

