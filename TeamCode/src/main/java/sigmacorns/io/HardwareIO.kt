package sigmacorns.io

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

// import odometry from some library
class HardwareIO(hardwareMap: HardwareMap): SigmaIO {
    //drive motor declarations
    private val driveFLMotor: DcMotor = hardwareMap.tryGet(DcMotor::class.java,"driveFL") as DcMotor
    private val driveBLMotor: DcMotor = hardwareMap.tryGet(DcMotor::class.java, "driveBL") as DcMotor
    private val driveFRMotor: DcMotor = hardwareMap.tryGet(DcMotor::class.java, "driveFR") as DcMotor
    private val driveBRMotor: DcMotor = hardwareMap.tryGet(DcMotor::class.java,"driveBR") as DcMotor

    //shooter
    private val flyWheelMotor0: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"flyWheel0")
    private val flyWheelMotor1: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"flyWheel1")
    //intake
    private val intakeMotor: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"intake")

    //sensors
    private val colorSensor: ColorRangeSensor? = hardwareMap.tryGet(ColorRangeSensor::class.java, "color")
    val limelight: Limelight3A? = hardwareMap.tryGet(Limelight3A::class.java, "limeLight")
    val imu: IMU? = hardwareMap.tryGet(IMU::class.java,"imu")

    override var driveFL: Double = 0.0
    override var driveBL: Double = 0.0
    override var driveFR: Double = 0.0
    override var driveBR: Double = 0.0
    override var flyWheel0: Double = 0.0
    override var flyWheel1: Double = 0.0
    override var intake: Double = 0.0

    override fun position(): Array<Double> {
        TODO("Not yet implemented")
    }

    override fun velocity(): Array<Double> {
        TODO("Not yet implemented")
    }

    override fun flywheelVelocity(): Array<Double> {
        TODO("Not yet implemented")
    }

    override fun update() {
        //updating power values of driveMotors
        driveFLMotor.power = driveFL
        driveFRMotor.power = driveFR
        driveBLMotor.power = driveBL
        driveBRMotor.power = driveBR

        //updating power values of auxilery motors
        flyWheelMotor0?.power = flyWheel0
        flyWheelMotor1?.power = flyWheel1
        intakeMotor?.power = intake
    }

    init {
        //drive motor direction declarations
        driveFLMotor.direction = DcMotorSimple.Direction.FORWARD
        driveFRMotor.direction = DcMotorSimple.Direction.REVERSE
        driveBLMotor.direction = DcMotorSimple.Direction.REVERSE
        driveBRMotor.direction = DcMotorSimple.Direction.FORWARD

        //flywheel and intake motors(auxilery) direction declarations
        flyWheelMotor0?.direction = DcMotorSimple.Direction.FORWARD
        flyWheelMotor1?.direction = DcMotorSimple.Direction.REVERSE
        intakeMotor?.direction = DcMotorSimple.Direction.FORWARD

        //declaring driveMode's for drive motors( which will be run without encoder for now)
        val driveMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        driveFLMotor.mode = driveMode
        driveFRMotor.mode = driveMode
        driveBLMotor.mode = driveMode
        driveBRMotor.mode = driveMode

        //stoping and resetting the encoders for the auxilery motors( stop and reset)
        flyWheelMotor0?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        flyWheelMotor1?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intakeMotor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        //declaring the driveMode's for auxilery motors(which will be run without encoder for now)
        flyWheelMotor0?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flyWheelMotor1?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        //do pinpoint odometry stuff here once thomas imports the lib







    }



}


