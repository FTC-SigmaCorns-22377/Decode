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
import sigmacorns.math.Pose2d
import kotlin.time.ComparableTimeMark
import kotlin.time.Duration
import kotlin.time.TimeSource
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

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

    //odometry
    var pinpoint0: GoBildaPinpointDriver? = null
    var pinpoint1: GoBildaPinpointDriver? = null

    override var driveFL: Double = 0.0
    override var driveBL: Double = 0.0
    override var driveFR: Double = 0.0
    override var driveBR: Double = 0.0
    override var flyWheel0: Double = 0.0
    override var flyWheel1: Double = 0.0
    override var intake: Double = 0.0

    override fun position(): Pose2d {
        TODO("Not yet implemented")
    }

    override fun velocity(): Pose2d {
        TODO("Not yet implemented")
    }

    override fun flywheelVelocity(): Double {
        TODO("Not yet implemented")
    }

    override fun setPosition(p: Pose2d) {
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



    val startTime: ComparableTimeMark = TimeSource.Monotonic.markNow()
    override fun time(): Duration {
        val duration = TimeSource.Monotonic.markNow() - startTime

        return duration
    }

    override fun configurePinpoint() {
        TODO("Not yet implemented")
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

        //getting reference to odometry pods
        pinpoint0 = hardwareMap.get<GoBildaPinpointDriver?>(GoBildaPinpointDriver::class.java,"pinpoint0")
        pinpoint1 = hardwareMap.get<GoBildaPinpointDriver?>(GoBildaPinpointDriver::class.java,"pinpoint1")

        // configuring pinpoint
        configurePinpoint()
        //setting the position of the odo pods
        pinpoint0!!.setPosition(Pose2D(DistanceUnit.METER, 0.0, 0.0, AngleUnit.RADIANS, 0.0))
        pinpoint1!!.setPosition(Pose2D(DistanceUnit.METER, 0.0, 0.0, AngleUnit.RADIANS, 0.0))

        fun configurePinpoint() {
            //setting encoder resolution
            pinpoint0!!.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            pinpoint1!!.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)

            //setting the directions of the ododmetry pods

            pinpoint0!!.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
            )

            pinpoint1!!.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
            )

            //resetting the positions for the IMU
            pinpoint0!!.resetPosAndIMU()
            pinpoint1!!.resetPosAndIMU()
        }









    }

}


