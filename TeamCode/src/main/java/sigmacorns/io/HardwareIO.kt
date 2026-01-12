package sigmacorns.io

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.joml.Vector2d
import sigmacorns.math.Pose2d
import sigmacorns.sim.Balls
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.time.ComparableTimeMark
import kotlin.time.Duration
import kotlin.time.TimeSource


typealias FTCPose2d = org.firstinspires.ftc.robotcore.external.navigation.Pose2D

// import odometry from some library
class HardwareIO(hardwareMap: HardwareMap): SigmaIO {
    //drive motor declarations
    private val driveFLMotor: DcMotor = hardwareMap.get(DcMotor::class.java,"driveFL")
    private val driveBLMotor: DcMotor = hardwareMap.get(DcMotor::class.java, "driveBL")
    private val driveFRMotor: DcMotor = hardwareMap.get(DcMotor::class.java, "driveFR")
    private val driveBRMotor: DcMotor = hardwareMap.get(DcMotor::class.java,"driveBR")

    //shooter
    private val flywheelMotor: DcMotorEx? = hardwareMap.tryGet(DcMotorEx::class.java,"shooter")!!
    //intake
    private val intakeMotor: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"intakeMotor")
//turret
    private val turretMotor: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"turret")
    //spindexer
    private val spindexerMotor: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"spindexer")
    //breakServo
    private val breakServo: Servo? = hardwareMap.tryGet(Servo::class.java,"break")
    private val transferServo: CRServo? = hardwareMap.tryGet(CRServo::class.java,"transfer")

    //sensors
    private val colorSensor: ColorRangeSensor? = hardwareMap.tryGet(ColorRangeSensor::class.java, "color")
    private val distanceSensor: DistanceSensor? = hardwareMap.tryGet(DistanceSensor::class.java, "dist")
    val limelight: Limelight3A? = hardwareMap.tryGet(Limelight3A::class.java, "limelight")
    val imu: IMU? = hardwareMap.tryGet(IMU::class.java,"imu")

    //odometry
    var pinpoint: GoBildaPinpointDriver? = hardwareMap.tryGet(GoBildaPinpointDriver::class.java,"pinpoint")

    val voltageSensor = hardwareMap.voltageSensor.iterator().let { if(it.hasNext()) it.next() else null }

    private val allHubs: MutableList<LynxModule> = hardwareMap.getAll(LynxModule::class.java)

    override var driveFL: Double = 0.0
    override var driveBL: Double = 0.0
    override var driveFR: Double = 0.0
    override var driveBR: Double = 0.0
    override var shooter: Double = 0.0
    override var intake: Double = 0.0
    override var turret: Double = 0.0
    override var turretAngle: Double = 0.0
    override var spindexer: Double = 0.0
    override var breakPower: Double = 0.0
    override var transfer: Double = 0.0

    private var savedVoltage: Double = 12.0

    // Cached motor values
    private var cachedFlywheelVelocity: Double = 0.0
    private var cachedTurretPosition: Double = 0.0
    private var cachedSpindexerPosition: Double = 0.0

    private fun FTCPose2d.toPose2d(): Pose2d = Pose2d(
            getX(DistanceUnit.METER),
            getY(DistanceUnit.METER),
            getHeading(AngleUnit.RADIANS)
    )

    private fun Pose2d.toFtcPose2d(): FTCPose2d = FTCPose2d(
        DistanceUnit.METER,
        v.x,
        v.y,
        AngleUnit.RADIANS,
        rot
    )

    override fun position(): Pose2d {
        val sensorPose = pinpoint?.position?.toPose2d()
        return when (sensorPose) {
            null -> posOffset
            else -> posOffset.compose(sensorPose)
        }
    }

    override fun velocity(): Pose2d {
        if (pinpoint == null ) return Pose2d()
        return Pose2d(
            pinpoint!!.getVelX(DistanceUnit.METER),
            pinpoint!!.getVelY(DistanceUnit.METER),
            pinpoint!!.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        )
    }

    override fun flywheelVelocity(): Double {
        return cachedFlywheelVelocity
    }

    override fun turretPosition(): Double {
        return cachedTurretPosition
    }

    override fun spindexerPosition(): Double {
        return cachedSpindexerPosition
    }

    override fun distance(): Double {
        return distanceSensor?.getDistance(DistanceUnit.METER) ?: 0.0
    }

    var posOffset = Pose2d()

    override fun setPosition(p: Pose2d) {
        val sensorPose = pinpoint?.position?.toPose2d()
        posOffset = when (sensorPose) {
            null -> p
            else -> p
        }
    }

    override fun update() {
        //updating power values of driveMotors
        driveFLMotor.power = driveFL
        driveFRMotor.power = driveFR
        driveBLMotor.power = driveBL
        driveBRMotor.power = driveBR


        //updating power values of auxilery motors
        flywheelMotor?.power = shooter
        intakeMotor?.power = intake
        turretMotor?.power = turret
        spindexerMotor?.power = spindexer
        //updating the positions of all the servos
        breakServo?.position = breakPower
        transferServo?.power = transfer

        pinpoint?.update()
        savedVoltage = voltageSensor?.voltage ?: 12.0

        // Cache motor positions and velocities

        allHubs.map { it.clearBulkCache() }
        cachedFlywheelVelocity = (flywheelMotor?.getVelocity(AngleUnit.RADIANS) ?: 0.0) * 28.0 * 2 * PI
        cachedTurretPosition = turretMotor?.currentPosition?.toDouble() ?: 0.0
        cachedSpindexerPosition = spindexerMotor?.currentPosition?.toDouble() ?: 0.0
    }

    val startTime: ComparableTimeMark = TimeSource.Monotonic.markNow()
    override fun time(): Duration {
        val duration = TimeSource.Monotonic.markNow() - startTime

        return duration
    }

    override fun configurePinpoint() {
        //setting encoder resolution
        pinpoint?.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

        pinpoint?.setOffsets(
            -92.00000,
            137.74906,
            DistanceUnit.MM
        )

        //setting the directions of the ododmetry pods
        pinpoint?.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        )

        //resetting the positions for the IMU
        pinpoint?.resetPosAndIMU()
    }

    override fun voltage(): Double = savedVoltage
    override fun colorSensorDetectsBall(): Boolean {
        return false
    }

    override fun colorSensorGetBallColor(): Balls? {
        return null
    }

    fun configureLimelight() {
        limelight?.pipelineSwitch(0);
        limelight?.start();
    }

    init {
        //drive motor direction declarations
        driveFLMotor.direction = DcMotorSimple.Direction.REVERSE
        driveFRMotor.direction = DcMotorSimple.Direction.FORWARD
        driveBLMotor.direction = DcMotorSimple.Direction.REVERSE
        driveBRMotor.direction = DcMotorSimple.Direction.FORWARD

        //flywheel and intake motors(auxilery) direction declarations
        flywheelMotor?.direction = DcMotorSimple.Direction.FORWARD
        turretMotor?.direction = DcMotorSimple.Direction.FORWARD
        intakeMotor?.direction = DcMotorSimple.Direction.FORWARD
        spindexerMotor?.direction = DcMotorSimple.Direction.REVERSE

        //declaring driveMode's for drive motors( which will be run without encoder for now)
        val driveMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        driveFLMotor.mode = driveMode
        driveFRMotor.mode = driveMode
        driveBLMotor.mode = driveMode
        driveBRMotor.mode = driveMode

        //stoping and resetting the encoders for the auxilery motors( stop and reset)
        flywheelMotor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        turretMotor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intakeMotor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        spindexerMotor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        //declaring the driveMode's for auxilery motors(which will be run without encoder for now)
        flywheelMotor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        turretMotor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        spindexerMotor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        transferServo?.direction = DcMotorSimple.Direction.REVERSE

        // configuring pinpoint
        configurePinpoint()

        for (hub in allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        }
    }

}

private fun Pose2d.compose(other: Pose2d): Pose2d {
    val rotated = other.v.rotate(rot)
    rotated.add(v)
    return Pose2d(rotated, rot + other.rot)
}

private fun Pose2d.inverse(): Pose2d {
    val invRot = -rot
    val invTranslation = Vector2d(v).negate().rotate(invRot)
    return Pose2d(invTranslation, invRot)
}

fun Vector2d.rotate(theta: Double) = Vector2d(
    cos(theta)*x - sin(theta)*y,
    sin(theta)*x + cos(theta)*y,
)
