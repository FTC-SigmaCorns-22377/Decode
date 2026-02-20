package sigmacorns.io

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevColorSensorV3
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
import sigmacorns.math.toPose2d
import sigmacorns.math.toFtcPose2d
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
    private val flywheelMotor: DcMotorEx? = hardwareMap.tryGet(DcMotorEx::class.java,"shooter")
    //intake
    private val intakeMotor: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"intakeMotor")
    //turret
    private val turretMotor: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"turret")
    //spindexer
    private val spindexerMotor: DcMotorEx? = hardwareMap.tryGet(DcMotorEx::class.java,"spindexer")
    //breakServo
    private val breakServo: Servo? = hardwareMap.tryGet(Servo::class.java,"break")
    private val transferServo: Servo? = hardwareMap.tryGet(Servo::class.java,"transfer")
    private val tilt1Servo: Servo? = hardwareMap.tryGet(Servo::class.java,"tilt1")
    private val tilt2Servo: Servo? = hardwareMap.tryGet(Servo::class.java,"tilt2")

    //sensors
    val colorSensor: RevColorSensorV3? = hardwareMap.tryGet(RevColorSensorV3::class.java, "color")
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
    override var tilt1: Double = 0.0
    override var tilt2: Double = 0.0

    private var savedVoltage: Double = 12.0

    var turretOffset = 0

    // Cached motor values
    private var cachedFlywheelVelocity: Double = 0.0
    private var cachedTurretPosition: Double = 0.0
    private var cachedSpindexerPosition: Double = 0.0

    // Last sent values for threshold checking
    private var lastDriveFL: Double = Double.NaN
    private var lastDriveFR: Double = Double.NaN
    private var lastDriveBL: Double = Double.NaN
    private var lastDriveBR: Double = Double.NaN
    private var lastShooter: Double = Double.NaN
    private var lastIntake: Double = Double.NaN
    private var lastTurret: Double = Double.NaN
    private var lastSpindexer: Double = Double.NaN
    private var lastBreakPower: Double = Double.NaN
    private var lastTransfer: Double = Double.NaN
    private var lastTilt1: Double = Double.NaN
    private var lastTilt2: Double = Double.NaN

    companion object {
        const val UPDATE_THRESHOLD = 0.001
    }

    private fun shouldUpdate(current: Double, last: Double): Boolean {
        return last.isNaN() || kotlin.math.abs(current - last) >= UPDATE_THRESHOLD
    }


    override fun position(): Pose2d {
        val sensorPose = pinpoint?.position?.toPose2d()
        return when (sensorPose) {
            null -> posOffset
            else -> posOffset.compose(sensorPose)
        }
    }

    override fun velocity(): Pose2d {
        if (pinpoint == null ) return Pose2d()

        val vx = pinpoint!!.getVelX(DistanceUnit.METER)
        val vy = pinpoint!!.getVelY(DistanceUnit.METER)
        return Pose2d(
            cos(-posOffset.rot)*vx + sin(-posOffset.rot)*vy,
            -sin(-posOffset.rot)*vx + cos(-posOffset.rot)*vy,

            pinpoint!!.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        )
    }

    override fun flywheelVelocity(): Double {
        return cachedFlywheelVelocity
    }

    override fun turretPosition(): Double {
        return cachedTurretPosition  + turretOffset
    }

    override fun spindexerPosition(): Double {
        return cachedSpindexerPosition
    }

    override fun distance(): Double {
        return distanceSensor?.getDistance(DistanceUnit.METER) ?: 0.0
    }

    override fun setTurretPosition(Offset: Int) {
        turretOffset = -Offset //change

    }

    var posOffset = Pose2d()

    override fun setPosition(p: Pose2d) {
        val sensorPose = pinpoint?.position?.toPose2d()
        posOffset = when (sensorPose) {
            null -> p
            else -> p.compose(sensorPose.inverse())
        }
    }

    override fun update() {
        if (shouldUpdate(driveFL, lastDriveFL)) {
            driveFLMotor.power = driveFL
            lastDriveFL = driveFL
        }
        if (shouldUpdate(driveFR, lastDriveFR)) {
            driveFRMotor.power = driveFR
            lastDriveFR = driveFR
        }
        if (shouldUpdate(driveBL, lastDriveBL)) {
            driveBLMotor.power = driveBL
            lastDriveBL = driveBL
        }
        if (shouldUpdate(driveBR, lastDriveBR)) {
            driveBRMotor.power = driveBR
            lastDriveBR = driveBR
        }

        if (shouldUpdate(shooter, lastShooter)) {
            flywheelMotor?.power = shooter
            lastShooter = shooter
        }
        if (shouldUpdate(intake, lastIntake)) {
            intakeMotor?.power = intake
            lastIntake = intake
        }
        if (shouldUpdate(turret, lastTurret)) {
            turretMotor?.power = turret
            lastTurret = turret
        }
        if (shouldUpdate(spindexer, lastSpindexer)) {
            spindexerMotor?.targetPosition = spindexer.toInt()
            spindexerMotor?.power = 1.0
            lastSpindexer = spindexer
        }
        //updating the positions of all the servos (only if changed beyond threshold)
        if (shouldUpdate(breakPower, lastBreakPower)) {
            breakServo?.position = breakPower
            lastBreakPower = breakPower
        }
        if (shouldUpdate(transfer, lastTransfer)) {
            transferServo?.position = transfer
            lastTransfer = transfer
        }
        if (shouldUpdate(tilt1, lastTilt1)) {
            tilt1Servo?.position = tilt1
            lastTilt1 = tilt1
        }
        if (shouldUpdate(tilt2, lastTilt2)) {
            tilt2Servo?.position = tilt2
            lastTilt2 = tilt2
        }

        pinpoint?.update()
        pollColorDist()
        savedVoltage = voltageSensor?.voltage ?: 12.0
        cachedFlywheelVelocity = (flywheelMotor?.getVelocity(AngleUnit.RADIANS) ?: 0.0) * 28.0 * 2 * PI
        cachedTurretPosition = turretMotor?.currentPosition?.toDouble() ?: 0.0
        cachedSpindexerPosition = spindexerMotor?.currentPosition?.toDouble() ?: 0.0
        allHubs.map { it.clearBulkCache() }
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
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )

        //resetting the positions for the IMU
        pinpoint?.resetPosAndIMU()
        Thread.sleep(300)
        pinpoint?.update()
    }

    override fun voltage(): Double = savedVoltage

    private var cachedDistance: Double = 0.0
    private var cachedR: Int = 0
    private var cachedG: Int = 0
    private var cachedB: Int = 0
    private var cachedA: Int = 0

    private val rawRField = RevColorSensorV3::class.java.getDeclaredField("red").also {
        it.isAccessible = true
    }

    private val rawBField = RevColorSensorV3::class.java.getDeclaredField("blue").also {
        it.isAccessible = true
    }

    private val rawGField = RevColorSensorV3::class.java.getDeclaredField("green").also {
        it.isAccessible = true
    }

    private val rawAField = RevColorSensorV3::class.java.getDeclaredField("alpha").also {
        it.isAccessible = true
    }

    private fun pollColorDist() {
        cachedDistance = (colorSensor as? DistanceSensor)?.getDistance(DistanceUnit.CM) ?: 100.0

        // just to call internal updateColors()
        colorSensor?.normalizedColors

        colorSensor?.let {
            cachedR = rawRField.get(it) as Int
            cachedG = rawGField.get(it) as Int
            cachedB = rawBField.get(it) as Int
            cachedA = rawAField.get(it) as Int
        }
    }
    override fun colorSensorDetectsBall(): Boolean {
        if (colorSensor == null) return false
        return cachedDistance < 5.0
    }

    override fun colorSensorGetBallColor(): Balls? {
        if (colorSensor == null) return null
        if (!colorSensorDetectsBall()) return null

        // Read RGB values from the sensor
        val red = cachedR
        val green = cachedG
        val blue = cachedB

        // Determine which color this is based on calibration
        // These thresholds will need to be tuned for your specific balls and lighting

        // Example logic (tune these values for your robot):
        if (cachedA < 95) return null

        if (green > red && green > blue) {
            // Green ball detected
            return Balls.Green
        } else if ((red + blue) > green * 1.5) {
            // Purple ball detected (red + blue makes purple)
            return Balls.Purple
        }

        // If we can't determine the color clearly, return null
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
        spindexerMotor?.targetPosition = 0
        spindexerMotor?.setPositionPIDFCoefficients(25.0)
        spindexerMotor?.setVelocityPIDFCoefficients(13.0,0.0,2.0,0.025)
        spindexerMotor?.mode = DcMotor.RunMode.RUN_TO_POSITION

        transferServo?.direction = Servo.Direction.REVERSE
        tilt1Servo?.direction = Servo.Direction.REVERSE
        tilt2Servo?.direction = Servo.Direction.FORWARD

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
