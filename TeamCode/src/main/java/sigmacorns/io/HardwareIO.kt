package sigmacorns.io

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.joml.Vector2d
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.TurretServoConfig
import sigmacorns.math.toPose2d
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
    private val flywheel1: DcMotorEx? = hardwareMap.tryGet(DcMotorEx::class.java,"shooter1")
    private val flywheel2: DcMotorEx? = hardwareMap.tryGet(DcMotorEx::class.java,"shooter2")
    //intake
    private val intake1Motor: DcMotorEx? = hardwareMap.tryGet(DcMotorEx::class.java,"intake1")
    //intake
    private val intake2Motor: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"intake2")

    // turret servos (dual-servo geared turret)
    private val turretLeftServo: Servo? = hardwareMap.tryGet(Servo::class.java, "turretLeft")
    private val turretRightServo: Servo? = hardwareMap.tryGet(Servo::class.java, "turretRight")
    private val turretEncoder: AnalogInput? = hardwareMap.tryGet(AnalogInput::class.java, "turretEncoder")

    // hood servo
    private val hoodServo: Servo? = hardwareMap.tryGet(Servo::class.java, "hood")

    // blocker servo
    private val blockerServo: Servo? = hardwareMap.tryGet(Servo::class.java, "blocker")

    // beam break sensors
    private val beamBreak1Sensor: DigitalChannel? = hardwareMap.tryGet(DigitalChannel::class.java, "beamBreak1")
    private val beamBreak2Sensor: DigitalChannel? = hardwareMap.tryGet(DigitalChannel::class.java, "beamBreak2")
    private val beamBreak3Sensor: DigitalChannel? = hardwareMap.tryGet(DigitalChannel::class.java, "beamBreak3")

    //sensors
    val limelight: Limelight3A? = hardwareMap.tryGet(Limelight3A::class.java, "limelight")

    //odometry
    // TODO: thread pinpoint
    var pinpoint: GoBildaPinpointDriver? = hardwareMap.tryGet(GoBildaPinpointDriver::class.java,"pinpoint")

    private val voltageSensor = hardwareMap.voltageSensor.iterator().let { if(it.hasNext()) it.next() else null }

    private val chub: LynxModule = hardwareMap.get("Control Hub") as LynxModule

    override var driveFL: Double = 0.0
    override var driveBL: Double = 0.0
    override var driveFR: Double = 0.0
    override var driveBR: Double = 0.0


    override var flywheel: Double = 0.0
    override var intake: Double = 0.0
    override var turret: Double = 0.0
    override var turretLeft: Double = 0.5
    override var turretRight: Double = 0.5
    override var hood: Double = 0.5
    override var blocker: Double = 0.0

    private var savedVoltage: Double = 12.0

    // Cached motor values
    private var cachedFlywheelVelocity: Double = 0.0
    private var cachedIntake1RPM: Double = 0.0
    private var cachedTurretPosition: Double = 0.0

    // Cached beam break values (true = beam broken = ball present)
    private var cachedBeamBreak1: Boolean = false
    private var cachedBeamBreak2: Boolean = false
    private var cachedBeamBreak3: Boolean = false

    // Last sent values for threshold checking
    private var lastDriveFL: Double = Double.NaN
    private var lastDriveFR: Double = Double.NaN
    private var lastDriveBL: Double = Double.NaN
    private var lastDriveBR: Double = Double.NaN
    private var lastFlywheel: Double = Double.NaN
    private var lastIntake: Double = Double.NaN
    private var lastTurret: Double = Double.NaN
    private var lastTurretLeft: Double = Double.NaN
    private var lastTurretRight: Double = Double.NaN
    private var lastHood: Double = Double.NaN
    private var lastBlocker: Double = Double.NaN

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

    // beam broken = !state (digital channel reads false when beam is broken)
    override fun beamBreak1(): Boolean = cachedBeamBreak1
    override fun beamBreak2(): Boolean = cachedBeamBreak2
    override fun beamBreak3(): Boolean = cachedBeamBreak3

    override fun flywheelVelocity(): Double {
        return cachedFlywheelVelocity
    }

    override fun intake1RPM(): Double {
        return cachedIntake1RPM
    }

    override fun turretPosition(): Double {
        return cachedTurretPosition
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

        if (shouldUpdate(driveBR, lastDriveBR)) {
            driveBRMotor.power = driveBR
            lastDriveBR = driveBR
        }

        if (shouldUpdate(flywheel, lastFlywheel)) {
            flywheel1?.power = flywheel
            flywheel2?.power = flywheel
            lastFlywheel = flywheel
        }
        if (shouldUpdate(intake, lastIntake)) {
            intake1Motor?.power = intake
            intake2Motor?.power = intake
            lastIntake = intake
        }
        if (shouldUpdate(turretLeft, lastTurretLeft)) {
            turretLeftServo?.position = turretLeft
            lastTurretLeft = turretLeft
        }
        if (shouldUpdate(turretRight, lastTurretRight)) {
            turretRightServo?.position = turretRight
            lastTurretRight = turretRight
        }
        if (shouldUpdate(hood, lastHood)) {
            hoodServo?.position = hood
            lastHood = hood
        }
        if (shouldUpdate(blocker, lastBlocker)) {
            blockerServo?.position = blocker
            lastBlocker = blocker
        }

        // Read beam break sensors (digital channel reads false when beam is broken)
        cachedBeamBreak1 = beamBreak1Sensor?.state?.not() ?: false
        cachedBeamBreak2 = beamBreak2Sensor?.state?.not() ?: false
        cachedBeamBreak3 = beamBreak3Sensor?.state?.not() ?: false

        // update analog reading
        cachedTurretPosition = (turretEncoder?.voltage ?: 0.0) / 3.3 * TurretServoConfig.servoTotalRange - TurretServoConfig.servoCenterAngle

        pinpoint?.update()
        savedVoltage = voltageSensor?.voltage ?: 12.0
        cachedFlywheelVelocity = (flywheel1?.velocity ?: 0.0) / 28.0 * 2 * PI
        cachedIntake1RPM = (intake1Motor?.velocity ?: 0.0) / 145.1 * 60
        chub.clearBulkCache()
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
            sigmacorns.constants.PinpointConfig.X_OFFSET_MM,
            sigmacorns.constants.PinpointConfig.Y_OFFSET_MM,
            DistanceUnit.MM
        )

        //setting the directions of the ododmetry pods
        pinpoint?.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )

        //resetting the positions for the IMU
        pinpoint?.resetPosAndIMU()
        Thread.sleep(400)
        pinpoint?.update()
    }

    override fun voltage(): Double = savedVoltage

    init {
        //drive motor direction declarations
        driveFLMotor.direction = DcMotorSimple.Direction.REVERSE
        driveFRMotor.direction = DcMotorSimple.Direction.FORWARD
        driveBLMotor.direction = DcMotorSimple.Direction.REVERSE
        driveBRMotor.direction = DcMotorSimple.Direction.FORWARD

        //flywheel and intake motors(auxilery) direction declarations
        flywheel1?.direction = DcMotorSimple.Direction.FORWARD
        flywheel2?.direction = DcMotorSimple.Direction.REVERSE
        intake1Motor?.direction = DcMotorSimple.Direction.FORWARD
        intake2Motor?.direction = DcMotorSimple.Direction.REVERSE

        //declaring driveMode's for drive motors( which will be run without encoder for now)
        val driveMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        driveFLMotor.mode = driveMode
        driveFRMotor.mode = driveMode
        driveBLMotor.mode = driveMode
        driveBRMotor.mode = driveMode

        //stoping and resetting the encoders for the auxilery motors( stop and reset)
        flywheel1?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        flywheel2?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intake1Motor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intake2Motor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        //declaring the driveMode's for auxilery motors(which will be run without encoder for now)
        flywheel1?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheel2?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake1Motor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake2Motor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        flywheel1?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        flywheel2?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        intake1Motor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        intake2Motor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        // Turret servos: left = FORWARD, right = REVERSE (mirrored mounting, geared together)
        // Both servos receive the same position value; reversing the right servo
        // makes both physical shafts rotate the same direction.
        turretLeftServo?.direction = Servo.Direction.FORWARD
        turretRightServo?.direction = Servo.Direction.REVERSE

        // configure beam break sensors as inputs
        beamBreak1Sensor?.mode = DigitalChannel.Mode.INPUT
        beamBreak2Sensor?.mode = DigitalChannel.Mode.INPUT
        beamBreak3Sensor?.mode = DigitalChannel.Mode.INPUT

        // configuring pinpoint
        configurePinpoint()

        chub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
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
