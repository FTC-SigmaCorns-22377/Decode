package sigmacorns.io

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.lynx.LynxModule
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
    private val flywheelMotor: DcMotorEx? = hardwareMap.tryGet(DcMotorEx::class.java,"shooter")
    //intake
    private val intakeMotor: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"intakeMotor")
    //turret
    private val turretMotor: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"turret")

    // turret servos (dual-servo geared turret)
    private val turretLeftServo: Servo? = hardwareMap.tryGet(Servo::class.java, "turretLeft")
    private val turretRightServo: Servo? = hardwareMap.tryGet(Servo::class.java, "turretRight")

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
    var pinpoint: GoBildaPinpointDriver? = hardwareMap.tryGet(GoBildaPinpointDriver::class.java,"pinpoint")

    val voltageSensor = hardwareMap.voltageSensor.iterator().let { if(it.hasNext()) it.next() else null }

    private val allHubs: MutableList<LynxModule> = hardwareMap.getAll(LynxModule::class.java)

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

    var turretOffset = 0.0

    // Cached motor values
    private var cachedFlywheelVelocity: Double = 0.0
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

    override fun turretPosition(): Double {
        return cachedTurretPosition + turretOffset
    }

    override fun setTurretPosition(Offset: Double) {
        turretOffset = Offset //change
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
            flywheelMotor?.power = flywheel
            lastFlywheel = flywheel
        }
        if (shouldUpdate(intake, lastIntake)) {
            intakeMotor?.power = intake
            lastIntake = intake
        }
        if (shouldUpdate(turret, lastTurret)) {
            turretMotor?.power = turret
            lastTurret = turret
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

        pinpoint?.update()
        savedVoltage = voltageSensor?.voltage ?: 12.0
        cachedFlywheelVelocity = (flywheelMotor?.getVelocity(AngleUnit.RADIANS) ?: 0.0) * 28.0 * 2 * PI
        cachedTurretPosition = turretMotor?.currentPosition?.toDouble() ?: 0.0
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
        flywheelMotor?.direction = DcMotorSimple.Direction.FORWARD
        turretMotor?.direction = DcMotorSimple.Direction.FORWARD
        intakeMotor?.direction = DcMotorSimple.Direction.FORWARD

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

        //declaring the driveMode's for auxilery motors(which will be run without encoder for now)
        flywheelMotor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        turretMotor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

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
