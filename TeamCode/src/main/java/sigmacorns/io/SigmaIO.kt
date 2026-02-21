package sigmacorns.io

import sigmacorns.math.Pose2d
import sigmacorns.sim.Balls
import kotlin.time.Duration

interface SigmaIO {

    // drive motors
    var driveFL: Double
    var driveBL: Double
    var driveFR: Double
    var driveBR: Double

    //shooter power
    var shooter: Double
    //intake
    var intake: Double // might just be human fed and we wont end up needing this
    //turret power
    var turret: Double // The motor that rotates the turrent to keep it lined up with the goal
    //turretservo value
    var turretAngle: Double // the value that controls the angle of the turret is shooting at
    //spindexer target position (ticks) — motor uses RUN_TO_POSITION mode
    var spindexer: Double
    //break servo
    var breakPower: Double //the value that contols the position of the servo to engage or disengage the servo
    var transfer: Double
    //tilt servos for brake
    var tilt1: Double
    var tilt2: Double
    /* function to get robo position
    position( drive base)
    velocity(drive base)
     */



    fun position(): Pose2d
    fun velocity(): Pose2d
    fun flywheelVelocity(): Double
    fun turretPosition(): Double
    fun spindexerPosition(): Double
    fun distance(): Double

    fun setTurretPosition(Offset: Int)
    fun update()

    fun setPosition(p: Pose2d)

    /**
     * @return time passed since the start of the opmode
     */
    fun time(): Duration
    fun configurePinpoint()

    fun voltage(): Double

    // Color sensor for Decode-14b: Automatic ball detection
    fun colorSensorDetectsBall(): Boolean
    fun colorSensorGetBallColor(): Balls?
}
