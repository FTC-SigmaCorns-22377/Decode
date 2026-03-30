package sigmacorns.io

import sigmacorns.math.Pose2d
import kotlin.time.Duration

interface SigmaIO {

    // drive motors
    var driveFL: Double
    var driveBL: Double
    var driveFR: Double
    var driveBR: Double

    var flywheel: Double
    //intake
    var intake: Double
    //turret motor power (legacy, used by tuning opmodes)
    var turret: Double
    // turret servos (dual-servo geared turret)
    var turretLeft: Double
    var turretRight: Double
    // hood servo position (controls launch angle)
    var hood: Double
    // blocker servo position (0.0 = engaged/blocking, 1.0 = disengaged/open)
    var blocker: Double

    // beam break sensors (true = beam broken = ball present)
    fun beamBreak1(): Boolean
    fun beamBreak2(): Boolean
    fun beamBreak3(): Boolean

    fun position(): Pose2d
    fun velocity(): Pose2d
    fun flywheelVelocity(): Double
    fun intake1Velocity(): Double
    fun turretPosition(): Double
    fun setTurretPosition(offset: Double)
    fun setPosition(p: Pose2d)
    fun time(): Duration
    fun configurePinpoint()
    fun voltage(): Double
    fun update()
}
