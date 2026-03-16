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
    //turret power
    var turret: Double
    fun position(): Pose2d
    fun velocity(): Pose2d
    fun flywheelVelocity(): Double
    fun turretPosition(): Double
    fun setTurretPosition(offset: Double)
    fun setPosition(p: Pose2d)
    fun time(): Duration
    fun configurePinpoint()
    fun voltage(): Double
    fun update()
}
