package sigmacorns.io

import sigmacorns.math.Pose2d

interface SigmaIO {

    // drive motors
    var driveFL: Double
    var driveBL: Double
    var driveFR: Double
    var driveBR: Double

    //shooter
    var flyWheel0: Double
    var flyWheel1: Double
    //intake
    var intake: Double // might just be human fed and we wont end up needing this


    /* function to get robo position
    position( drive base)
    velocity(drive base)
     */

    fun position(): Pose2d
    fun velocity(): Pose2d
    fun flywheelVelocity(): Double

    fun update()
}
