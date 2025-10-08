package sigmacorns.io

import sigmacorns.math.Pose2d
import kotlin.time.Duration

interface SigmaIO {

    // drive motors
    var driveFL: Double
    var driveBL: Double
    var driveFR: Double
    var driveBR: Double

    //shooter
    var shooter: Double
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

    fun setPosition(p: Pose2d)

    /**
     * @return time passed since the start of the opmode
     */
    fun time(): Duration
    fun configurePinpoint()
}
