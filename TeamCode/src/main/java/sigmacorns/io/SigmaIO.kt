package sigmacorns.io

interface SigmaIO {

    // drive motors
    var driveFL: Double
    var driveBL: Double
    var driveFR: Double
    var driveBR: Double

    //shooter
    var flyWheel: Double
    var guidingArm: Double //servo, the rest are motors

    //intake
    var intakeMotor: Double // might just be human fed and we wont end up needing this


    /* function to get robo position
    position( drive base)
    velocity(drive base)
     */

    fun position(): Array<Double>
    fun velocity(): Array<Double>
    fun flywheelVelocity(): Array<Double>

}
