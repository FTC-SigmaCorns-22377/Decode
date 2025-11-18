package sigmacorns.sim

<<<<<<< HEAD:TeamCode/src/main/java/sigmacorns/sim/spindexerParameters.kt
import sigmacorns.io.SigmaIO
import sigmacorns.sim.LinearDcMotor

=======
>>>>>>> origin/master:TeamCode/src/main/java/sigmacorns/sim/SpindexerParameters.kt
data class SpindexerParameters (
    val spinMotor: LinearDcMotor,
    val inertia: Double = 6.0, // the inertia of the spindexr when no balls are present
    val dinertia: Double = 7.0, // the inertia that each ball adds
    val spinViscousFriction: Double = 0.0

){

}