package sigmacorns.sim

import sigmacorns.io.SigmaIO
import sigmacorns.sim.LinearDcMotor

data class SpindexerParameters (
    val spinMotor: LinearDcMotor,
    val inertia: Double = 6.0, // the inertia of the spindexr when no balls are present
    val dinertia: Double = 7.0, // the inertia that each ball adds
    val spinViscousFriction: Double = 0.0

){

}