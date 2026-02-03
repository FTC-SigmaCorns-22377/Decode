package sigmacorns.sim

enum class Balls() {
    Green,
    Purple,
    Empty
}


data class SpindexerState (
    var spindexerRotation: Double = 0.0, // this is the rotation of the motor that powers the spindexr( using ticks)
    var balls: List<Balls?>, //type of balls present in the spindexer
    var omega: Double = 0.0,
    ) {

    constructor(data: DoubleArray): this(
        spindexerRotation = data[0], // Extract spindexerRotation from data
        balls = listOf(null, null, null),
        omega = data[0],
    )

    fun toDoubleArray(): DoubleArray = doubleArrayOf(omega)

}
