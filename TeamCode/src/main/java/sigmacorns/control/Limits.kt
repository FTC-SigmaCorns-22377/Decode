package sigmacorns.control

import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

class MotorRangeMapper(
    var limits: ClosedRange<Double>,
    var limitsTick: ClosedRange<Double>,
    var slowdownDist: Double
) {
    fun tickToPos(tick: Double) = limitsTick.remap(limits,tick)
    fun posToTick(pos: Double) = limits.remap(limitsTick, pos)

    fun limitPower(power: Double, pos: Double): Double {
        val dir = (limits.endInclusive - limits.start).sign
        val e0 = ((pos-limits.start)*dir/slowdownDist).coerceIn(0.0,1.0)
        val e1 = ((limits.endInclusive-pos)*dir/slowdownDist).coerceIn(0.0,1.0)
        return power.sign * power.absoluteValue*min(e0,e1)
    }

    private fun ClosedRange<Double>.remap(to: ClosedRange<Double>, v: Double): Double {
        return to.start + (v - this.start) * (to.endInclusive - to.start) / (this.endInclusive - this.start)
    }
}
