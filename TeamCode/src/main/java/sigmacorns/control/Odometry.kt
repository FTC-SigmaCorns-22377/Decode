package sigmacorns.control

import org.joml.Vector2d
import sigmacorns.math.Pose2d
import kotlin.math.cos
import kotlin.math.sin

class Odometry(
    val deadwheelxOffset: Double,
    val deadwheelyOffset: Double,
    val metersPerTick: Double
) {


    var position = Pose2d()
    var xOld = 0
    var yOld = 0
    var rotOld = 0.0

    fun update(x: Int, y: Int, rot: Double) {
        var drot = (rot - rotOld)
        var dx = (x - xOld).toDouble() * metersPerTick + drot * deadwheelxOffset
        var dy = (y - yOld).toDouble() * metersPerTick - drot * deadwheelyOffset
        var fieldV = Vector2d(cos(rot) * dx - sin(rot) * dy, sin(rot) * dx + cos(rot) * dy)
        position += Pose2d(fieldV, rot)
        xOld = x
        yOld = y
        rotOld = rot

    }
}
