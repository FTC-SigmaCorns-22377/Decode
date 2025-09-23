package sigmacorns.constants

import org.joml.Vector3d


object FieldZones
{
    //whenever you set a bound, try to do whatever calculations you do to find the coordinates in code
    //for example, if x is two meters to the right of the center, you might say "val x = 0 + 2"
    //make it readable as to how you got that value
    val GOAL = Pair(0.0f, 0.0f) //example
    val flywheelRadius = 0.048f
    val artifactRadius = 0.0635f
    val shotAngle = 0.0f
    val goalPosition = Vector3d(2.0, 0.0, 0.0)

}