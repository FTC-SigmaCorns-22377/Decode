package sigmacorns

import org.joml.Vector2d
import org.joml.Vector3d
import org.joml.minus
import sigmacorns.constants.FieldZones.artifactRadius
import sigmacorns.constants.FieldZones.goalPosition
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan

fun aimingShooter(state: State): Double {
    // check discord for physics derivation
    val directionVector = goalPosition - Vector3d(state.driveTrainPosition.v, 0.0)
    val driveTrainAngle = Vector3d(directionVector.x,directionVector.y,0.0).angle(directionVector)
    val driveTrain2dVector = Vector2d(directionVector.x, directionVector.y)
    val numerator = (directionVector.z().pow(2)*(9.81))
    val denominatorPartOne = (2*driveTrain2dVector.length()*tan(driveTrainAngle) - 2*directionVector.z())
    val denominatorPartTwo = cos(driveTrainAngle).pow(2)
    val denominator = (denominatorPartTwo)*(denominatorPartOne)
    val finalSubstitution = numerator/denominator
    return sqrt(finalSubstitution)

}