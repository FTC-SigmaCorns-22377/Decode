package sigmacorns

import org.joml.Vector2d
import org.joml.Vector3d
import org.joml.minus
import sigmacorns.constants.FieldZones.artifactRadius
import sigmacorns.constants.FieldZones.goalPosition
import sigmacorns.constants.TIME_LAG_LAUNCH
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan

fun aimingshooterconstantturretAngle(state: State): Double {
    // check discord for physics derivation
    val directionVector = goalPosition - (Vector3d(state.driveTrainPosition.v, 0.0) - (Vector3d(state.driveTrainVelocity.v, 0.0).mul(TIME_LAG_LAUNCH)))
    val driveTrainAngle = Vector3d(directionVector.x,directionVector.y,0.0).angle(directionVector)
    val driveTrain2dVector = Vector2d(directionVector.x, directionVector.y)
    val numerator = (driveTrain2dVector.length().pow(2)*(9.81))
    val denominatorPartOne = (2*driveTrain2dVector.length()*tan(driveTrainAngle) - 2*directionVector.z())
    val denominatorPartTwo = cos(driveTrainAngle).pow(2)
    val denominator = (denominatorPartTwo)*(denominatorPartOne)
    val finalSubstitution = numerator/denominator
    return sqrt(finalSubstitution)

}