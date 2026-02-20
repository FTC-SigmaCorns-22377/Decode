package sigmacorns.math

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import sigmacorns.io.FTCPose2d

fun normalizeAngle(angle: Double): Double {
    var normalized = angle
    while (normalized > kotlin.math.PI) normalized -= 2 * kotlin.math.PI
    while (normalized < -kotlin.math.PI) normalized += 2 * kotlin.math.PI
    return normalized
}
fun FTCPose2d.toPose2d(): Pose2d = Pose2d(
    getX(DistanceUnit.METER),
    getY(DistanceUnit.METER),
    getHeading(AngleUnit.RADIANS)
)

fun Pose2d.toFtcPose2d(): FTCPose2d = FTCPose2d(
    DistanceUnit.METER,
    v.x,
    v.y,
    AngleUnit.RADIANS,
    rot
)
