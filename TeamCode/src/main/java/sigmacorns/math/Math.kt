package sigmacorns.math

fun normalizeAngle(angle: Double): Double {
    var normalized = angle
    while (normalized > kotlin.math.PI) normalized -= 2 * kotlin.math.PI
    while (normalized < -kotlin.math.PI) normalized += 2 * kotlin.math.PI
    return normalized
}
