package sigmacorns.constants

import org.joml.Vector2d

enum class ShotZone(val pts: List<Vector2d>) {
    FAR(listOf(
        Vector2d(0.0,-1.178221),
        Vector2d(-0.589756,-1.768771),
        Vector2d(0.589756,-1.768771)
    )),

    CLOSE(listOf(
        Vector2d(-1.789906,1.776708),
        Vector2d(-1.789906,1.776708),
        Vector2d(0.0,-0.013992),
    ));
}