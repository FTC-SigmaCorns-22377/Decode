package sigmacorns.sim.viz

/**
 * Stub SimServer for robot deployment.
 * The real implementation is in src/test for desktop simulation only.
 * This stub does nothing - it's a no-op that allows DrakeSimIO to compile
 * for Android without pulling in Javalin dependencies.
 */
class SimServer(private val port: Int = 8000) {
    fun start() {
        // No-op on robot
    }

    fun stop() {
        // No-op on robot
    }

    fun broadcast(state: SimState) {
        // No-op on robot
    }

    fun setChoreoPath(path: List<PathPoint>) {
        // No-op on robot
    }

    fun getGamepad1(): GamepadState = GamepadState()
    fun getGamepad2(): GamepadState = GamepadState()
}

data class SimState(
    val t: Double,
    val base: BaseState,
    val joints: Map<String, Double>,
    val telemetry: TelemetryState,
    val wheelForces: List<ForceState> = emptyList(),
    val error: ErrorState? = null,
    val balls: List<BallState> = emptyList(),
    val mpcTarget: List<PathPoint> = emptyList()
)

data class BallState(
    val x: Double,
    val y: Double,
    val z: Double,
    val color: String = "orange"
)

data class ForceState(val x: Double, val y: Double, val z: Double)

data class ErrorState(
    val x: Double,
    val y: Double,
    val yaw: Double,
    val vx: Double,
    val vy: Double,
    val omega: Double
)

data class PathPoint(val x: Double, val y: Double)

data class BaseState(
    val x: Double,
    val y: Double,
    val z: Double,
    val roll: Double,
    val pitch: Double,
    val yaw: Double
)

data class TelemetryState(
    val fl: Double,
    val fr: Double,
    val bl: Double,
    val br: Double,
    val flywheel: Double,
    val turret: Double
)

