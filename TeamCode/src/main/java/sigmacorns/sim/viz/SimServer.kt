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
    val mpcTarget: List<PathPoint> = emptyList(),
    /** MPC predicted state evolution over the planning horizon (K knot points). */
    val mpcPredicted: List<PathPoint> = emptyList(),
    /** MPC contour targets sent to the solver (K knot points with heading). */
    val mpcContours: List<ContourVizState> = emptyList(),
    /** MPC horizon metadata. */
    val mpcHorizon: MPCHorizonState? = null,
    /** GTSAM factor graph visualization state. */
    val gtsam: GTSAMVizState? = null,
    /** Aiming / move-while-shoot visualization state. */
    val aiming: AimingVizState? = null
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
    val turret: Double,
    val spindexerPower: Double = 0.0,
    val spindexerAngle: Double = 0.0,
    val intakePower: Double = 0.0
)

/** A contour point with heading for MPC visualization. */
data class ContourVizState(
    val x: Double,
    val y: Double,
    val theta: Double,
    val vx: Double = 0.0,
    val vy: Double = 0.0
)

/** MPC horizon metadata for visualization. */
data class MPCHorizonState(
    val horizonSec: Double,
    val solveTimeMs: Double = 0.0,
    val sampleIndex: Int = 0,
    val trajectoryComplete: Boolean = false,
    /** Time offsets for each knot point (variable timesteps). */
    val knotTimesMs: List<Double> = emptyList()
)

/** GTSAM factor graph visualization data. */
data class GTSAMVizState(
    /** Fused (corrected) robot pose. */
    val fusedX: Double,
    val fusedY: Double,
    val fusedTheta: Double,
    /** Localization uncertainty (0..1). */
    val uncertainty: Double = 0.0,
    /** Whether GTSAM has been initialized. */
    val initialized: Boolean = false,
    /** Number of factors in the graph. */
    val graphFactors: Int = 0,
    /** Current pose node index. */
    val poseIndex: Int = 0,
    /** GTSAM solve time in ms. */
    val solveTimeMs: Double = 0.0,
    /** Landmark (AprilTag) positions for 3D display. */
    val landmarks: List<LandmarkVizState> = emptyList(),
    /** Currently detected tag IDs. */
    val detectedTags: List<Int> = emptyList(),
    /** Whether we have a current vision target. */
    val hasVision: Boolean = false,
    /** Whether we're using dead-reckoning prediction. */
    val usingPrediction: Boolean = false
)

/** Landmark data for visualization. */
data class LandmarkVizState(
    val tagId: Int,
    val x: Double,
    val y: Double,
    val z: Double,
    val yaw: Double = 0.0
)

/** Aiming / move-while-shoot visualization data. */
data class AimingVizState(
    /** Goal (basket) position in field frame. */
    val goalX: Double,
    val goalY: Double,
    /** Turret yaw angle in field frame (robot yaw + turret offset). */
    val turretFieldYaw: Double,
    /** Whether auto-aim has a target lock. */
    val hasTarget: Boolean = false,
    /** Whether aim is from prediction vs vision. */
    val usingPrediction: Boolean = false,
    /** Target distance in meters. */
    val targetDistance: Double = 0.0,
    /** Flywheel angular velocity (rad/s). */
    val flywheelOmega: Double = 0.0,
    /** Target flywheel velocity (rad/s). */
    val flywheelTarget: Double = 0.0,
    /** Whether flywheel is at target speed (ready to fire). */
    val flywheelReady: Boolean = false,
    /** Hood pitch angle (radians). */
    val hoodPitch: Double = 0.0,
    /** Projectile trajectory arc points (x,y,z in field frame). */
    val projectileArc: List<Point3D> = emptyList(),
    /** Turret effective target angle (robot-relative). */
    val turretTargetAngle: Double = 0.0,
    /** Turret actual angle (robot-relative). */
    val turretActualAngle: Double = 0.0,
    /** Robot velocity (for showing lead compensation). */
    val robotVx: Double = 0.0,
    val robotVy: Double = 0.0
)

data class Point3D(val x: Double, val y: Double, val z: Double)

