package sigmacorns.control.aim

import com.bylazar.configurables.annotations.Configurable
import kotlin.math.PI

/**
 * Configuration for move-while-shoot solver.
 * Tunable via Panels dashboard.
 */
@Configurable
object MoveWhileShootConfig {
    // Timing parameters (seconds)
    @JvmField var transferTime = 0.3          // Ball transfer time
    @JvmField var contactTime = 0.075         // Ball-flywheel contact time

    // Solver parameters
    @JvmField var maxHorizon = 5.0            // Maximum lookahead time (s)
    @JvmField var binarySearchTolerance = 0.01 // Convergence tolerance for T (s)
    @JvmField var maxBinarySearchIterations = 20
    @JvmField var maxRefinementIterations = 5  // Iterations for flywheel speed convergence
    @JvmField var flywheelSpeedTolerance = 5.0 // Convergence tolerance for ω (rad/s)

    // Flywheel parameters
    @JvmField var maxFlywheelSpeed = 628.0    // Max flywheel angular velocity (rad/s)

    // 3D kinematics parameters
    @JvmField var launchOffsetX = 0.03401469         // Ball exit offset from robot center, forward (m)
    @JvmField var launchOffsetY = 0.0         // Ball exit offset from robot center, left (m)
    @JvmField var launchOffsetZ = 0.37492951        // Ball exit height above ground (m)
    @JvmField var launchAngle = Math.toRadians(37.133)      // Fixed ball launch elevation angle (rad)
    @JvmField var gravity = 9.81              // Gravitational acceleration (m/s²)
    @JvmField var goalHeight = 0.984250           // Goal target height above ground (m)
    @JvmField var numericSolverIterations = 20 // Bisection iterations for time-of-flight

    // Turret constraints (robot frame)
    @JvmField var turretMaxAngle = 1.57       // Max turret angle magnitude (rad, ~90 deg)

    // Feature toggles
    @JvmField var enabled = false             // Enable move-while-shoot
    @JvmField var minRobotSpeed = 0.1         // Minimum robot speed to use MWS (m/s)

    // State prediction
    @JvmField var predictionDt = 0.001        // RK4 integration timestep (s)
    @JvmField var predictionSampleDt = 0.05   // Trajectory sample interval (s)

    // Cache invalidation tolerances
    @JvmField var trajectoryPositionTolerance = 0.05  // Max position deviation before recalc (m)
    @JvmField var trajectoryVelocityTolerance = 0.2   // Max velocity deviation before recalc (m/s)
}
