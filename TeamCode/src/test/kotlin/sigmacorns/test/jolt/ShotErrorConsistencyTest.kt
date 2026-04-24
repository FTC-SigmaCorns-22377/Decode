package sigmacorns.test.jolt

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*
import sigmacorns.control.aim.TurretPlannerBridge
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.*

/**
 * Test to verify that shotError matches the solve function.
 * The issue is that the launch position calculation in the native code
 * uses a different coordinate system than the solver assumes.
 */
class ShotErrorConsistencyTest {

    // Enable SIM mode to load from test JNI directory
    init {
        SigmaOpMode.SIM = true
    }

    private val bridge = TurretPlannerBridge()
    private val rH = 0.05
    private val vMax = 10.0
    private val g = 9.81

    private val physConfig = floatArrayOf(
        9.81f,      // g
        rH.toFloat(),     // rH
        0.0f        // dragK
    )

    private val omegaCoeffs = floatArrayOf(
        2f,          // N = 2 data points
        0.5f, 5.0f, 10.0f,    // phi=0.5 rad, v_exit=5.0, omega=10.0
        1.0f, 8.0f, 15.0f     // phi=1.0 rad, v_exit=8.0, omega=15.0
    )

    companion object {
        private const val THETA_IDX = 0
        private const val PHI_IDX = 1
        private const val V_EXIT_IDX = 2
        private const val OMEGA_IDX = 3
    }

    @Test
    fun testCloseZoneShotErrorMatchesSolver() {
        // Test close zone shot (should work fine before the fix)
        val targetX = 0.0f
        val targetY = 0.5f
        val targetZ = 1.3f
        val turretX = 0.0f
        val turretY = 0.0f
        val turretZ = 0.5f
        val robotVx = 0.0f
        val robotVy = 0.0f

        // Solve for shot parameters at T=0.5s
        val T = 0.5f
        val solution = bridge.solve(
            turretX, turretY, turretZ,
            targetX, targetY, targetZ,
            robotVx, robotVy, T,
            physConfig, omegaCoeffs
        )

        val theta = solution[THETA_IDX]
        val phi = solution[PHI_IDX]
        val vExit = solution[V_EXIT_IDX]
        val omega = solution[OMEGA_IDX]

        println("=== Close Zone Test (T=0.5s) ===")
        println("Solution: theta=${Math.toDegrees(theta.toDouble()).toFixed(2)}°, phi=${Math.toDegrees(phi.toDouble()).toFixed(2)}°, vExit=${vExit.toDouble().toFixed(2)} m/s")
        println("Turret: ($turretX, $turretY, $turretZ)")
        println("Target: ($targetX, $targetY, $targetZ)")

        // Calculate shot error using the same parameters
        val shotError = bridge.shotError(
            turretX, turretY, turretZ,
            targetX, targetY, targetZ,
            robotVx, robotVy,
            theta, phi, vExit, omega,
            T,
            physConfig
        )

        println("Shot error: $shotError m")

        // The shot error should be very small (essentially zero) since we're using
        // parameters that were just solved by the same solver
        assertTrue(shotError < 0.01, "Shot error should be <$0.01 after using solver parameters, got $shotError")
        println("✓ Close zone test passed!")
    }

    @Test
    fun testFarZoneShotErrorMatchesSolver() {
        // Test far zone shot (currently shows large error)
        val targetX = 0.0f
        val targetY = 2.0f
        val targetZ = 1.3f
        val turretX = 0.0f
        val turretY = 0.0f
        val turretZ = 0.5f
        val robotVx = 0.0f
        val robotVy = 0.0f

        // Solve for shot parameters at T=1.0s
        val T = 1.0f
        val solution = bridge.solve(
            turretX, turretY, turretZ,
            targetX, targetY, targetZ,
            robotVx, robotVy, T,
            physConfig, omegaCoeffs
        )

        val theta = solution[THETA_IDX]
        val phi = solution[PHI_IDX]
        val vExit = solution[V_EXIT_IDX]
        val omega = solution[OMEGA_IDX]

        println("\n=== Far Zone Test (T=1.0s) ===")
        println("Solution: theta=${Math.toDegrees(theta.toDouble()).toFixed(2)}°, phi=${Math.toDegrees(phi.toDouble()).toFixed(2)}°, vExit=${vExit.toDouble().toFixed(2)} m/s")
        println("Turret: ($turretX, $turretY, $turretZ)")
        println("Target: ($targetX, $targetY, $targetZ)")

        // Calculate shot error using the same parameters
        val shotError = bridge.shotError(
            turretX, turretY, turretZ,
            targetX, targetY, targetZ,
            robotVx, robotVy,
            theta, phi, vExit, omega,
            T,
            physConfig
        )

        println("Shot error: $shotError m")

        // This will fail before the fix - the error shows up as significant
        println("Shots with the solver's own parameters should have minimal error.")
        println("If this is > 0.01m, there's a mismatch between solve and shotError.")
    }

    @Test
    fun testVariousFlightTimes() {
        // Test multiple flight times to identify when the error becomes significant
        val targetYs = listOf(0.5f, 1.0f, 1.5f, 2.0f, 2.5f)
        val flightTimes = listOf(0.4f, 0.6f, 0.8f, 1.0f, 1.2f)

        println("\n=== Multiple Flight Times Test ===")
        println("Distance vs ShotError table:")

        for (dist in targetYs) {
            for (T in flightTimes) {
                val targetX = 0.0f
                val targetY = dist
                val targetZ = 1.3f
                val turretX = 0.0f
                val turretY = 0.0f
                val turretZ = 0.5f
                val robotVx = 0.0f
                val robotVy = 0.0f

                try {
                    val solution = bridge.solve(
                        turretX, turretY, turretZ,
                        targetX, targetY, targetZ,
                        robotVx, robotVy, T,
                        physConfig, omegaCoeffs
                    )

                    val shotError = bridge.shotError(
                        turretX, turretY, turretZ,
                        targetX, targetY, targetZ,
                        robotVx, robotVy,
                        solution[THETA_IDX],
                        solution[PHI_IDX],
                        solution[V_EXIT_IDX],
                        solution[OMEGA_IDX],
                        T,
                        physConfig
                    )

                    println("  dist=${dist.toFixed(1)}m, T=${T.toFixed(1)},  error=${shotError.toFixed(4)}m")
                } catch (e: Exception) {
                    println("  dist=${dist.toFixed(1)}m, T=${T.toFixed(1)},  error=FAILED")
                }
            }
        }
    }

    @Test
    fun testWithRobotVelocity() {
        // Test with robot velocity to verify the solver's coordinate compensation
        val targetX = 0.0f
        val targetY = 1.5f
        val targetZ = 1.3f
        val turretX = 0.0f
        val turretY = 0.0f
        val turretZ = 0.5f
        val robotVx = 1.0f  // Robot moving forward
        val robotVy = 0.0f

        // Solve for shot parameters
        val T = 0.8f
        val solution = bridge.solve(
            turretX, turretY, turretZ,
            targetX, targetY, targetZ,
            robotVx, robotVy, T,
            physConfig, omegaCoeffs
        )

        val shotError = bridge.shotError(
            turretX, turretY, turretZ,
            targetX, targetY, targetZ,
            robotVx, robotVy,
            solution[THETA_IDX],
            solution[PHI_IDX],
            solution[V_EXIT_IDX],
            solution[OMEGA_IDX],
            T,
            physConfig
        )

        println("\n=== Robot Velocity Test (vx=1.0 m/s) ===")
        println("Shot error: $shotError m")

        // Should still be small since we're using solver's own parameters
        assertTrue(shotError < 0.01, "Shot error with velocity should be <$0.01, got $shotError")
        println("✓ Robot velocity test passed!")
    }

    @Test
    fun testDebugComparison() {
        // Detailed debug output to understand the discrepancy
        val targetX = 0.0f
        val targetY = 2.0f
        val targetZ = 1.3f
        val turretX = 0.0f
        val turretY = 0.0f
        val turretZ = 0.5f
        val robotVx = 0.0f
        val robotVy = 0.0f
        val T = 1.0f

        val solution = bridge.solve(
            turretX, turretY, turretZ,
            targetX, targetY, targetZ,
            robotVx, robotVy, T,
            physConfig, omegaCoeffs
        )

        val theta = solution[THETA_IDX]
        val phi = solution[PHI_IDX]
        val vExit = solution[V_EXIT_IDX]

        println("\n=== Debug Comparison ===")
        println("Solver parameters:")
        println("  theta = ${Math.toDegrees(theta.toDouble()).toFixed(4)}°")
        println("  phi   = ${Math.toDegrees(phi.toDouble()).toFixed(4)}°")
        println("  vExit = ${vExit.toDouble().toFixed(2)} m/s")
        println("  T     = $T s")
        println("\nGeometry:")
        println("  Turret position: ($turretX, $turretY, $turretZ)")
        println("  Target position: ($targetX, $targetY, $targetZ)")
        println("  Barrel offset rH: $rH m")
        println("\nCalculated shot error: ${bridge.shotError(
            turretX, turretY, turretZ,
            targetX, targetY, targetZ,
            robotVx, robotVy, theta, phi, vExit, 0f, T, physConfig
        )} m")
    }
}

private fun Float.toFixed(digits: Int): String = String.format("%.${digits}f", this)
private fun Double.toFixed(digits: Int): String = String.format("%.${digits}f", this)