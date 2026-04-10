package sigmacorns.test.jolt

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.params.ParameterizedTest
import org.junit.jupiter.params.provider.ValueSource
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.control.aim.Ballistics
import sigmacorns.control.aim.OmegaMap
import sigmacorns.control.aim.ShotSolver
import kotlin.math.*

/**
 * Ballistics trajectory verification tests using simple kinematic simulation.
 *
 * This test suite simulates actual ball trajectories using basic physics (gravity, no air resistance)
 * to verify that calculated shot parameters result in hits. For each test:
 *
 * 1. Use Ballistics solver to compute required launch parameters (vExit, phi, theta) for a target
 * 2. Simulate the 3D trajectory using kinematic equations with gravity
 * 3. Track minimum distance to target as ball flies
 * 4. Verify that the minimum distance is within acceptable tolerance
 *
 * Hood angle decomposition (matches JoltSimIO.shootBall() lines 195-201):
 *   - Hood angle (phi) splits exit speed into components:
 *     * horizontalSpeed = vExit * cos(phi)
 *     * verticalSpeed = vExit * sin(phi)
 *   - Azimuth angle (theta) rotates horizontal component in xy-plane:
 *     * vx = horizontalSpeed * cos(theta) + robotVx
 *     * vy = horizontalSpeed * sin(theta) + robotVy
 *     * vz = verticalSpeed
 *   - Robot velocity is added to the shot (JoltSimIO behavior)
 *
 * Coordinate system:
 *   - x: forward (robot front)
 *   - y: left (robot left side)
 *   - z: up (vertical)
 *   - theta: azimuth angle in xy-plane (0 = forward, π/2 = left)
 *   - phi: elevation angle from horizontal (0 = horizontal, π/2 = straight up)
 *
 * Example trajectory for close-range shot (0.5m, 1.3m height):
 *   vExit=4.19 m/s, phi=70.4°, theta=90.0°
 *   Resulting accuracy: 1.5cm hit distance
 */
class BallisticsTrajectorySimTest {

    // Ballistics parameters matching robot shooter
    private val ballistics = Ballistics(
        rH = 0.05,              // 5cm barrel offset
        vMax = 15.0,            // m/s max exit velocity
        phiMin = Math.toRadians(0.0),   // 0° min elevation
        phiMax = Math.toRadians(80.0),  // 80° max elevation
        g = 9.81                // gravity
    )

    // Simple omega map for optimization
    private val testOmegaMap = object : OmegaMap {
        override fun omega(hood: Double, vExit: Double): Double {
            return abs(hood) * 0.5 + vExit / 100.0
        }

        override fun lipschitzBound(
            hood: ClosedRange<Double>,
            vExit: ClosedRange<Double>,
            lHood: Double,
            lvExit: Double
        ): Double {
            val maxHood = max(abs(hood.start), abs(hood.endInclusive))
            return maxHood * 0.5 + 40.0 / 100.0
        }
    }

    private val shotSolver = ShotSolver(
        omega = testOmegaMap,
        wOmega = 0.1,
        wTheta = 0.05,
        wPhi = 0.05,
        ballistics = ballistics
    )

    /**
     * Simple kinematic trajectory simulator.
     * Simulates a projectile launched with given initial position and velocity,
     * accounting for hood angle offset to launch position.
     */
    data class TrajectorySimulator(
        val initialPos: Vector3d,
        val initialVel: Vector3d,
        val gravity: Double = 9.81
    ) {
        /**
         * Get ball position at time t.
         */
        fun positionAt(t: Double): Vector3d {
            return Vector3d(
                initialPos.x + initialVel.x * t,
                initialPos.y + initialVel.y * t,
                initialPos.z + initialVel.z * t - 0.5 * gravity * t * t
            )
        }

        /**
         * Check if ball has hit the ground (z <= 0).
         */
        fun hasHitGround(pos: Vector3d): Boolean = pos.z <= 0.0

        /**
         * Simulate trajectory and return minimum distance to target.
         */
        fun simulateToTarget(
            target: Vector3d,
            timeStep: Double = 0.001,
            maxTime: Double = 5.0,
            minHeightLimit: Double = -0.1
        ): Pair<Double, Double> {
            var minDistance = Double.MAX_VALUE
            var impactTime = 0.0
            var t = 0.0

            while (t < maxTime) {
                val pos = positionAt(t)

                // Check if hit ground
                if (pos.z < minHeightLimit) {
                    impactTime = t
                    break
                }

                // Compute distance to target
                val distance = pos.distance(target)
                minDistance = min(minDistance, distance)

                t += timeStep
            }

            return Pair(minDistance, impactTime)
        }
    }

    /**
     * Helper: Create a target at field position from robot position.
     */
    private fun createTarget(
        robotPos: Vector3d,
        targetFieldX: Double,
        targetFieldY: Double,
        targetZ: Double
    ): Ballistics.Target {
        return Ballistics.Target(
            target = Vector3d(targetFieldX, targetFieldY, targetZ),
            turret = robotPos,
            vR = Vector2d(0.0, 0.0)
        )
    }

    /**
     * Helper: Launch a projectile with given spherical coordinates, matching JoltSimIO behavior.
     *
     * Converts theta (azimuth) and phi (elevation) + vExit into Cartesian velocity,
     * accounting for robot velocity being added to the shot.
     *
     * This matches JoltSimIO.shootBall():
     *   val horizontalSpeed = exitSpeed * cos(hoodAngleRad)
     *   val verticalSpeed = exitSpeed * sin(hoodAngleRad)
     *   val vx = horizontalSpeed * cos(launchHeading) + robotState[3]
     *   val vy = horizontalSpeed * sin(launchHeading) + robotState[4]
     *   val vz = verticalSpeed
     *
     * Note: The ballistics solver computes theta and phi in a robot-relative frame
     * (accounting for robot velocity in the target). Here we convert to absolute
     * field frame by adding robot velocity.
     */
    private fun computeVelocity(
        vExit: Double,
        theta: Double,  // azimuth in radians (atan2 of relative velocity)
        phi: Double,    // elevation in radians
        robotVx: Double = 0.0,  // robot velocity x (field frame)
        robotVy: Double = 0.0   // robot velocity y (field frame)
    ): Vector3d {
        // Hood angle splits exit speed into horizontal and vertical components
        // (matching JoltSimIO line 196-197)
        val horizontalSpeed = vExit * cos(phi)
        val verticalSpeed = vExit * sin(phi)

        // Decompose horizontal component by azimuth angle in xy-plane
        // Then add robot velocity to get absolute velocity in field frame
        // (matching JoltSimIO line 199-201)
        val vx = horizontalSpeed * cos(theta) + robotVx
        val vy = horizontalSpeed * sin(theta) + robotVy
        val vz = verticalSpeed

        return Vector3d(vx, vy, vz)
    }

    /**
     * Helper: Compute launch position accounting for hood angle offset.
     *
     * Matches JoltSimIO.shootBall() lines 208-211:
     *   val spawnX = shooterX + cos(launchHeading)*(1.0 - sin(hoodAngleRad))*ballExitRadius
     *   val spawnY = shooterY + sin(launchHeading)*(1.0 - sin(hoodAngleRad))*ballExitRadius
     *   val spawnZ = shooterZ + cos(hoodAngleRad)*ballExitRadius
     *
     * The hood angle affects where the ball spawns relative to the turret position.
     */
    private fun computeLaunchPos(
        turretPos: Vector3d,
        phi: Double,           // hood angle in radians
        theta: Double,         // azimuth angle in radians
        ballExitRadius: Double = 0.0635 // ball radius in meters, from JoltSimIO constants
    ): Vector3d {
        // Launch heading combines turret rotation in xy-plane
        // In ballistics frame: theta already accounts for yaw, phi for elevation
        val launchHeading = theta

        // Hood angle offset to spawn position (perpendicular to barrel, adjusts z and forward distance)
        val horizontalOffset = (1.0 - sin(phi)) * ballExitRadius
        val verticalOffset = cos(phi) * ballExitRadius

        return Vector3d(
            turretPos.x + cos(launchHeading) * horizontalOffset,
            turretPos.y + sin(launchHeading) * horizontalOffset,
            turretPos.z + verticalOffset
        )
    }

    /**
     * Helper: Simulate a shot and measure accuracy.
     * Automatically extracts robot velocity from target and adds it to shot velocity
     * (matching JoltSimIO behavior where robot velocity is added to the launched ball).
     * Also adjusts launch position based on hood angle offset.
     */
    private fun shootAndMeasure(
        turretPos: Vector3d,     // base turret position
        targetPos: Vector3d,
        vExit: Double,
        phi: Double,    // radians (hood angle)
        theta: Double,  // radians (azimuth angle)
        robotVx: Double = 0.0,  // robot velocity x (field frame)
        robotVy: Double = 0.0   // robot velocity y (field frame)
    ): Pair<Double, String> {
        // Compute launch position accounting for hood angle offset (matches JoltSimIO lines 208-211)
        val turretPos = computeLaunchPos(turretPos, phi, theta)

        // Compute absolute velocity (robot velocity is added by JoltSimIO)
        val vel = computeVelocity(vExit, theta, phi, robotVx, robotVy)
        val sim = TrajectorySimulator(turretPos, vel)
        val (minDistance, impactTime) = sim.simulateToTarget(targetPos)

        val details = "vExit=%.2f m/s, phi=%.1f°, theta=%.1f°, minDist=%.3f m, impactTime=%.3f s".format(
            vExit,
            Math.toDegrees(phi),
            Math.toDegrees(theta),
            minDistance,
            impactTime
        )

        return Pair(minDistance, details)
    }

    // ========== Tests ==========

    @Test
    fun testHoodAngleAffectsLaunchPosition() {
        // Verify that hood angle changes the launch position (spawn offset).
        // This matches JoltSimIO lines 208-211:
        //   val spawnX = shooterX + cos(launchHeading)*(1.0 - sin(hoodAngleRad))*ballExitRadius
        //   val spawnY = shooterY + sin(launchHeading)*(1.0 - sin(hoodAngleRad))*ballExitRadius
        //   val spawnZ = shooterZ + cos(hoodAngleRad)*ballExitRadius
        //
        // When phi=0 (horizontal):
        //   spawnX/Y offset = (1.0 - sin(0)) * r = 1.0 * r = r
        //   spawnZ offset = cos(0) * r = 1.0 * r = r
        // When phi=90° (vertical):
        //   spawnX/Y offset = (1.0 - sin(90°)) * r = 0.0 * r = 0
        //   spawnZ offset = cos(90°) * r = 0.0 * r = 0

        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val theta = 0.0  // Forward direction
        val r = 0.0635   // ballExitRadius

        // At phi=0 (horizontal hood), ball spawns with maximum forward offset
        val launchPos0 = computeLaunchPos(turretPos, phi = 0.0, theta)
        println("Hood angle=0°: spawn at (${launchPos0.x.toFixed(4)}, ${launchPos0.y.toFixed(4)}, ${launchPos0.z.toFixed(4)})")
        // Expected: (r, 0, turretZ + r) ≈ (0.0635, 0, 0.5635)
        assertEquals(r, launchPos0.x, 0.001, "At phi=0°, should spawn forward by ballExitRadius")
        assertEquals(turretPos.z + r, launchPos0.z, 0.001, "At phi=0°, should spawn up by ballExitRadius from turret")

        // At phi=90° (vertical hood), ball spawns with no forward offset, same height as turret
        val phi90 = Math.toRadians(90.0)
        val launchPos90 = computeLaunchPos(turretPos, phi = phi90, theta)
        println("Hood angle=90°: spawn at (${launchPos90.x.toFixed(4)}, ${launchPos90.y.toFixed(4)}, ${launchPos90.z.toFixed(4)})")
        // Expected: (0, 0, turretZ) = (0, 0, 0.5)
        assertEquals(0.0, launchPos90.x, 0.001, "At phi=90°, should have no forward offset")
        assertEquals(turretPos.z, launchPos90.z, 0.001, "At phi=90°, should spawn at turret height (no vertical offset)")

        // At phi=45° (intermediate angle), spawn position is intermediate
        val phi45 = Math.toRadians(45.0)
        val launchPos45 = computeLaunchPos(turretPos, phi = phi45, theta)
        println("Hood angle=45°: spawn at (${launchPos45.x.toFixed(4)}, ${launchPos45.y.toFixed(4)}, ${launchPos45.z.toFixed(4)})")
        // Should be between the two extremes
        assertTrue(launchPos45.x > launchPos90.x && launchPos45.x < launchPos0.x,
            "Forward offset should decrease as hood angle increases from 0° to 90°")
        assertTrue(launchPos45.z > launchPos90.z && launchPos45.z < launchPos0.z,
            "Vertical offset should decrease as hood angle increases from 0° to 90°")

        println("Hood angle effect verified: spawn position correctly adjusts with hood angle")
    }

    @Test
    fun testTrajectorySimulatorBasics() {
        // Simple sanity check: horizontal shot should eventually hit ground
        val initialPos = Vector3d(0.0, 0.0, 1.0)
        val initialVel = Vector3d(1.0, 0.0, 0.0)  // Horizontal shot

        val sim = TrajectorySimulator(initialPos, initialVel)

        // Check positions over time
        val pos0 = sim.positionAt(0.0)
        assertEquals(0.0, pos0.x, 0.001)
        assertEquals(1.0, pos0.z, 0.001)

        val pos1 = sim.positionAt(1.0)
        assertEquals(1.0, pos1.x, 0.001)  // Moved forward
        assertTrue(pos1.z < 1.0, "Should drop due to gravity")

        // Find when it hits ground
        var tHit = 0.0
        for (i in 0..200) {
            val t = i * 0.01
            val p = sim.positionAt(t)
            if (p.z < 0.0) {
                tHit = t
                println("Horizontal shot hits ground at t=$t, x=${p.x}, z=${p.z}")
                assertTrue(t < 1.0, "Should hit in < 1s with gravity")
                break
            }
        }
        assertTrue(tHit > 0.0, "Should have found impact time")
    }

    @Test
    fun testCloseRangeShot() {
        // Close-range shot to target at y=0.5m, z=1.3m
        val turretPos = Vector3d(0.0, 0.0, 0.5)  // Turret height
        val targetPos = Vector3d(0.0, 0.5, 1.3)

        val target = createTarget(turretPos, 0.0, 0.5, 1.3)

        // Solve for shot parameters
        val bounds = ballistics.tBounds(target, 0.001)
        assertTrue(bounds.start < bounds.endInclusive, "Should have valid bounds")

        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)

        println("Close-range solution: vExit=${solution.vExit.toFixed(2)}, phi=${Math.toDegrees(solution.phi).toFixed(1)}°")

        // Pass robot velocity from target (stationary in this case)
        // Launch position will be adjusted based on hood angle phi
        val (minDist, details) = shootAndMeasure(
            turretPos, targetPos, solution.vExit, solution.phi, solution.theta,
            target.vR.x, target.vR.y
        )
        println("Close-range result: $details")

        assertTrue(minDist < 0.5, "Close-range shot should be very accurate (< 50cm), got $minDist m")
    }

    @Test
    fun testMidRangeShot() {
        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val targetPos = Vector3d(0.0, 1.5, 1.2)

        val target = createTarget(turretPos, 0.0, 1.5, 1.2)

        val bounds = ballistics.tBounds(target, 0.001)
        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)

        println("Mid-range solution: vExit=${solution.vExit.toFixed(2)}, phi=${Math.toDegrees(solution.phi).toFixed(1)}°")

        // Pass robot velocity from target (matches JoltSimIO behavior where robot velocity is added)
        // Launch position will be adjusted based on hood angle phi
        val (minDist, details) = shootAndMeasure(
            turretPos, targetPos, solution.vExit, solution.phi, solution.theta,
            target.vR.x, target.vR.y
        )
        println("Mid-range result: $details")

        assertTrue(minDist < 0.6, "Mid-range shot should be reasonably accurate (< 60cm), got $minDist m")
    }

    @ParameterizedTest
    @ValueSource(doubles = [0.3, 0.5, 0.8, 1.0, 1.3])
    fun testVaryingDistances(distance: Double) {
        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val targetPos = Vector3d(0.0, distance, 1.2)

        val target = createTarget(turretPos, 0.0, distance, 1.2)

        val bounds = ballistics.tBounds(target, 0.001)
        if (bounds.start >= bounds.endInclusive) {
            println("Distance=$distance m: Infeasible")
            return
        }

        val tMid = (bounds.start + bounds.endInclusive) / 2.0
        val solution = ballistics.solve(target, tMid)

        // Pass robot velocity from target (will be 0 for stationary target)
        val (minDist, details) = shootAndMeasure(
            turretPos, targetPos, solution.vExit, solution.phi, solution.theta,
            target.vR.x, target.vR.y
        )
        println("Distance=$distance m: $details")

        // Tolerance increases with distance
        val tolerance = 0.3 + distance * 0.15
        assertTrue(minDist < tolerance, "Shot at $distance m should be within $tolerance m tolerance, got $minDist m")
    }

    @Test
    fun testSymmetricShotsLeftRight() {
        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val leftTarget = Vector3d(-0.5, 0.8, 1.2)
        val rightTarget = Vector3d(0.5, 0.8, 1.2)

        val leftBallTarget = createTarget(turretPos, -0.5, 0.8, 1.2)
        val rightBallTarget = createTarget(turretPos, 0.5, 0.8, 1.2)

        val leftBounds = ballistics.tBounds(leftBallTarget, 0.001)
        val rightBounds = ballistics.tBounds(rightBallTarget, 0.001)

        val leftSol = ballistics.solve(leftBallTarget, (leftBounds.start + leftBounds.endInclusive) / 2.0)
        val rightSol = ballistics.solve(rightBallTarget, (rightBounds.start + rightBounds.endInclusive) / 2.0)

        // Verify solutions are symmetric
        assertEquals(leftSol.vExit, rightSol.vExit, 0.2, "Symmetric shots should have same exit velocity")
        assertEquals(leftSol.phi, rightSol.phi, 0.1, "Symmetric shots should have same hood angle")

        // Theta angles should be opposite (symmetric about robot heading)
        val expectedTheta = PI - rightSol.theta
        assertEquals(leftSol.theta, expectedTheta, 0.2, "Theta angles should be symmetric")

        // Shoot both with robot velocity from target (stationary in this case)
        val (leftDist, _) = shootAndMeasure(
            turretPos, leftTarget, leftSol.vExit, leftSol.phi, leftSol.theta,
            leftBallTarget.vR.x, leftBallTarget.vR.y
        )
        val (rightDist, _) = shootAndMeasure(
            turretPos, rightTarget, rightSol.vExit, rightSol.phi, rightSol.theta,
            rightBallTarget.vR.x, rightBallTarget.vR.y
        )

        println("Left shot: vExit=${leftSol.vExit.toFixed(2)}, phi=${Math.toDegrees(leftSol.phi).toFixed(1)}°, accuracy=${"%.3f".format(leftDist)}m")
        println("Right shot: vExit=${rightSol.vExit.toFixed(2)}, phi=${Math.toDegrees(rightSol.phi).toFixed(1)}°, accuracy=${"%.3f".format(rightDist)}m")

        // Both should be reasonably accurate
        assertTrue(leftDist < 0.5, "Left shot accuracy")
        assertTrue(rightDist < 0.5, "Right shot accuracy")
    }

    @Test
    fun testHighAngleShot() {
        // High-angle shot (lob pass)
        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val targetPos = Vector3d(0.0, 0.6, 1.5)  // Higher target

        val target = createTarget(turretPos, 0.0, 0.6, 1.5)

        val bounds = ballistics.tBounds(target, 0.001)
        // Use longer flight time for higher arc
        val tLong = bounds.start + (bounds.endInclusive - bounds.start) * 0.8
        val solution = ballistics.solve(target, tLong)

        println("High-angle solution: vExit=${solution.vExit.toFixed(2)}, phi=${Math.toDegrees(solution.phi).toFixed(1)}°")

        // Pass robot velocity from target (matches JoltSimIO behavior)
        val (minDist, details) = shootAndMeasure(
            turretPos, targetPos, solution.vExit, solution.phi, solution.theta,
            target.vR.x, target.vR.y
        )
        println("High-angle result: $details")

        assertTrue(minDist < 0.5, "High-angle shot should still be accurate")
    }

    @Test
    fun testLowAngleShot() {
        // Low-angle shot (line drive)
        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val targetPos = Vector3d(0.0, 0.8, 0.9)  // Lower target

        val target = createTarget(turretPos, 0.0, 0.8, 0.9)

        val bounds = ballistics.tBounds(target, 0.001)
        // Use shorter flight time for shallower arc
        val tShort = bounds.start + (bounds.endInclusive - bounds.start) * 0.2
        val solution = ballistics.solve(target, tShort)

        println("Low-angle solution: vExit=${solution.vExit.toFixed(2)}, phi=${Math.toDegrees(solution.phi).toFixed(1)}°")

        // Pass robot velocity from target (matches JoltSimIO behavior)
        val (minDist, details) = shootAndMeasure(
            turretPos, targetPos, solution.vExit, solution.phi, solution.theta,
            target.vR.x, target.vR.y
        )
        println("Low-angle result: $details")

        assertTrue(minDist < 0.6, "Low-angle shot should be accurate")
    }

    @Test
    fun testOptimalAdjustmentAccuracy() {
        // Verify that optimalAdjust improves accuracy
        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val targetPos = Vector3d(0.0, 0.8, 1.2)

        val target = createTarget(turretPos, 0.0, 0.8, 1.2)

        val bounds = ballistics.tBounds(target, 0.001)
        val tMax = bounds.endInclusive

        // Start with suboptimal solution
        val initial = ballistics.solve(target, tMax)

        // Find optimal
        val optimal = shotSolver.optimalAdjust(initial, target, tol = 0.01)

        // Compare accuracy, passing robot velocity from target
        val (initialDist, _) = shootAndMeasure(
            turretPos, targetPos, initial.vExit, initial.phi, initial.theta,
            target.vR.x, target.vR.y
        )
        val (optimalDist, _) = shootAndMeasure(
            turretPos, targetPos, optimal.vExit, optimal.phi, optimal.theta,
            target.vR.x, target.vR.y
        )

        println("Initial accuracy: ${"%.3f".format(initialDist)} m")
        println("Optimal accuracy: ${"%.3f".format(optimalDist)} m")

        // Optimal should be at least as good
        assertTrue(optimalDist <= initialDist + 0.1, "Optimal should be at least as good as initial")
    }

    @Test
    fun testFlightTimeParameterSweep() {
        // Sweep flight times and verify all produce hits
        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val targetPos = Vector3d(0.0, 0.7, 1.2)

        val target = createTarget(turretPos, 0.0, 0.7, 1.2)

        val bounds = ballistics.tBounds(target, 0.001)
        val tMin = bounds.start
        val tMax = bounds.endInclusive

        // Sample flight times across the feasible range
        val flightTimes = listOf(
            tMin,
            tMin + (tMax - tMin) * 0.25,
            tMin + (tMax - tMin) * 0.5,
            tMin + (tMax - tMin) * 0.75,
            tMax
        )

        println("Flight time sweep from ${"%.3f".format(tMin)} to ${"%.3f".format(tMax)} seconds:")

        for (t in flightTimes) {
            val solution = ballistics.solve(target, t)
            // Pass robot velocity from target (matches JoltSimIO behavior)
            val (minDist, _) = shootAndMeasure(
                turretPos, targetPos, solution.vExit, solution.phi, solution.theta,
                target.vR.x, target.vR.y
            )
            println("  t=${"%.3f".format(t)}s: vExit=${"%.2f".format(solution.vExit)} m/s, phi=${"%.1f".format(Math.toDegrees(solution.phi))}°, accuracy=${"%.3f".format(minDist)}m")
            assertTrue(minDist < 0.7, "All flight times should produce reasonable accuracy")
        }
    }

    @Test
    fun testMultipleTargetsSequence() {
        // Test shooting at multiple targets in sequence
        val turretPos = Vector3d(0.0, 0.0, 0.5)
        val targets = listOf(
            Vector3d(0.0, 0.4, 1.2),
            Vector3d(0.3, 0.6, 1.3),
            Vector3d(-0.3, 0.8, 1.1),
            Vector3d(0.0, 1.0, 1.2)
        )

        println("Multi-target sequence:")
        var successCount = 0

        for ((idx, targetPos) in targets.withIndex()) {
            val ballTarget = createTarget(turretPos, targetPos.x, targetPos.y, targetPos.z)

            val bounds = ballistics.tBounds(ballTarget, 0.001)
            if (bounds.start >= bounds.endInclusive) {
                println("  Target $idx: Infeasible")
                continue
            }

            val tMid = (bounds.start + bounds.endInclusive) / 2.0
            val solution = ballistics.solve(ballTarget, tMid)

            // Pass robot velocity from target (matches JoltSimIO behavior)
            val (minDist, _) = shootAndMeasure(
                turretPos, targetPos, solution.vExit, solution.phi, solution.theta,
                ballTarget.vR.x, ballTarget.vR.y
            )
            println("  Target $idx at (${"%.1f".format(targetPos.x)}, ${"%.1f".format(targetPos.y)}, ${"%.1f".format(targetPos.z)}): accuracy=${"%.3f".format(minDist)}m")

            if (minDist < 0.6) {
                successCount++
            }
        }

        assertTrue(successCount >= 3, "Should hit at least 3 out of 4 targets")
    }

    @Test
    fun testFeasibilityBoundary() {
        // Find the maximum distance where shots are feasible and accurate
        val turretPos = Vector3d(0.0, 0.0, 0.5)

        println("Feasibility boundary search:")
        var maxFeasible = 0.0

        for (i in 0..40) {
            val distance = 0.3 + i * 0.05
            val targetPos = Vector3d(0.0, distance, 1.2)

            val ballTarget = createTarget(turretPos, 0.0, distance, 1.2)

            val bounds = ballistics.tBounds(ballTarget, 0.001)
            if (bounds.start >= bounds.endInclusive) continue

            val tMid = (bounds.start + bounds.endInclusive) / 2.0
            val solution = ballistics.solve(ballTarget, tMid)

            if (solution.vExit > ballistics.vMax) continue

            // Pass robot velocity from target (matches JoltSimIO behavior)
            val (minDist, _) = shootAndMeasure(
                turretPos, targetPos, solution.vExit, solution.phi, solution.theta,
                ballTarget.vR.x, ballTarget.vR.y
            )

            // Allow looser tolerance at distance
            val tolerance = 0.3 + distance * 0.2
            if (minDist < tolerance) {
                maxFeasible = distance
                println("  Distance=${"%.2f".format(distance)}m: FEASIBLE (accuracy=${"%.3f".format(minDist)}m)")
            } else {
                println("  Distance=${"%.2f".format(distance)}m: MISS (accuracy=${"%.3f".format(minDist)}m)")
            }
        }

        println("Maximum feasible range: ${"%.2f".format(maxFeasible)} m")
    }
}

// Helper extension for formatting doubles
private fun Double.toFixed(digits: Int): String {
    return String.format("%.${digits}f", this)
}
