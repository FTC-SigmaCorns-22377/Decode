package sigmacorns.test

import org.joml.Vector2d
import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.control.aim.MoveWhileShootConfig
import sigmacorns.control.aim.MoveWhileShootSolver
import sigmacorns.control.aim.MuzzleSpeedMapping
import sigmacorns.constants.drivetrainParameters
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumDynamics
import sigmacorns.sim.MecanumState
import sigmacorns.tuning.AdaptiveTuner
import sigmacorns.tuning.ShotDataStore
import sigmacorns.tuning.SpeedPoint
import kotlin.math.*

class MoveWhileShootSolverTest {

    private val alpha = PI / 4.0  // 45 degree launch angle
    private val g = 9.81
    private val launchHeight = 0.12
    private val goalHeight = 0.80
    private val dz = goalHeight - launchHeight

    // Physically consistent tuner data: omega proportional to required muzzle speed.
    // vm = dh * sqrt(g / (dh - dz)) at each distance, then omega = vm / 0.025 (BALL_EXIT_SPEED_PER_RADIAN)
    // This ensures the (omega, vm) mapping is monotonic for proper interpolation.
    private val tunerPoints: List<SpeedPoint> by lazy {
        val distances = listOf(1.5, 2.0, 2.5, 3.0, 3.5)
        distances.map { d ->
            val vm = MuzzleSpeedMapping.stationaryMuzzleSpeed(d, dz, alpha, g)!!
            SpeedPoint(d, vm / 0.025)  // omega = vm / exit_speed_per_radian
        }
    }

    private val goalPosition3d = Vector3d(-1.48, 1.60, goalHeight)
    private val launchOffset = Vector3d(0.0, 0.0, launchHeight)

    private lateinit var dataStore: ShotDataStore
    private lateinit var tuner: AdaptiveTuner
    private lateinit var mapping: MuzzleSpeedMapping

    private val allContainsZone = object : MoveWhileShootSolver.ShootingZone {
        override fun contains(pos: Vector2d) = true
        override fun closestTo(pos: Vector2d) = pos
    }

    @BeforeEach
    fun setup() {
        dataStore = ShotDataStore()
        tunerPoints.forEach { dataStore.addPoint(it) }
        tuner = AdaptiveTuner(dataStore)

        mapping = MuzzleSpeedMapping.buildFromTuner(
            tunerPoints = tuner.getPointsSorted(),
            dz = dz,
            alpha = alpha,
            g = g
        )

        // Configure for test
        MoveWhileShootConfig.launchAngle = alpha
        MoveWhileShootConfig.gravity = g
        MoveWhileShootConfig.launchOffsetX = 0.0
        MoveWhileShootConfig.launchOffsetY = 0.0
        MoveWhileShootConfig.launchOffsetZ = launchHeight
        MoveWhileShootConfig.goalHeight = goalHeight
        MoveWhileShootConfig.enabled = false  // Stationary mode
        MoveWhileShootConfig.maxFlywheelSpeed = 628.0
        MoveWhileShootConfig.turretMaxAngle = PI / 2.0
    }

    /**
     * Stationary round-trip test: the solver should produce the same flywheel speed
     * as the adaptive tuner for stationary shots at tuner data point distances.
     *
     * The mapping is built FROM tuner data using kinematics, and the solver uses
     * the same kinematics to compute required v_m, then reverse-lookups via the mapping.
     * This should round-trip exactly at data points.
     */
    @Test
    fun stationaryRoundTripAtDataPoints() {
        for (point in tunerPoints) {
            val distance = point.distance
            val expectedOmega = point.speed

            // Position the robot so it's exactly `distance` away from the goal (horizontally)
            // Robot at (goalX, goalY - distance) facing north
            val robotX = goalPosition3d.x
            val robotY = goalPosition3d.y - distance
            val heading = PI / 2.0  // Facing goal

            val solver = MoveWhileShootSolver(
                validShootingZone = allContainsZone,
                muzzleSpeedMapping = mapping,
                goalPosition3d = goalPosition3d,
                launchOffset = launchOffset,
                mecanumDynamics = MecanumDynamics(drivetrainParameters),
            )

            val state = MecanumState(
                pos = Pose2d(robotX, robotY, heading),
                vel = Pose2d(0.0, 0.0, 0.0)
            )

            val solution = solver.computeSolutionAtTime(0.0, state, 0.0)

            assertNotNull(solution, "Solution should exist at distance $distance m")
            assertEquals(
                expectedOmega,
                solution!!.flywheelSpeed,
                expectedOmega * 0.01,  // 1% tolerance
                "Flywheel speed should round-trip at distance $distance m"
            )
        }
    }

    /**
     * At interpolated distances, verify the solver's answer is self-consistent:
     * the solved omega maps to a vm that satisfies the kinematics.
     */
    @Test
    fun stationaryRoundTripAtInterpolatedDistances() {
        val testDistances = listOf(1.75, 2.25, 2.75, 3.25)

        for (distance in testDistances) {
            val robotX = goalPosition3d.x
            val robotY = goalPosition3d.y - distance
            val heading = PI / 2.0

            val solver = MoveWhileShootSolver(
                validShootingZone = allContainsZone,
                muzzleSpeedMapping = mapping,
                goalPosition3d = goalPosition3d,
                launchOffset = launchOffset,
                mecanumDynamics = MecanumDynamics(drivetrainParameters),
            )

            val state = MecanumState(
                pos = Pose2d(robotX, robotY, heading),
                vel = Pose2d(0.0, 0.0, 0.0)
            )

            val solution = solver.computeSolutionAtTime(0.0, state, 0.0)
            assertNotNull(solution, "Solution should exist at distance $distance m")

            // Verify self-consistency: omega → vm → kinematics should reach goal
            val omega = solution!!.flywheelSpeed
            val vm = mapping.omegaToMuzzle(omega)
            val expectedVm = MuzzleSpeedMapping.stationaryMuzzleSpeed(distance, dz, alpha, g)!!

            assertEquals(
                expectedVm,
                vm,
                expectedVm * 0.05,  // 5% tolerance for interpolation paths
                "Muzzle speed from solved omega should be close to kinematic requirement at $distance m"
            )

            // Omega should be within the range of the tuner data
            assertTrue(
                omega > 0 && omega < 628.0,
                "Flywheel speed $omega should be in valid range at distance $distance m"
            )
        }
    }

    /**
     * A moving robot should produce a different flywheel speed than a stationary one.
     */
    @Test
    fun movingRobotAngleDivergesFromStationary() {
        val distance = 2.0
        val robotX = goalPosition3d.x
        val robotY = goalPosition3d.y - distance
        val heading = PI / 2.0

        val solver = MoveWhileShootSolver(
            validShootingZone = allContainsZone,
            muzzleSpeedMapping = mapping,
            goalPosition3d = goalPosition3d,
            launchOffset = launchOffset,
            mecanumDynamics = MecanumDynamics(drivetrainParameters),
        )

        // Stationary solution
        val stationaryState = MecanumState(
            pos = Pose2d(robotX, robotY, heading),
            vel = Pose2d(0.0, 0.0, 0.0)
        )
        val stationarySolution = solver.computeSolutionAtTime(0.0, stationaryState, 0.0)
        assertNotNull(stationarySolution)

        // Moving solution — robot moving sideways at 0.5 m/s
        val movingState = MecanumState(
            pos = Pose2d(robotX, robotY, heading),
            vel = Pose2d(0.5, 0.0, 0.0)
        )
        val movingSolution = solver.computeSolutionAtTime(0.0, movingState, 0.0)
        assertNotNull(movingSolution)

        println("stationary=${stationarySolution}")
        println("moving=${movingSolution}")

        // Turret angle should differ significantly due to velocity composition
        // Perpendicular robot motion requires the turret to lead the target
        assertTrue(
            abs(stationarySolution!!.turretAngle - movingSolution!!.turretAngle) > 0.01,
            "Moving robot should produce different turret angle " +
                "(stationary=${stationarySolution.turretAngle}, moving=${movingSolution.turretAngle})"
        )

        // Flywheel speed may differ slightly (vertical kinematics dominate)
        // but the time of flight should be different
        assertNotEquals(
            stationarySolution.timeOfFlight,
            movingSolution.timeOfFlight,
            0.001,
            "Moving robot should have different time of flight"
        )
    }

    /**
     * A moving robot should produce a different flywheel speed than a stationary one.
     */
    @Test
    fun movingRobotFlywheelDivergesFromStationary() {
        val distance = 2.0
        val robotX = goalPosition3d.x
        val robotY = goalPosition3d.y - distance
        val heading = PI / 2.0

        val solver = MoveWhileShootSolver(
            validShootingZone = allContainsZone,
            muzzleSpeedMapping = mapping,
            goalPosition3d = goalPosition3d,
            launchOffset = launchOffset,
            mecanumDynamics = MecanumDynamics(drivetrainParameters),
        )

        // Stationary solution
        val stationaryState = MecanumState(
            pos = Pose2d(robotX, robotY, heading),
            vel = Pose2d(0.0, 0.0, 0.0)
        )
        val stationarySolution = solver.computeSolutionAtTime(0.0, stationaryState, 0.0)
        assertNotNull(stationarySolution)

        // Moving solution — robot moving backwards at 0.5 m/s
        val movingState = MecanumState(
            pos = Pose2d(robotX, robotY, heading),
            vel = Pose2d(0.0, -0.5, 0.0)
        )
        val movingSolution = solver.computeSolutionAtTime(0.0, movingState, 0.0)
        assertNotNull(movingSolution)

        println("stationary=${stationarySolution}")
        println("moving=${movingSolution}")

        // Flywheel should differ significantly due to velocity composition
        // Perpendicular robot motion requires the turret to lead the target
        assertTrue(
            abs(stationarySolution!!.flywheelSpeed - movingSolution!!.flywheelSpeed) > 0.01,
            "Moving robot should produce different turret angle " +
                    "(stationary=${stationarySolution.flywheelSpeed}, moving=${movingSolution.flywheelSpeed})"
        )

        // Turret angle may differ slightly (vertical kinematics dominate)
        // but the time of flight should be different
        assertEquals(
            stationarySolution.turretAngle,
            movingSolution.turretAngle,
            0.001,
            "Moving robot should have similar angles"
        )
    }

    /**
     * When the goal is below the launch height and the angle makes it infeasible,
     * the solver should return null.
     */
    @Test
    fun infeasibleGeometryReturnsNull() {
        // Goal well below launch height with high launch angle
        val lowGoal = Vector3d(-1.48, 1.60, 0.0)  // ground level
        val highLaunch = Vector3d(0.0, 0.0, 2.0)   // launch from 2m high

        // With 45 degree launch angle and goal below, at short distance the discriminant
        // 2*(d_h*tan(alpha) - dz)/g could go negative
        // dz = 0.0 - 2.0 = -2.0, for d_h = 0.5: 2*(0.5*1.0 - (-2.0))/9.81 = 2*2.5/9.81 > 0
        // Actually this is still feasible. Let's use a very steep angle where dz > d_h*tan(alpha)
        MoveWhileShootConfig.launchAngle = 0.1  // ~6 degrees, very flat

        val steepMapping = MuzzleSpeedMapping.buildFromTuner(
            tunerPoints = tuner.getPointsSorted(),
            dz = lowGoal.z - highLaunch.z,  // -2.0
            alpha = 0.1,
            g = g
        )

        // At distance 0.1m with flat angle: d_h*tan(0.1) = 0.1*0.1003 = 0.01
        // dz = -2.0, so d_h*tan(alpha) - dz = 0.01 + 2.0 = 2.01 > 0 ... still feasible

        // Actually for infeasible: we need dz > d_h*tan(alpha)
        // With alpha = 80 degrees (steep), tan(80) = 5.67
        // dz = goalZ - launchZ needs to be > d_h * tan(alpha)
        // Let's make goalZ very high
        val veryHighGoal = Vector3d(-1.48, 1.60, 100.0)
        val normalLaunch = Vector3d(0.0, 0.0, 0.12)
        MoveWhileShootConfig.launchAngle = alpha  // 45 degrees

        // dz = 100 - 0.12 = 99.88, d_h ~= 2m, d_h*tan(45) = 2.0
        // discriminant = 2*(2.0 - 99.88)/9.81 < 0 → infeasible
        val infeasibleMapping = MuzzleSpeedMapping.buildFromTuner(
            tunerPoints = tuner.getPointsSorted(),
            dz = veryHighGoal.z - normalLaunch.z,
            alpha = alpha,
            g = g
        )

        val solver = MoveWhileShootSolver(
            validShootingZone = allContainsZone,
            muzzleSpeedMapping = infeasibleMapping,
            goalPosition3d = veryHighGoal,
            launchOffset = normalLaunch,
            mecanumDynamics = MecanumDynamics(drivetrainParameters),
        )

        val state = MecanumState(
            pos = Pose2d(goalPosition3d.x, goalPosition3d.y - 2.0, PI / 2.0),
            vel = Pose2d(0.0, 0.0, 0.0)
        )

        val solution = solver.computeSolutionAtTime(0.0, state, 0.0)
        assertNull(solution, "Should return null for infeasible geometry (goal too high)")
    }

    /**
     * Verify that the time of flight is physically reasonable.
     */
    @Test
    fun timeOfFlightIsReasonable() {
        val distance = 2.0
        val robotX = goalPosition3d.x
        val robotY = goalPosition3d.y - distance
        val heading = PI / 2.0

        val solver = MoveWhileShootSolver(
            validShootingZone = allContainsZone,
            muzzleSpeedMapping = mapping,
            goalPosition3d = goalPosition3d,
            launchOffset = launchOffset,
            mecanumDynamics = MecanumDynamics(drivetrainParameters),
        )

        val state = MecanumState(
            pos = Pose2d(robotX, robotY, heading),
            vel = Pose2d(0.0, 0.0, 0.0)
        )

        val solution = solver.computeSolutionAtTime(0.0, state, 0.0)
        assertNotNull(solution)

        // Time of flight for 2m at 45 degrees should be ~0.5-1.5 seconds
        assertTrue(
            solution!!.timeOfFlight in 0.1..3.0,
            "Time of flight ${solution.timeOfFlight}s should be reasonable for 2m shot"
        )
    }

    /**
     * Verify MuzzleSpeedMapping forward and inverse are consistent.
     */
    @Test
    fun muzzleSpeedMappingRoundTrip() {
        // Test forward then inverse
        for (point in tunerPoints) {
            val vm = mapping.omegaToMuzzle(point.speed)
            val omegaBack = mapping.muzzleToOmega(vm)
            assertEquals(
                point.speed,
                omegaBack,
                point.speed * 0.001,  // Very tight tolerance at data points
                "omega -> vm -> omega should round-trip at omega = ${point.speed}"
            )
        }
    }

    /**
     * Turret angle should point toward goal for a stationary robot.
     */
    @Test
    fun turretAnglePointsTowardGoal() {
        val distance = 2.0
        // Robot directly south of goal, facing east
        val robotX = goalPosition3d.x
        val robotY = goalPosition3d.y - distance
        val heading = 0.0  // Facing east (+x)

        val solver = MoveWhileShootSolver(
            validShootingZone = allContainsZone,
            muzzleSpeedMapping = mapping,
            goalPosition3d = goalPosition3d,
            launchOffset = launchOffset,
            mecanumDynamics = MecanumDynamics(drivetrainParameters),
        )

        val state = MecanumState(
            pos = Pose2d(robotX, robotY, heading),
            vel = Pose2d(0.0, 0.0, 0.0)
        )

        val solution = solver.computeSolutionAtTime(0.0, state, 0.0)
        assertNotNull(solution)

        // Goal is directly north (+y in field frame), robot faces east
        // In robot frame, goal is at 90 degrees (left)
        assertEquals(
            PI / 2.0,
            solution!!.turretAngle,
            0.05,
            "Turret should point 90 deg left when robot faces east and goal is north"
        )
    }
}
