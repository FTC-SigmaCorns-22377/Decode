package sigmacorns.control.aim

import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.constants.drivetrainParameters
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumDynamics
import sigmacorns.sim.MecanumState
import sigmacorns.tuning.AdaptiveTuner
import kotlin.math.abs

/**
 * High-level controller that integrates move-while-shoot solving with the aiming system.
 *
 * Uses [AdaptiveTuner] data to build a [MuzzleSpeedMapping] that calibrates the relationship
 * between flywheel angular velocity and ball muzzle speed via 3D kinematics.
 *
 * State prediction uses [MecanumDynamics] to integrate the robot's equations of motion
 * forward in time, accounting for motor torque curves, inertia, and friction.
 *
 * The controller caches solutions and only recalculates when:
 * - The cached solution becomes infeasible from current flywheel state
 * - The robot's predicted trajectory deviates too much from what was assumed
 */
class MoveWhileShootController(
    goalPosition3d: Vector3d,
    private val shootingZone: MoveWhileShootSolver.ShootingZone,
    private val adaptiveTuner: AdaptiveTuner,
    private val launchOffset: Vector3d = Vector3d(
        MoveWhileShootConfig.launchOffsetX,
        MoveWhileShootConfig.launchOffsetY,
        MoveWhileShootConfig.launchOffsetZ
    ),
    private val mecanumDynamics: MecanumDynamics = MecanumDynamics(drivetrainParameters),
    private val spinupTimeEstimator: (Double, Double) -> Double = { current, target ->
        abs(target - current) / 500.0
    }
) {
    private val goalPos3d = goalPosition3d
    private var muzzleMapping: MuzzleSpeedMapping
    private var solver: MoveWhileShootSolver

    private var cachedSolution: CachedSolution? = null

    private data class CachedSolution(
        val solution: MoveWhileShootSolver.ShotSolution,
        val solveTimeNanos: Long,
        val startingState: MecanumState,
        val motorPowers: DoubleArray,
        val predictedVelocityAtShot: Vector2d,
        val targetFlywheelSpeed: Double
    )

    data class ControlOutput(
        val turretAngle: Double,
        val flywheelSpeed: Double,
        val timeToShot: Double,
        val inShootingZone: Boolean,
        val readyToShoot: Boolean,
        val fromCache: Boolean
    )

    init {
        muzzleMapping = buildMapping()
        solver = MoveWhileShootSolver(
            validShootingZone = shootingZone,
            muzzleSpeedMapping = muzzleMapping,
            goalPosition3d = goalPos3d,
            launchOffset = launchOffset,
            mecanumDynamics = mecanumDynamics,
            spinupTimeEstimator = spinupTimeEstimator
        )
    }

    private fun buildMapping(): MuzzleSpeedMapping {
        val cfg = MoveWhileShootConfig
        val dz = goalPos3d.z - launchOffset.z
        return MuzzleSpeedMapping.buildFromTuner(
            tunerPoints = adaptiveTuner.getPointsSorted(),
            dz = dz,
            alpha = cfg.launchAngle,
            g = cfg.gravity
        )
    }

    /**
     * Rebuild the muzzle speed mapping from current tuner data.
     * Call this when tuner data changes (e.g., new calibration points added).
     */
    fun rebuildMapping() {
        muzzleMapping = buildMapping()
        solver = MoveWhileShootSolver(
            validShootingZone = shootingZone,
            muzzleSpeedMapping = muzzleMapping,
            goalPosition3d = goalPos3d,
            launchOffset = launchOffset,
            mecanumDynamics = mecanumDynamics,
            spinupTimeEstimator = spinupTimeEstimator
        )
        cachedSolution = null
    }

    fun update(
        robotState: MecanumState,
        motorPowers: DoubleArray,
        currentFlywheelSpeed: Double
    ): ControlOutput? {
        val cfg = MoveWhileShootConfig

        val cached = cachedSolution
        if (cached != null && isCacheValid(cached, robotState, currentFlywheelSpeed)) {
            val elapsedSeconds = (System.nanoTime() - cached.solveTimeNanos) / 1e9
            val updatedTimeToShot = (cached.solution.timeToShot - elapsedSeconds).coerceAtLeast(0.0)

            val spinupTime = spinupTimeEstimator(currentFlywheelSpeed, cached.targetFlywheelSpeed)
            val readyToShoot = cached.solution.feasible &&
                    cached.solution.inShootingZone &&
                    spinupTime <= cfg.transferTime

            return ControlOutput(
                turretAngle = cached.solution.turretAngle,
                flywheelSpeed = cached.targetFlywheelSpeed,
                timeToShot = updatedTimeToShot,
                inShootingZone = cached.solution.inShootingZone,
                readyToShoot = readyToShoot,
                fromCache = true
            )
        }

        val solution = solver.solve(
            currentState = robotState,
            motorPowers = motorPowers,
            currentFlywheelSpeed = currentFlywheelSpeed
        ) ?: run {
            cachedSolution = null
            return null
        }

        cachedSolution = CachedSolution(
            solution = solution,
            solveTimeNanos = System.nanoTime(),
            startingState = robotState,
            motorPowers = motorPowers.copyOf(),
            predictedVelocityAtShot = Vector2d(solution.predictedVelocity),
            targetFlywheelSpeed = solution.flywheelSpeed
        )

        val spinupTime = spinupTimeEstimator(currentFlywheelSpeed, solution.flywheelSpeed)
        val readyToShoot = solution.feasible &&
                solution.inShootingZone &&
                spinupTime <= cfg.transferTime

        return ControlOutput(
            turretAngle = solution.turretAngle,
            flywheelSpeed = solution.flywheelSpeed,
            timeToShot = solution.timeToShot,
            inShootingZone = solution.inShootingZone,
            readyToShoot = readyToShoot,
            fromCache = false
        )
    }

    private fun isCacheValid(
        cached: CachedSolution,
        currentState: MecanumState,
        currentFlywheelSpeed: Double
    ): Boolean {
        val cfg = MoveWhileShootConfig

        val elapsedSeconds = (System.nanoTime() - cached.solveTimeNanos) / 1e9
        val remainingTime = cached.solution.timeToShot - elapsedSeconds

        if (remainingTime <= 0) return false

        val spinupTime = spinupTimeEstimator(currentFlywheelSpeed, cached.targetFlywheelSpeed)
        val minRequiredTime = spinupTime + cfg.transferTime + cfg.contactTime

        if (remainingTime < minRequiredTime) return false

        val expectedStateNow = mecanumDynamics.integrate(
            elapsedSeconds,
            cfg.predictionDt,
            cached.motorPowers,
            cached.startingState
        )

        val positionError = currentState.pos.v.distance(expectedStateNow.pos.v)
        if (positionError > cfg.trajectoryPositionTolerance) return false

        val velocityError = currentState.vel.v.distance(expectedStateNow.vel.v)
        if (velocityError > cfg.trajectoryVelocityTolerance) return false

        return true
    }

    fun invalidateCache() {
        cachedSolution = null
    }

    fun getStaticSolution(
        robotPose: Pose2d,
        currentFlywheelSpeed: Double
    ): ControlOutput? {
        val staticState = MecanumState(
            vel = Pose2d(0.0, 0.0, 0.0),
            pos = robotPose
        )

        val zeroMotorPowers = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

        val solution = solver.solve(
            currentState = staticState,
            motorPowers = zeroMotorPowers,
            currentFlywheelSpeed = currentFlywheelSpeed
        ) ?: return null

        val spinupTime = spinupTimeEstimator(currentFlywheelSpeed, solution.flywheelSpeed)

        return ControlOutput(
            turretAngle = solution.turretAngle,
            flywheelSpeed = solution.flywheelSpeed,
            timeToShot = solution.timeToShot,
            inShootingZone = solution.inShootingZone,
            readyToShoot = solution.feasible && solution.inShootingZone &&
                    spinupTime <= MoveWhileShootConfig.transferTime,
            fromCache = false
        )
    }
}
