package sigmacorns.control.aim

import org.joml.Matrix2d
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumDynamics
import sigmacorns.sim.MecanumState
import sigmacorns.tuning.SpeedPoint
import kotlin.math.*

/**
 * Maps between flywheel angular velocity (ω) and ball muzzle speed (v_m)
 * using empirical tuner data and 3D kinematics.
 *
 * For each tuner data point (distance, ω), we compute the muzzle speed
 * required by stationary 3D projectile kinematics, giving us calibrated
 * (ω, v_m) pairs. Linear interpolation provides the continuous mapping.
 */
class MuzzleSpeedMapping private constructor(
    private val byOmega: List<MappingPoint>,     // sorted by omega ascending
    private val byMuzzle: List<MappingPoint>     // sorted by muzzleSpeed ascending
) {
    data class MappingPoint(val omega: Double, val muzzleSpeed: Double)

    fun omegaToMuzzle(omega: Double): Double {
        if (byOmega.isEmpty()) return omega * 0.025  // fallback
        if (byOmega.size == 1) return byOmega[0].muzzleSpeed

        val lower = byOmega.lastOrNull { it.omega <= omega }
        val upper = byOmega.firstOrNull { it.omega > omega }

        return when {
            lower != null && upper != null -> {
                val t = (omega - lower.omega) / (upper.omega - lower.omega)
                lower.muzzleSpeed + t * (upper.muzzleSpeed - lower.muzzleSpeed)
            }
            lower != null -> {
                val last = byOmega[byOmega.size - 1]
                val secondLast = byOmega[byOmega.size - 2]
                val slope = (last.muzzleSpeed - secondLast.muzzleSpeed) / (last.omega - secondLast.omega)
                last.muzzleSpeed + slope * (omega - last.omega)
            }
            upper != null -> {
                val first = byOmega[0]
                val second = byOmega[1]
                val slope = (second.muzzleSpeed - first.muzzleSpeed) / (second.omega - first.omega)
                first.muzzleSpeed + slope * (omega - first.omega)
            }
            else -> omega * 0.025
        }
    }

    fun muzzleToOmega(vm: Double): Double {
        if (byMuzzle.isEmpty()) return vm / 0.025  // fallback
        if (byMuzzle.size == 1) return byMuzzle[0].omega

        val lower = byMuzzle.lastOrNull { it.muzzleSpeed <= vm }
        val upper = byMuzzle.firstOrNull { it.muzzleSpeed > vm }

        return when {
            lower != null && upper != null -> {
                val t = (vm - lower.muzzleSpeed) / (upper.muzzleSpeed - lower.muzzleSpeed)
                lower.omega + t * (upper.omega - lower.omega)
            }
            lower != null -> {
                val last = byMuzzle[byMuzzle.size - 1]
                val secondLast = byMuzzle[byMuzzle.size - 2]
                val slope = (last.omega - secondLast.omega) / (last.muzzleSpeed - secondLast.muzzleSpeed)
                last.omega + slope * (vm - last.muzzleSpeed)
            }
            upper != null -> {
                val first = byMuzzle[0]
                val second = byMuzzle[1]
                val slope = (second.omega - first.omega) / (second.muzzleSpeed - first.muzzleSpeed)
                first.omega + slope * (vm - first.muzzleSpeed)
            }
            else -> vm / 0.025
        }
    }

    companion object {
        /**
         * Build mapping from adaptive tuner data points and 3D geometry.
         *
         * For each (distance, omega) pair from the tuner, compute the muzzle speed
         * required for a stationary shot at that horizontal distance using:
         *   t_air = sqrt(2 * (d_h * tan(alpha) - dz) / g)
         *   v_m = d_h / (t_air * cos(alpha))
         *
         * @param tunerPoints sorted (distance, omega) pairs from AdaptiveTuner
         * @param dz height difference: goalZ - launchZ
         * @param alpha launch elevation angle (rad)
         * @param g gravity (m/s²)
         */
        fun buildFromTuner(
            tunerPoints: List<SpeedPoint>,
            dz: Double,
            alpha: Double,
            g: Double
        ): MuzzleSpeedMapping {
            val mappingPoints = tunerPoints.mapNotNull { point ->
                val vm = stationaryMuzzleSpeed(point.distance, dz, alpha, g) ?: return@mapNotNull null
                MappingPoint(omega = point.speed, muzzleSpeed = vm)
            }

            return MuzzleSpeedMapping(
                byOmega = mappingPoints.sortedBy { it.omega },
                byMuzzle = mappingPoints.sortedBy { it.muzzleSpeed }
            )
        }

        /**
         * Compute required muzzle speed for a stationary shot at horizontal distance d_h.
         * Returns null if the geometry is infeasible (negative discriminant).
         */
        fun stationaryMuzzleSpeed(dh: Double, dz: Double, alpha: Double, g: Double): Double? {
            val discriminant = 2.0 * (dh * tan(alpha) - dz) / g
            if (discriminant <= 0.0) return null
            val tAir = sqrt(discriminant)
            return dh / (tAir * cos(alpha))
        }

        /**
         * Compute stationary time of flight.
         */
        fun stationaryTimeOfFlight(dh: Double, dz: Double, alpha: Double, g: Double): Double? {
            val discriminant = 2.0 * (dh * tan(alpha) - dz) / g
            if (discriminant <= 0.0) return null
            return sqrt(discriminant)
        }
    }
}

/**
 * Pre-computed trajectory: integrates the robot state once, stores samples
 * at regular intervals, and provides O(1) lookups via linear interpolation.
 */
class PredictedTrajectory(
    private val samples: List<MecanumState>,
    private val sampleDt: Double,
    private val totalTime: Double
) {
    /** Sample the trajectory at time t via linear interpolation. */
    fun sampleAt(t: Double): MecanumState {
        if (t <= 0.0) return samples.first()
        if (t >= totalTime) return samples.last()

        val index = t / sampleDt
        val lo = index.toInt().coerceIn(0, samples.size - 2)
        val frac = index - lo

        val a = samples[lo]
        val b = samples[lo + 1]

        return MecanumState(
            pos = Pose2d(
                a.pos.v.x + frac * (b.pos.v.x - a.pos.v.x),
                a.pos.v.y + frac * (b.pos.v.y - a.pos.v.y),
                a.pos.rot + frac * (b.pos.rot - a.pos.rot)
            ),
            vel = Pose2d(
                a.vel.v.x + frac * (b.vel.v.x - a.vel.v.x),
                a.vel.v.y + frac * (b.vel.v.y - a.vel.v.y),
                a.vel.rot + frac * (b.vel.rot - a.vel.rot)
            )
        )
    }

    companion object {
        /**
         * Build a predicted trajectory by integrating once with fine timestep,
         * then sampling at coarser intervals.
         *
         * @param dynamics physics model
         * @param initialState starting state
         * @param motorPowers constant motor powers during prediction
         * @param horizon total prediction time (s)
         * @param integrationDt fine RK4 timestep (s)
         * @param sampleDt interval between stored samples (s)
         */
        fun build(
            dynamics: MecanumDynamics,
            initialState: MecanumState,
            motorPowers: DoubleArray,
            horizon: Double,
            integrationDt: Double,
            sampleDt: Double
        ): PredictedTrajectory {
            val numSamples = (horizon / sampleDt).toInt() + 1
            val samples = ArrayList<MecanumState>(numSamples)
            samples.add(initialState)

            var current = initialState
            for (i in 1 until numSamples) {
                current = dynamics.integrate(sampleDt, integrationDt, motorPowers, current)
                samples.add(current)
            }

            return PredictedTrajectory(samples, sampleDt, horizon)
        }
    }
}

/**
 * Solves for optimal shot timing when robot is moving, using 3D ballistic kinematics.
 *
 * Key physics:
 * - Ball exits at fixed elevation angle α with muzzle speed v_m
 * - Ball inherits robot translational velocity at launch
 * - 3D projectile motion under gravity determines time of flight
 * - Muzzle speed maps to flywheel speed via calibrated MuzzleSpeedMapping
 *
 * Performance: trajectory is pre-integrated once per solve() call, then sampled
 * O(1) for each binary search iteration. This avoids redundant RK4 integration.
 */
class MoveWhileShootSolver(
    private val validShootingZone: ShootingZone,
    val muzzleSpeedMapping: MuzzleSpeedMapping,
    private val goalPosition3d: Vector3d,
    private val launchOffset: Vector3d,
    private val mecanumDynamics: MecanumDynamics,
    private val spinupTimeEstimator: (Double, Double) -> Double = { current, target -> abs(target - current) / 1000.0 }
) {
    private val rotMatrix = Matrix2d()

    interface ShootingZone {
        fun contains(pos: Vector2d): Boolean
        fun closestTo(pos: Vector2d): Vector2d
    }

    data class ShotSolution(
        val timeToShot: Double,
        val turretAngle: Double,
        val flywheelSpeed: Double,
        val predictedPosition: Vector2d,
        val predictedVelocity: Vector2d,
        val predictedHeading: Double,
        val inShootingZone: Boolean,
        val feasible: Boolean,
        val timeOfFlight: Double
    )

    fun solve(
        currentState: MecanumState,
        motorPowers: DoubleArray,
        currentFlywheelSpeed: Double = 0.0
    ): ShotSolution? {
        val cfg = MoveWhileShootConfig

        val robotSpeed = currentState.vel.v.length()
        if (!cfg.enabled || robotSpeed < cfg.minRobotSpeed) {
            val stationaryState = MecanumState(
                pos = currentState.pos,
                vel = Pose2d(0.0, 0.0, 0.0)
            )
            return computeSolutionAtTime(0.0, stationaryState, currentFlywheelSpeed)
        }

        // Pre-compute trajectory ONCE for the entire solve
        val trajectory = PredictedTrajectory.build(
            dynamics = mecanumDynamics,
            initialState = currentState,
            motorPowers = motorPowers,
            horizon = cfg.maxHorizon,
            integrationDt = cfg.predictionDt,
            sampleDt = cfg.predictionSampleDt
        )

        // Binary search for minimum feasible T
        var lo = 0.0
        var hi = cfg.maxHorizon

        val maxSolution = computeSolutionAtTime(hi, trajectory.sampleAt(hi), currentFlywheelSpeed)
        if (maxSolution == null || !maxSolution.feasible) {
            return maxSolution
        }

        repeat(cfg.maxBinarySearchIterations) {
            if (hi - lo < cfg.binarySearchTolerance) return@repeat

            val mid = (lo + hi) / 2
            val solution = computeSolutionAtTime(mid, trajectory.sampleAt(mid), currentFlywheelSpeed)

            if (solution != null && solution.feasible) {
                hi = mid
            } else {
                lo = mid
            }
        }

        // Iterative refinement
        var T = hi
        var lastFlywheelSpeed = Double.MAX_VALUE

        repeat(cfg.maxRefinementIterations) {
            val solution = computeSolutionAtTime(T, trajectory.sampleAt(T), currentFlywheelSpeed)
                ?: return null

            if (abs(solution.flywheelSpeed - lastFlywheelSpeed) < cfg.flywheelSpeedTolerance) {
                return solution
            }

            lastFlywheelSpeed = solution.flywheelSpeed

            val spinupTime = spinupTimeEstimator(currentFlywheelSpeed, solution.flywheelSpeed)
            val minT = spinupTime + cfg.transferTime + cfg.contactTime
            T = maxOf(T, minT)
        }

        return computeSolutionAtTime(T, trajectory.sampleAt(T), currentFlywheelSpeed)
    }

    /**
     * Compute shot solution using 3D ballistic kinematics.
     *
     * Steps:
     * 1. Compute field-frame 3D launch position from robot pose + launchOffset
     * 2. Compute 3D displacement to goal
     * 3. Solve for time of flight (closed-form for stationary, bisection for moving)
     * 4. Compute muzzle speed and turret angle from solved t
     * 5. Convert muzzle speed to flywheel ω via MuzzleSpeedMapping
     */
    fun computeSolutionAtTime(
        T: Double,
        predictedState: MecanumState,
        currentFlywheelSpeed: Double
    ): ShotSolution? {
        val cfg = MoveWhileShootConfig
        val alpha = cfg.launchAngle
        val g = cfg.gravity

        val predictedPos = predictedState.pos.v
        val predictedVel = predictedState.vel.v
        val predictedHeading = predictedState.pos.rot

        // Check shooting zone (2D)
        val inZone = validShootingZone.contains(predictedPos)
        val effectivePos = if (inZone) predictedPos else validShootingZone.closestTo(predictedPos)

        // Step 1: Field-frame 3D launch position
        val cosH = cos(predictedHeading)
        val sinH = sin(predictedHeading)
        val launchFieldX = effectivePos.x + launchOffset.x * cosH - launchOffset.y * sinH
        val launchFieldY = effectivePos.y + launchOffset.x * sinH + launchOffset.y * cosH
        val launchFieldZ = launchOffset.z

        // Step 2: 3D displacement to goal
        val dx = goalPosition3d.x - launchFieldX
        val dy = goalPosition3d.y - launchFieldY
        val dz = goalPosition3d.z - launchFieldZ
        val dh = hypot(dx, dy)

        if (dh < 0.01) return null  // Too close

        // Robot velocity components
        val vrx = predictedVel.x
        val vry = predictedVel.y

        // Step 3: Solve for time of flight
        val tStationary = MuzzleSpeedMapping.stationaryTimeOfFlight(dh, dz, alpha, g)
            ?: return null  // Infeasible geometry

        val isMoving = hypot(vrx, vry) > 0.001
        val tFlight: Double

        if (!isMoving) {
            tFlight = tStationary
        } else {
            // Solve f(t) = 0 numerically via bisection
            // f(t) = ||(dx/t - vrx, dy/t - vry)|| - cos(α)/sin(α) * (dz + ½gt²)/t
            val tanAlphaInv = cos(alpha) / sin(alpha)

            fun f(t: Double): Double {
                val relX = dx / t - vrx
                val relY = dy / t - vry
                val relMag = hypot(relX, relY)
                val vertTerm = tanAlphaInv * (dz + 0.5 * g * t * t) / t
                return relMag - vertTerm
            }

            // Bisection bounds: search around t_stationary
            var tLo = tStationary * 0.1
            var tHi = tStationary * 5.0

            val fLo = f(tLo)
            val fHi = f(tHi)

            if (fLo * fHi > 0) {
                tFlight = tStationary
            } else {
                var lo2 = tLo
                var hi2 = tHi
                repeat(cfg.numericSolverIterations) {
                    val mid = (lo2 + hi2) / 2.0
                    if (f(mid) * f(lo2) <= 0) {
                        hi2 = mid
                    } else {
                        lo2 = mid
                    }
                }
                tFlight = (lo2 + hi2) / 2.0
            }
        }

        // Step 4: Compute muzzle speed from vertical constraint
        // v_m = (dz + ½gt²) / (sin(α)·t)
        val vm = (dz + 0.5 * g * tFlight * tFlight) / (sin(alpha) * tFlight)
        if (vm <= 0.0) return null  // Negative muzzle speed is infeasible

        // Step 5: Convert v_m → flywheel ω
        val flywheelOmega = muzzleSpeedMapping.muzzleToOmega(vm)

        // Step 6: Compute turret angle θ
        // Horizontal relative velocity (subtracting robot velocity, in field frame)
        val relVelX = dx / tFlight - vrx
        val relVelY = dy / tFlight - vry

        // Rotate into robot frame
        rotMatrix.identity().rotate(-predictedHeading)
        val relVelRobot = rotMatrix.transform(Vector2d(relVelX, relVelY))
        val turretAngle = atan2(relVelRobot.y, relVelRobot.x)

        // Check constraints
        val spinupTime = spinupTimeEstimator(currentFlywheelSpeed, flywheelOmega)
        val minTime = spinupTime + cfg.transferTime + cfg.contactTime

        val turretFeasible = abs(turretAngle) <= cfg.turretMaxAngle
        val flywheelFeasible = flywheelOmega <= cfg.maxFlywheelSpeed && flywheelOmega > 0
        val timingFeasible = T >= minTime

        val feasible = turretFeasible && flywheelFeasible && timingFeasible

        return ShotSolution(
            timeToShot = T,
            turretAngle = turretAngle,
            flywheelSpeed = flywheelOmega,
            predictedPosition = predictedPos,
            predictedVelocity = predictedVel,
            predictedHeading = predictedHeading,
            inShootingZone = inZone,
            feasible = feasible,
            timeOfFlight = tFlight
        )
    }
}
