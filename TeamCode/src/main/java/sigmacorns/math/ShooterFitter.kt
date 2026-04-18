package sigmacorns.math

import kotlin.math.*

/**
 * Shooter velocity surface fitting and inverse lookup.
 *
 * Fits empirical polynomial surfaces:
 *   v(RPM, hoodAngle)     — launch speed (m/s)
 *   theta(RPM, hoodAngle) — actual launch angle (rad)
 *
 * from a table of (RPM, hoodAngle, distance) data points,
 * using projectile motion as the only physical constraint.
 *
 * Then provides inverse functions:
 *   (vx, vy)   -> (RPM, hoodAngle)
 *   distance   -> (RPM, hoodAngle)
 */

/**
 * A single calibration data point: (RPM, hood angle setting in degrees, distance in metres).
 */
data class ShooterDataPoint(
    val rpm: Double,
    val hoodAngle: Double,
    val distance: Double
)

/**
 * Result of fitting the polynomial surfaces.
 */
data class ShooterSurfaceCoeffs(
    val vCoeffs: DoubleArray,
    val thetaCoeffs: DoubleArray
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is ShooterSurfaceCoeffs) return false
        return vCoeffs.contentEquals(other.vCoeffs) && thetaCoeffs.contentEquals(other.thetaCoeffs)
    }

    override fun hashCode(): Int = 31 * vCoeffs.contentHashCode() + thetaCoeffs.contentHashCode()
}

/**
 * Result of an inverse lookup: the shooter parameters needed to achieve a desired outcome.
 */
data class ShooterParams(
    val rpm: Double,
    val hoodAngle: Double
)

/**
 * Fits polynomial surfaces to empirical shooter data and provides inverse lookups.
 *
 * @param goalHeight goal height above floor (m)
 * @param launchHeight shooter exit height above floor (m)
 * @param g gravitational acceleration (m/s^2)
 * @param rpmNorm normalisation constant for RPM (default 5000)
 * @param hoodNorm normalisation constant for hood angle (default 45)
 */
class ShooterFitter(
    private val goalHeight: Double,
    private val launchHeight: Double,
    private val g: Double = 9.81,
    private val rpmNorm: Double = 5000.0,
    private val hoodNorm: Double = 45.0
) {
    private val h: Double = goalHeight - launchHeight

    companion object {
        /** Number of coefficients for a degree-2 polynomial in 2 variables. */
        const val NUM_COEFFS = 6

        /** Total parameter count (v coefficients + theta coefficients). */
        const val TOTAL_PARAMS = NUM_COEFFS * 2
    }

    // ------------------------------------------------------------------
    // Polynomial features
    // ------------------------------------------------------------------

    /**
     * Build degree-2 polynomial feature vector [1, r, h, r*h, r^2, h^2]
     * where r = rpm/rpmNorm and h = hood/hoodNorm.
     */
    private fun polyFeatures(rpm: Double, hood: Double): DoubleArray {
        val r = rpm / rpmNorm
        val h = hood / hoodNorm
        return doubleArrayOf(1.0, r, h, r * h, r * r, h * h)
    }

    private fun dot(a: DoubleArray, b: DoubleArray): Double {
        var sum = 0.0
        for (i in a.indices) sum += a[i] * b[i]
        return sum
    }

    /** Predict launch speed (m/s) from RPM and hood angle setting. */
    fun predictSpeed(rpm: Double, hood: Double, coeffs: ShooterSurfaceCoeffs): Double =
        dot(polyFeatures(rpm, hood), coeffs.vCoeffs)

    /** Predict actual launch angle (rad) from RPM and hood angle setting. */
    fun predictAngle(rpm: Double, hood: Double, coeffs: ShooterSurfaceCoeffs): Double =
        dot(polyFeatures(rpm, hood), coeffs.thetaCoeffs)

    // ------------------------------------------------------------------
    // Projectile distance prediction
    // ------------------------------------------------------------------

    /**
     * Predict horizontal distance for a projectile launched at speed [v], angle [thetaRad],
     * that must reach height [h] (goal height minus launch height).
     */
    fun predictedDistance(v: Double, thetaRad: Double): Double {
        if (v <= 0.0 || thetaRad <= 0.0 || thetaRad >= PI / 2.0) return 0.0

        val vy = v * sin(thetaRad)
        val vx = v * cos(thetaRad)

        val discriminant = vy * vy - 2.0 * g * h
        if (discriminant < 0.0) return 0.0

        var t = (vy - sqrt(discriminant)) / g
        if (t <= 0.0) t = (vy + sqrt(discriminant)) / g

        return vx * t
    }

    // ------------------------------------------------------------------
    // Fitting (differential evolution)
    // ------------------------------------------------------------------

    /**
     * Cost function: sum of squared distance errors plus regularisation penalties.
     */
    private fun fitCost(params: DoubleArray, data: List<ShooterDataPoint>): Double {
        val coeffs = ShooterSurfaceCoeffs(
            vCoeffs = params.copyOfRange(0, NUM_COEFFS),
            thetaCoeffs = params.copyOfRange(NUM_COEFFS, TOTAL_PARAMS)
        )

        var totalError = 0.0
        for (point in data) {
            val v = predictSpeed(point.rpm, point.hoodAngle, coeffs)
            val theta = predictAngle(point.rpm, point.hoodAngle, coeffs)
            val dPred = predictedDistance(v, theta)
            val err = dPred - point.distance
            totalError += err * err

            // Penalise non-physical values
            if (v < 0.0) totalError += 1000.0 * v * v
            if (theta < 0.05 || theta > 1.4) totalError += 1000.0
        }

        return totalError
    }

    /**
     * Fit polynomial surfaces to the given data points using differential evolution.
     *
     * @param data calibration data points
     * @param populationSize DE population size (default 15 * TOTAL_PARAMS)
     * @param maxGenerations maximum number of generations
     * @param mutationFactor DE mutation scale factor F (default 0.8)
     * @param crossoverRate DE crossover probability CR (default 0.9)
     * @param tolerance convergence tolerance on cost improvement
     * @param seed random seed for reproducibility
     * @return fitted surface coefficients
     */
    fun fit(
        data: List<ShooterDataPoint>,
        populationSize: Int = 15 * TOTAL_PARAMS,
        maxGenerations: Int = 5000,
        mutationFactor: Double = 0.8,
        crossoverRate: Double = 0.9,
        tolerance: Double = 1e-12,
        seed: Long = 42L
    ): ShooterSurfaceCoeffs {
        require(data.size >= 3) { "Need at least 3 data points to fit surfaces" }

        // Parameter bounds: [v coefficients | theta coefficients]
        val lowerBounds = doubleArrayOf(
            // v: intercept, r, h, r*h, r^2, h^2
            -5.0, 0.0, -5.0, -5.0, -5.0, -5.0,
            // theta: should produce angles in radians (~0.1 to 1.2)
            -1.0, -1.0, 0.0, -1.0, -1.0, -1.0
        )
        val upperBounds = doubleArrayOf(
            5.0, 20.0, 5.0, 5.0, 5.0, 5.0,
            1.0, 1.0, 2.0, 1.0, 1.0, 1.0
        )

        return differentialEvolution(
            costFn = { params -> fitCost(params, data) },
            lowerBounds = lowerBounds,
            upperBounds = upperBounds,
            populationSize = populationSize,
            maxGenerations = maxGenerations,
            mutationFactor = mutationFactor,
            crossoverRate = crossoverRate,
            tolerance = tolerance,
            seed = seed
        )
    }

    // ------------------------------------------------------------------
    // Differential evolution optimiser
    // ------------------------------------------------------------------

    private fun differentialEvolution(
        costFn: (DoubleArray) -> Double,
        lowerBounds: DoubleArray,
        upperBounds: DoubleArray,
        populationSize: Int,
        maxGenerations: Int,
        mutationFactor: Double,
        crossoverRate: Double,
        tolerance: Double,
        seed: Long
    ): ShooterSurfaceCoeffs {
        val rng = java.util.Random(seed)
        val dim = lowerBounds.size

        // Initialise population uniformly within bounds
        val population = Array(populationSize) { DoubleArray(dim) { i ->
            lowerBounds[i] + rng.nextDouble() * (upperBounds[i] - lowerBounds[i])
        }}
        val costs = DoubleArray(populationSize) { costFn(population[it]) }

        var bestIdx = costs.indices.minByOrNull { costs[it] }!!
        var bestCost = costs[bestIdx]

        for (gen in 0 until maxGenerations) {
            val prevBestCost = bestCost

            for (i in 0 until populationSize) {
                // Pick 3 distinct random indices != i
                val indices = mutableListOf<Int>()
                while (indices.size < 3) {
                    val r = rng.nextInt(populationSize)
                    if (r != i && r !in indices) indices.add(r)
                }
                val (a, b, c) = indices

                // Mutation: donor = a + F * (b - c)
                val donor = DoubleArray(dim) { j ->
                    (population[a][j] + mutationFactor * (population[b][j] - population[c][j]))
                        .coerceIn(lowerBounds[j], upperBounds[j])
                }

                // Crossover
                val jRand = rng.nextInt(dim)
                val trial = DoubleArray(dim) { j ->
                    if (rng.nextDouble() < crossoverRate || j == jRand) donor[j]
                    else population[i][j]
                }

                // Selection
                val trialCost = costFn(trial)
                if (trialCost <= costs[i]) {
                    population[i] = trial
                    costs[i] = trialCost
                    if (trialCost < bestCost) {
                        bestCost = trialCost
                        bestIdx = i
                    }
                }
            }

            // Convergence check
            if (abs(bestCost - prevBestCost) < tolerance && gen > 100) break
        }

        val best = population[bestIdx]

        // Polish with Nelder-Mead
        val polished = nelderMead(costFn, best, lowerBounds, upperBounds)

        return ShooterSurfaceCoeffs(
            vCoeffs = polished.copyOfRange(0, NUM_COEFFS),
            thetaCoeffs = polished.copyOfRange(NUM_COEFFS, TOTAL_PARAMS)
        )
    }

    // ------------------------------------------------------------------
    // Nelder-Mead simplex (polishing step)
    // ------------------------------------------------------------------

    private fun nelderMead(
        costFn: (DoubleArray) -> Double,
        initial: DoubleArray,
        lowerBounds: DoubleArray,
        upperBounds: DoubleArray,
        maxIter: Int = 10000,
        tolerance: Double = 1e-14
    ): DoubleArray {
        val dim = initial.size
        val alpha = 1.0
        val gamma = 2.0
        val rho = 0.5
        val sigma = 0.5

        fun clamp(x: DoubleArray): DoubleArray =
            DoubleArray(dim) { i -> x[i].coerceIn(lowerBounds[i], upperBounds[i]) }

        // Build initial simplex
        val simplex = Array(dim + 1) { idx ->
            if (idx == 0) initial.copyOf()
            else DoubleArray(dim) { j ->
                if (j == idx - 1) initial[j] + 0.05 * (upperBounds[j] - lowerBounds[j])
                else initial[j]
            }.let(::clamp)
        }
        val fValues = DoubleArray(dim + 1) { costFn(simplex[it]) }

        for (iter in 0 until maxIter) {
            // Sort by cost
            val order = fValues.indices.sortedBy { fValues[it] }
            val sortedSimplex = order.map { simplex[it].copyOf() }
            val sortedF = order.map { fValues[it] }.toDoubleArray()
            for (i in sortedSimplex.indices) {
                simplex[i] = sortedSimplex[i]
                fValues[i] = sortedF[i]
            }

            // Convergence check
            if (fValues[dim] - fValues[0] < tolerance) break

            // Centroid (excluding worst)
            val centroid = DoubleArray(dim)
            for (i in 0 until dim) {
                for (j in 0 until dim) centroid[j] += simplex[i][j]
            }
            for (j in 0 until dim) centroid[j] /= dim.toDouble()

            // Reflection
            val reflected = clamp(DoubleArray(dim) { centroid[it] + alpha * (centroid[it] - simplex[dim][it]) })
            val fReflected = costFn(reflected)

            if (fReflected < fValues[0]) {
                // Expansion
                val expanded = clamp(DoubleArray(dim) { centroid[it] + gamma * (reflected[it] - centroid[it]) })
                val fExpanded = costFn(expanded)
                if (fExpanded < fReflected) {
                    simplex[dim] = expanded; fValues[dim] = fExpanded
                } else {
                    simplex[dim] = reflected; fValues[dim] = fReflected
                }
            } else if (fReflected < fValues[dim - 1]) {
                simplex[dim] = reflected; fValues[dim] = fReflected
            } else {
                // Contraction
                val contracted = clamp(DoubleArray(dim) { centroid[it] + rho * (simplex[dim][it] - centroid[it]) })
                val fContracted = costFn(contracted)
                if (fContracted < fValues[dim]) {
                    simplex[dim] = contracted; fValues[dim] = fContracted
                } else {
                    // Shrink
                    for (i in 1..dim) {
                        for (j in 0 until dim) simplex[i][j] = simplex[0][j] + sigma * (simplex[i][j] - simplex[0][j])
                        simplex[i] = clamp(simplex[i])
                        fValues[i] = costFn(simplex[i])
                    }
                }
            }
        }

        return simplex[fValues.indices.minByOrNull { fValues[it] }!!]
    }

    // ------------------------------------------------------------------
    // Inverse lookups
    // ------------------------------------------------------------------

    /**
     * Find (RPM, hoodAngle) that produces the desired velocity vector (vx, vy) in m/s.
     *
     * Uses multi-start L-BFGS-B-style bounded optimisation (Nelder-Mead in practice).
     *
     * @param vx desired horizontal velocity (m/s)
     * @param vy desired vertical velocity (m/s)
     * @param coeffs fitted surface coefficients
     * @param rpmRange allowed RPM range
     * @param hoodRange allowed hood angle range (degrees)
     * @return best (RPM, hoodAngle) or null if no solution found
     */
    fun velocityToShooterParams(
        vx: Double,
        vy: Double,
        coeffs: ShooterSurfaceCoeffs,
        rpmRange: ClosedFloatingPointRange<Double> = 1000.0..6000.0,
        hoodRange: ClosedFloatingPointRange<Double> = 5.0..45.0
    ): ShooterParams? {
        val targetV = sqrt(vx * vx + vy * vy)
        val targetTheta = atan2(vy, vx)

        return multiStartMinimise(rpmRange, hoodRange) { rpm, hood ->
            val vPred = predictSpeed(rpm, hood, coeffs)
            val thetaPred = predictAngle(rpm, hood, coeffs)
            val vErr = (vPred - targetV).let { it * it }
            val thetaErr = 100.0 * (thetaPred - targetTheta).let { it * it }
            vErr + thetaErr
        }
    }

    /**
     * Find (RPM, hoodAngle) that lands the ball at the given distance
     * and at the correct goal height.
     *
     * @param distance target distance (m)
     * @param coeffs fitted surface coefficients
     * @param rpmRange allowed RPM range
     * @param hoodRange allowed hood angle range (degrees)
     * @return best (RPM, hoodAngle) or null if no solution found
     */
    fun distanceToShooterParams(
        distance: Double,
        coeffs: ShooterSurfaceCoeffs,
        rpmRange: ClosedFloatingPointRange<Double> = 1000.0..6000.0,
        hoodRange: ClosedFloatingPointRange<Double> = 5.0..45.0
    ): ShooterParams? {
        return multiStartMinimise(rpmRange, hoodRange) { rpm, hood ->
            val v = predictSpeed(rpm, hood, coeffs)
            val theta = predictAngle(rpm, hood, coeffs)
            val dPred = predictedDistance(v, theta)
            (dPred - distance).let { it * it }
        }
    }

    /**
     * Multi-start optimisation over a 2D grid of initial guesses.
     * Returns the best (RPM, hoodAngle) found, or null if all attempts fail.
     */
    private fun multiStartMinimise(
        rpmRange: ClosedFloatingPointRange<Double>,
        hoodRange: ClosedFloatingPointRange<Double>,
        gridSize: Int = 5,
        cost: (rpm: Double, hood: Double) -> Double
    ): ShooterParams? {
        val lowerBounds = doubleArrayOf(rpmRange.start, hoodRange.start)
        val upperBounds = doubleArrayOf(rpmRange.endInclusive, hoodRange.endInclusive)

        var bestParams: DoubleArray? = null
        var bestCost = Double.MAX_VALUE

        val rpmStep = (rpmRange.endInclusive - rpmRange.start) / (gridSize - 1).coerceAtLeast(1)
        val hoodStep = (hoodRange.endInclusive - hoodRange.start) / (gridSize - 1).coerceAtLeast(1)

        for (ri in 0 until gridSize) {
            for (hi in 0 until gridSize) {
                val rpmInit = rpmRange.start + ri * rpmStep
                val hoodInit = hoodRange.start + hi * hoodStep
                val initial = doubleArrayOf(rpmInit, hoodInit)

                val result = nelderMead(
                    costFn = { p -> cost(p[0], p[1]) },
                    initial = initial,
                    lowerBounds = lowerBounds,
                    upperBounds = upperBounds,
                    maxIter = 2000,
                    tolerance = 1e-12
                )
                val c = cost(result[0], result[1])
                if (c < bestCost) {
                    bestCost = c
                    bestParams = result
                }
            }
        }

        return bestParams?.let { ShooterParams(it[0], it[1]) }
    }
}
