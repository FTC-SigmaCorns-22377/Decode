package sigmacorns.control.aim.tune

import sigmacorns.constants.ballExitRadius
import sigmacorns.constants.flywheelRadius
import sigmacorns.constants.turretPos
import sigmacorns.logic.AimConfig
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Fits the 6-coefficient bivariate polynomial
 *   omega(phi, v) = c0 + c1*v + c2*phi + c3*v^2 + c4*phi*v + c5*phi^2
 * from a list of [SpeedPoint]s. Produces a FloatArray(6) suitable for
 * the native turret_planner (TurretPlannerBridge omegaCoeffs).
 *
 * Each SpeedPoint is converted from (distance, hoodAngle, speed) to
 * (phi, vExit, omega) using a drag-aware projectile inversion that
 * matches the linear-drag model (dragK) used by the native solver.
 * A weighted least-squares fit is solved via Cholesky on the 6x6 normal equations.
 *
 * Empirical points get weight 1.0; physicsEstimate points get weight 0.1.
 * Falls back to the trivial linear model [0, 1/(r*eta), 0, 0, 0, 0] when
 * fewer than 6 usable points are available.
 *
 * Drag model: linear (Stokes) drag, dv/dt = -k*v - g*ẑ
 *   x(t) = (vx0/k)(1 - e^{-kt})
 *   z(t) = z0 + ((vz0/k + g/k²)(1 - e^{-kt}) - g*t/k
 *
 * The inversion accounts for the ball exit radius (rH) offset from the turret pivot.
 */
object OmegaCoefFitter {
    private const val EMPIRICAL_W = 1.0
    private const val PHYSICS_W = 0.0

    private fun fallback(): FloatArray {
        val c1 = (1.0 / (flywheelRadius * AimConfig.launchEfficiency)).toFloat()
        return floatArrayOf(0f, c1, 0f, 0f, 0f, 0f)
    }

    /**
     * Drag-aware inversion: (targetDistance, phi) → v_exit required to hit the goal,
     * consistent with the native solver's linear-drag physics (AimConfig.dragK).
     *
     * targetDistance: robot-frame distance to goal (metres), as reported by NativeAutoAim.targetDistance.
     * The turret offset and ball exit radius are accounted for internally.
     *
     * Returns null when the shot is geometrically infeasible at this phi.
     */
    fun distanceHoodToVExit(targetDistance: Double, phiRad: Double): Double? {
        val sinPhi = sin(phiRad)
        val cosPhi = cos(phiRad)
        val rH = ballExitRadius

        // Launch point: turret pivot is at turretPos.x behind robot centre.
        // Native solver: sx = turretPos.x + rH*(1-sinPhi), so dx = dist - turretPos.x - rH*(1-sinPhi).
        // turretPos.x is negative (pivot is behind centre), so -turretPos.x > 0.
        val launchZ = turretPos.z + rH * cosPhi
        val dH = targetDistance - turretPos.x - rH * (1.0 - sinPhi)   // matches native solver dx
        val dZ = AimConfig.goalHeight - launchZ    // vertical rise required

        val k = AimConfig.dragK
        val g = AimConfig.g

        if (k < 1e-6) {
            // Drag-free closed form
            if (cosPhi <= 0.0) return null
            val tanPhi = tan(phiRad)
            val denom = dH * tanPhi - dZ
            if (denom <= 0.0) return null
            val vSq = g * dH * dH / (2.0 * cosPhi * cosPhi * denom)
            return if (vSq > 0.0 && vSq.isFinite()) sqrt(vSq) else null
        }

        // Drag-aware: binary search on v_exit.
        // For a given v, T is determined by x(T)=dH, then check z(T)=dZ.
        //
        // x(T) = (vx0/k)(1-e^{-kT}) = dH  →  e^{-kT} = 1 - k*dH/vx0  →  T = -ln(…)/k
        // z(T) = ((vz0+g/k)/k)(1-e^{-kT}) - g*T/k
        //
        // With increasing v, z(T) is monotonically increasing (higher launch → ball
        // arrives at dH at greater height). So bisect z_error(v) = z(T) - dZ.

        val vMax = AimConfig.vMax

        // Minimum v for the ball to reach dH horizontally against drag
        val vHMin = k * dH / cosPhi * 1.0001
        if (vHMin >= vMax) return null

        fun zError(v: Double): Double {
            val vx0 = v * cosPhi
            val vz0 = v * sinPhi
            val arg = 1.0 - k * dH / vx0
            if (arg <= 0.0) return Double.NEGATIVE_INFINITY
            val T = -ln(arg) / k
            val z = ((vz0 + g / k) / k) * (1.0 - exp(-k * T)) - g * T / k
            return z - dZ
        }

        val errLo = zError(vHMin + 0.5)
        val errHi = zError(vMax)

        // If errLo >= 0 at min feasible speed: target is too high, shot never drops enough
        if (!errLo.isFinite() || errHi < 0.0) return null
        // If errHi < 0: max speed insufficient to reach height
        if (errLo > 0.0) {
            // Even minimum speed overshoots — try smaller phi? Not our problem, return null.
            return null
        }

        var lo = vHMin + 0.5; var hi = vMax
        repeat(52) {
            val mid = (lo + hi) / 2.0
            if (zError(mid) < 0.0) lo = mid else hi = mid
        }
        val v = (lo + hi) / 2.0
        return if (v.isFinite() && v > 0.0) v else null
    }

    /**
     * Fit from bisection make-samples. Uses measured omega (not commanded).
     * Augments with any existing empirical SpeedPoints (e.g. pre-bisection data).
     */
    fun fitFromMakes(makes: List<BisectionSample>, extra: List<SpeedPoint> = emptyList()): FloatArray {
        val pts = makes.map { SpeedPoint(it.distance, it.omegaMeasured, it.hoodAngleDeg) } + extra
        return fit(pts)
    }

    fun fit(points: List<SpeedPoint>): FloatArray {
        data class Row(val basis: DoubleArray, val omega: Double, val w: Double)
        val rows = ArrayList<Row>(points.size)
        for (p in points) {
            val phi = Math.toRadians(p.hoodAngle)
            val v = distanceHoodToVExit(p.distance, phi) ?: continue
            val basis = doubleArrayOf(1.0, v, phi, v * v, phi * v, phi * phi)
            val w = if (p.physicsEstimate) PHYSICS_W else EMPIRICAL_W
            rows.add(Row(basis, p.speed, w))
        }
        if (rows.size < 6) return fallback()

        val ata = Array(6) { DoubleArray(6) }
        val atb = DoubleArray(6)
        for (r in rows) {
            val b = r.basis
            val wy = r.w * r.omega
            for (i in 0..5) {
                atb[i] += b[i] * wy
                val wbi = r.w * b[i]
                for (j in i..5) ata[i][j] += wbi * b[j]
            }
        }
        for (i in 0..5) for (j in 0 until i) ata[i][j] = ata[j][i]

        val c = choleskySolve(ata, atb) ?: return fallback()
        return FloatArray(6) { c[it].toFloat() }
    }

    private fun choleskySolve(a: Array<DoubleArray>, b: DoubleArray): DoubleArray? {
        val n = 6
        val l = Array(n) { DoubleArray(n) }
        for (i in 0 until n) {
            for (j in 0..i) {
                var sum = a[i][j]
                for (k in 0 until j) sum -= l[i][k] * l[j][k]
                if (i == j) {
                    if (sum <= 1e-12) return null
                    l[i][i] = sqrt(sum)
                } else {
                    l[i][j] = sum / l[j][j]
                }
            }
        }
        val y = DoubleArray(n)
        for (i in 0 until n) {
            var s = b[i]
            for (k in 0 until i) s -= l[i][k] * y[k]
            y[i] = s / l[i][i]
        }
        val x = DoubleArray(n)
        for (i in n - 1 downTo 0) {
            var s = y[i]
            for (k in i + 1 until n) s -= l[k][i] * x[k]
            x[i] = s / l[i][i]
        }
        return x
    }
}
