package sigmacorns.control.aim.tune

import sigmacorns.constants.flywheelRadius
import sigmacorns.constants.turretPos
import sigmacorns.logic.AimConfig
import kotlin.math.cos
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Fits the 6-coefficient bivariate polynomial
 *   omega(phi, v) = c0 + c1*v + c2*phi + c3*v^2 + c4*phi*v + c5*phi^2
 * from a list of [SpeedPoint]s. Produces a FloatArray(6) suitable for
 * the native turret_planner (TurretPlannerBridge omegaCoeffs).
 *
 * Each SpeedPoint is converted from (distance, hoodAngle, speed) to
 * (phi, vExit, omega) by inverting the flat-earth projectile motion at
 * the goal height, then a weighted least-squares fit is solved via
 * Cholesky on the 6x6 normal equations.
 *
 * Empirical points get weight 1.0; physicsEstimate points get weight 0.1.
 * Falls back to the trivial linear model [0, 1/(r*eta), 0, 0, 0, 0] when
 * fewer than 6 usable points are available.
 */
object OmegaCoefFitter {
    private const val G = 9.81
    private const val EMPIRICAL_W = 1.0
    private const val PHYSICS_W = 1.0

    private fun fallback(): FloatArray {
        val c1 = (1.0 / (flywheelRadius * AimConfig.launchEfficiency)).toFloat()
        return floatArrayOf(0f, c1, 0f, 0f, 0f, 0f)
    }

    /** Invert projectile motion: distance + hood angle -> exit velocity (m/s), or null if infeasible. */
    fun distanceHoodToVExit(distance: Double, phiRad: Double): Double? {
        val h = AimConfig.goalHeight - turretPos.z
        val cosPhi = cos(phiRad)
        val tanPhi = tan(phiRad)
        val denom = distance * tanPhi - h
        if (denom <= 0.0 || cosPhi <= 0.0) return null
        val vSq = G * distance * distance / (2.0 * cosPhi * cosPhi * denom)
        if (vSq <= 0.0 || !vSq.isFinite()) return null
        return sqrt(vSq)
    }

    fun fit(points: List<SpeedPoint>): FloatArray {
        // Build rows: basis [1, v, phi, v^2, phi*v, phi^2], target omega, weight w.
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

        // Normal equations: (A^T W A) c = A^T W omega. 6x6 symmetric.
        val ata = Array(6) { DoubleArray(6) }
        val atb = DoubleArray(6)
        for (r in rows) {
            val b = r.basis
            val wy = r.w * r.omega
            for (i in 0..5) {
                atb[i] += b[i] * wy
                val wbi = r.w * b[i]
                for (j in i..5) {
                    ata[i][j] += wbi * b[j]
                }
            }
        }
        for (i in 0..5) for (j in 0 until i) ata[i][j] = ata[j][i]

        val c = choleskySolve(ata, atb) ?: return fallback()
        return FloatArray(6) { c[it].toFloat() }
    }

    /**
     * Solve symmetric positive-definite 6x6 A x = b via Cholesky. Returns null if not PD
     * (e.g. degenerate point set). A is modified in place.
     */
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
        // Forward solve L y = b
        val y = DoubleArray(n)
        for (i in 0 until n) {
            var s = b[i]
            for (k in 0 until i) s -= l[i][k] * y[k]
            y[i] = s / l[i][i]
        }
        // Back solve L^T x = y
        val x = DoubleArray(n)
        for (i in n - 1 downTo 0) {
            var s = y[i]
            for (k in i + 1 until n) s -= l[k][i] * x[k]
            x[i] = s / l[i][i]
        }
        return x
    }
}
