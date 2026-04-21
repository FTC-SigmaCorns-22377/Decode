package sigmacorns.test

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import sigmacorns.constants.flywheelRadius
import sigmacorns.control.aim.tune.OmegaCoefFitter
import sigmacorns.control.aim.tune.SpeedPoint
import sigmacorns.logic.AimConfig
import kotlin.math.abs

class OmegaCoefFitterTest {

    private fun evalPoly(c: FloatArray, phi: Double, v: Double): Double {
        return c[0] + c[1] * v + c[2] * phi + c[3] * v * v + c[4] * phi * v + c[5] * phi * phi
    }

    /** Build a SpeedPoint from (distance, hoodDeg) by inverting projectile + evaluating a known polynomial. */
    private fun pointFromPoly(dist: Double, hoodDeg: Double, trueC: DoubleArray, physics: Boolean = false): SpeedPoint? {
        val phi = Math.toRadians(hoodDeg)
        val v = OmegaCoefFitter.distanceHoodToVExit(dist, phi) ?: return null
        val omega = trueC[0] + trueC[1] * v + trueC[2] * phi + trueC[3] * v * v + trueC[4] * phi * v + trueC[5] * phi * phi
        return SpeedPoint(dist, omega, hoodDeg, physics)
    }

    @Test
    fun fallbackWhenTooFewPoints() {
        val coeffs = OmegaCoefFitter.fit(emptyList())
        val c1 = (1.0 / (flywheelRadius * AimConfig.launchEfficiency)).toFloat()
        assertArrayEquals(floatArrayOf(0f, c1, 0f, 0f, 0f, 0f), coeffs, 1e-6f)
    }

    @Test
    fun recoversKnownPolynomial() {
        // Pick a polynomial whose shape is non-trivial but realistic.
        val trueC = doubleArrayOf(5.0, 10.0, 20.0, 0.1, -1.5, -3.0)
        val pts = mutableListOf<SpeedPoint>()
        val dists = listOf(1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5)
        val hoods = listOf(25.0, 35.0, 45.0, 55.0, 65.0)
        for (d in dists) for (h in hoods) pointFromPoly(d, h, trueC)?.let { pts.add(it) }

        assertTrue(pts.size >= 6, "need at least 6 feasible points, got ${pts.size}")
        val coeffs = OmegaCoefFitter.fit(pts)

        // Check residual at each original point is tiny — coefficients should match exactly
        // (up to numerical precision) since data was generated from the same polynomial.
        for (p in pts) {
            val phi = Math.toRadians(p.hoodAngle)
            val v = OmegaCoefFitter.distanceHoodToVExit(p.distance, phi)!!
            val predicted = evalPoly(coeffs, phi, v)
            assertEquals(p.speed, predicted, 1e-2, "residual at d=${p.distance} h=${p.hoodAngle}")
        }
    }

    @Test
    fun physicsPointsWeightedLessThanEmpirical() {
        // Two consistent empirical points per feasible (dist, hood) from poly A,
        // plus many physics points from poly B. Fit should land much closer to A.
        val polyA = doubleArrayOf(1.0, 8.0, 5.0, 0.05, -0.5, -1.0)
        val polyB = doubleArrayOf(50.0, 0.0, 0.0, 0.0, 0.0, 0.0) // wildly different constant offset

        val empirical = mutableListOf<SpeedPoint>()
        val physics = mutableListOf<SpeedPoint>()
        val dists = listOf(1.0, 2.0, 3.0, 4.0)
        val hoods = listOf(30.0, 45.0, 60.0)
        for (d in dists) for (h in hoods) {
            pointFromPoly(d, h, polyA, physics = false)?.let { empirical.add(it) }
            pointFromPoly(d, h, polyB, physics = true)?.let { physics.add(it) }
        }
        // Duplicate physics to ensure count dominates empirical
        val all = empirical + physics + physics + physics

        val coeffs = OmegaCoefFitter.fit(all)

        // Residual vs polyA should be smaller than vs polyB at empirical points
        var errA = 0.0
        var errB = 0.0
        for (p in empirical) {
            val phi = Math.toRadians(p.hoodAngle)
            val v = OmegaCoefFitter.distanceHoodToVExit(p.distance, phi)!!
            val pred = evalPoly(coeffs, phi, v)
            errA += abs(pred - p.speed)
            val bSpeed = polyB[0] + polyB[1] * v + polyB[2] * phi + polyB[3] * v * v + polyB[4] * phi * v + polyB[5] * phi * phi
            errB += abs(pred - bSpeed)
        }
        assertTrue(errA < errB, "fit closer to empirical polyA than physics polyB — errA=$errA errB=$errB")
    }
}
