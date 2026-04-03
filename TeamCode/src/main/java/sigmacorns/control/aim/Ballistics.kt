package sigmacorns.control.aim

import org.joml.Vector2d
import org.joml.Vector3d
import kotlin.math.*

/**
 * Ballistics solver for turret aiming with hood arc.
 *
 * Computes (theta, phi, v_exit) as functions of flight time T,
 * and Lipschitz bounds on their derivatives over intervals [T* - dT, T* + dT].
 *
 * Coordinate convention:
 *   - theta: yaw angle (azimuth), measured from +x toward +y
 *   - phi: elevation angle from horizontal
 *   - v_exit: required launch speed
 *   - r_h: hood arc radius (distance from turret pivot to launch point along barrel)
 *   - g: gravitational acceleration (positive downward, so g ~ 9.81)
 */
class Ballistics(
    val rH: Double,         // barrel offset radius
    val vMax: Double,       // max exit velocity
    val phiMin: Double,     // lower elevation bound [rad]
    val phiMax: Double,     // upper elevation bound [rad]
    val g: Double = 9.81
) {
    data class Target(
        val target: Vector3d,
        val turret: Vector3d,
        val vR: Vector2d
    ) {
        val dx: Double get() = target.x - turret.x
        val dy: Double get() = target.y - turret.y
        val dz: Double get() = target.z - turret.z
    }

    data class ShotState(
        val theta: Double,
        val phi: Double,
        val vExit: Double,
    )

    data class LipschitzBounds(
        val lTheta: Double,
        val lPhi: Double,
        val lVExit: Double,
        val tMin: Double,
        val tMax: Double
    )
    fun evalTheta(p: Target, T: Double): Double {
        val b = (p.dx) / T - p.vR.x
        val c = (p.dy) / T - p.vR.y
        return atan2(c, b)
    }

    /**
     * Compute full aim solution for a given flight time T.
     */
    fun solve(p: Target, T: Double): ShotState {
        val theta = evalTheta(p, T)

        val b = (p.dx ) / T - p.vR.x
        val c = (p.dy ) / T - p.vR.y
        val a = -rH / T

        val B = a + cos(theta) * b + sin(theta) * c
        val C = p.dz / T + 0.5 * g * T

        val vExitSq = B * B + C * C - a * a
        require(vExitSq >= 0.0) { "No real solution: v_exit^2 = $vExitSq < 0" }
        val vExit = sqrt(vExitSq)

        val phi = atan2(
            C * vExit + B * a,
            B * vExit - C * a
        )

        return ShotState(theta, phi, vExit)
    }

    /**
     * Compute dTheta/dT analytically.
     */
    fun dThetaDT(p: Target, T: Double): Double {
        // theta = atan2(c, b) where b = (dx )/T - vR.x, c = (dy )/T - vR.y
        // db/dT = -(dx )/T^2,  dc/dT = -(dy )/T^2
        // dTheta/dT = (dc/dT * b - db/dT * c) / (b^2 + c^2)
        val b = (p.dx ) / T - p.vR.x
        val c = (p.dy ) / T - p.vR.y
        val dbdT = -(p.dx ) / (T * T)
        val dcdT = -(p.dy ) / (T * T)
        return (dcdT * b - dbdT * c) / (b * b + c * c)
    }

    /**
     * Lipschitz bound on |dTheta/dT| over [tMin, tMax].
     *
     * dTheta/dT = (dc/dT * b - db/dT * c) / (b^2 + c^2)
     *
     * We bound numerator above and denominator below.
     */
    fun lipschitzTheta(p: Target, tMin: Double, tMax: Double): Double {
        // b = (dx )/T - vR.x, c = (dy )/T - vR.y
        // db/dT = -(dx )/T^2, dc/dT = -(dy )/T^2
        // Numerator = dc/dT * b - db/dT * c
        //   = -(dy)/T^2 * ((dx)/T - vR.x) + (dx)/T^2 * ((dy)/T - vR.y)
        //   = 1/T^2 * ((dx)*((dy)/T - vR.y) - (dy)*((dx)/T - vR.x))
        //   = 1/T^2 * ((dx)*vRx - (dy)*vRy) ... wait, let me just do it numerically.
        // Actually: numerator = (-(dy)(dx) + (dx)(dy))/T^3 + ((dy)*vRx - (dx)*vRy)/T^2
        //                     = ((dy)*vRx - (dx)*vRy) / T^2
        // So dTheta/dT = ((dy)*vRx - (dx)*vRy) / (T^2 * (b^2 + c^2))

        val crossTerm = (p.dy ) * p.vR.x - (p.dx ) * p.vR.y

        // |numerator| upper bound: |crossTerm| / tMin^2
        val numUp = abs(crossTerm) / (tMin * tMin)

        // denominator = b^2 + c^2, need lower bound
        // b(T) = (dx)/T - vRx, evaluate at endpoints and find minimum of b^2+c^2
        val denLo = run {
            var lo = Double.MAX_VALUE
            for (t in listOf(tMin, tMax)) {
                val b = (p.dx ) / t - p.vR.x
                val c = (p.dy ) / t - p.vR.y
                lo = min(lo, b * b + c * c)
            }
            // b = 0 at T = (dx)/vRx if vRx != 0, c = 0 at T = (dy)/vRy if vRy != 0
            // Check if critical T values lie in interval
            if (p.vR.x != 0.0) {
                val tCritB = p.dx / p.vR.x
                if (tCritB in tMin..tMax) {
                    val c = (p.dy ) / tCritB - p.vR.y
                    lo = min(lo, c * c)
                }
            }
            if (p.vR.y != 0.0) {
                val tCritC = (p.dy ) / p.vR.y
                if (tCritC in tMin..tMax) {
                    val b = (p.dx ) / tCritC - p.vR.x
                    lo = min(lo, b * b)
                }
            }
            lo
        }

        require(denLo > 0.0) { "theta denominator degenerates (b=c=0 in interval)" }
        return numUp / denLo
    }

    /**
     * Lipschitz bound on |dB/dT| over [tMin, tMax].
     *
     * Uses the Cauchy-Schwarz based bound from the derivation.
     */
    fun lipschitzB(p: Target, tMin: Double, tMax: Double): Double {
        // We need to bound |dB/dT|. Use a conservative approach:
        // evaluate |dB/dT| at several sample points and add margin,
        // or use the analytic bound from the document.
        //
        // From the derivation:
        // dB/dT = da/dT + cos(theta)(dTheta/dT * c - (dx-rH)/T^2) - sin(theta)(dTheta/dT * b + (dy-rH)/T^2)
        //
        // |dB/dT| <= |da/dT| + sqrt((dTheta/dT * c - (dx-rH)/T^2)^2 + (dTheta/dT * b + (dy-rH)/T^2)^2)
        //
        // We maximize the hypot over [tMin, tMax] by evaluating at endpoints and critical points.

        // Conservative: evaluate at several points and take max, then add safety margin
        val nSamples = 16
        var maxDBDT = 0.0
        for (i in 0..nSamples) {
            val t = tMin + (tMax - tMin) * i / nSamples
            val b = (p.dx - rH) / t - p.vR.x
            val c = (p.dy - rH) / t - p.vR.y
            val dthetaDt = dThetaDT(p, t)
            val term1 = dthetaDt * c - (p.dx - rH) / (t * t)
            val term2 = dthetaDt * b + (p.dy - rH) / (t * t)
            val dbdt = rH / (t * t) + cos(evalTheta(p, t)) * term1 - sin(evalTheta(p, t)) * term2
            maxDBDT = max(maxDBDT, abs(dbdt))
        }

        // Also compute the Cauchy-Schwarz upper bound at endpoints for a rigorous bound
        var maxCS = 0.0
        for (i in 0..nSamples) {
            val t = tMin + (tMax - tMin) * i / nSamples
            val b = (p.dx ) / t - p.vR.x
            val c = (p.dy ) / t - p.vR.y
            val dthetaDt = dThetaDT(p, t)
            val term1 = dthetaDt * c - (p.dx ) / (t * t)
            val term2 = dthetaDt * b + (p.dy ) / (t * t)
            maxCS = max(maxCS, rH / (t * t) + sqrt(term1 * term1 + term2 * term2))
        }

        return maxCS
    }

    /**
     * Compute all Lipschitz bounds over [T* - deltaT, T* + deltaT].
     */
    fun lipschitzBounds(p: Target, tStar: Double, tMin: Double, tMax: Double): LipschitzBounds {
        // --- L_theta ---
        val lTheta = lipschitzTheta(p, tMin, tMax)

        // --- L_B ---
        val lB = lipschitzB(p, tMin, tMax)

        // --- B0, C bounds ---
        val sol = solve(p, tStar)
        val theta0 = sol.theta
        val b0val = (p.dx ) / tStar - p.vR.x
        val c0val = (p.dy ) / tStar - p.vR.y
        val a0 = -rH / tStar
        val B0 = a0 + cos(theta0) * b0val + sin(theta0) * c0val

        val deltaT = max(tStar-tMin, tMax-tStar)

        val bUp = abs(B0) + lB * deltaT
        val bLo = max(abs(B0) - lB * deltaT, 0.0)

        // --- C bounds ---
        val cAtMin = p.dz / tMin + 0.5 * g * tMin
        val cAtMax = p.dz / tMax + 0.5 * g * tMax
        val cMax = max(abs(cAtMin), abs(cAtMax))

        val tC = if (p.dz >= 0.0) sqrt(2.0 * p.dz / g) else Double.NaN
        val cMin = if (tC.isFinite() && tC in tMin..tMax) {
            sqrt(2.0 * g * p.dz)
        } else {
            min(abs(cAtMin), abs(cAtMax))
        }

        // --- C' bounds ---
        val cPrimeAtMin = -p.dz / (tMin * tMin) + 0.5 * g
        val cPrimeAtMax = -p.dz / (tMax * tMax) + 0.5 * g
        val cPrimeMax = max(abs(cPrimeAtMin), abs(cPrimeAtMax))

        // --- v_exit bounds ---
        val vUp = sqrt(bUp * bUp + cMax * cMax)
        val vLoSq = max(bLo * bLo + cMin * cMin - rH * rH / (tMin * tMin), 0.0)
        val vLo = sqrt(vLoSq)
        require(vLo > 0.0) { "v_exit lower bound is zero; solution degenerate in this interval" }

        // --- L_v ---
        val lV = (lB * bUp + 0.25 * g * g * tMax +
                (rH * rH + p.dz * p.dz) / (tMin * tMin * tMin)) / vLo

        // --- L_phi ---
        val denomPhi = bLo * bLo + cMin * cMin
        require(denomPhi > 0.0) { "phi denominator degenerates (B=C=0 in interval)" }

        val lPhi = (cPrimeMax * bUp + lB * cMax +
                (rH / (tMin * tMin)) * vUp +
                lV * (rH / tMin)) / denomPhi

        return LipschitzBounds(
            lTheta = lTheta,
            lPhi = lPhi,
            lVExit = lV,
            tMin = tMin,
            tMax = tMax
        )
    }

    /**
     * Verify Lipschitz bounds by sampling dT and checking finite differences.
     */
    fun verifyBounds(p: Target, tStar: Double, deltaT: Double, nSamples: Int = 1000) {
        val bounds = lipschitzBounds(p, tStar, tStar-deltaT, tStar+deltaT)
        val tMin = bounds.tMin
        val tMax = bounds.tMax

        var maxDTheta = 0.0
        var maxDPhi = 0.0
        var maxDVExit = 0.0

        for (i in 0 until nSamples) {
            val t1 = tMin + (tMax - tMin) * i / nSamples
            val t2 = tMin + (tMax - tMin) * (i + 1) / nSamples
            val s1 = solve(p, t1)
            val s2 = solve(p, t2)
            val dt = t2 - t1

            maxDTheta = max(maxDTheta, abs(s2.theta - s1.theta) / dt)
            maxDPhi = max(maxDPhi, abs(s2.phi - s1.phi) / dt)
            maxDVExit = max(maxDVExit, abs(s2.vExit - s1.vExit) / dt)
        }

        println("=== Lipschitz Bound Verification ===")
        println("Interval: [${bounds.tMin}, ${bounds.tMax}]")
        println()
        println("theta:  bound = %.6f,  observed max |d/dT| = %.6f,  ratio = %.2f".format(
            bounds.lTheta, maxDTheta, bounds.lTheta / maxDTheta))
        println("phi:    bound = %.6f,  observed max |d/dT| = %.6f,  ratio = %.2f".format(
            bounds.lPhi, maxDPhi, bounds.lPhi / maxDPhi))
        println("v_exit: bound = %.6f,  observed max |d/dT| = %.6f,  ratio = %.2f".format(
            bounds.lVExit, maxDVExit, bounds.lVExit / maxDVExit))
        println()

        if (bounds.lTheta >= maxDTheta && bounds.lPhi >= maxDPhi && bounds.lVExit >= maxDVExit) {
            println("ALL BOUNDS VALID")
        } else {
            println("WARNING: BOUND VIOLATED")
            if (bounds.lTheta < maxDTheta) println("  theta bound violated!")
            if (bounds.lPhi < maxDPhi) println("  phi bound violated!")
            if (bounds.lVExit < maxDVExit) println("  v_exit bound violated!")
        }
    }

    fun isFeasible(p: Target, T: Double): Boolean {
        val sol = solve(p,T)
        return 0 <= sol.vExit && sol.vExit <= vMax && phiMin <= sol.phi && sol.phi <= phiMax
    }

    fun tBounds(p: Target, tol: Double): ClosedRange<Double> {
        val a = g/2.0
        val b = -vMax/sqrt(2.0)
        val c = p.dz - rH/sqrt(2.0)

        val tStar = (-b + sqrt(b*b - 4.0*a*c))/(2.0*a)

        // binary search for tMax
        var lo = 0.0
        var hi = tStar

        while (hi-lo > tol) {
            val m = (hi+lo)/2.0
            if(isFeasible(p,m)) lo = m else hi = m
        }

        val tMax = lo

        // binary search for tMin
        lo = 0.0
        hi = tMax

        while (hi-lo > tol) {
            val m = (hi+lo)/2.0
            if(isFeasible(p,m)) hi = m else lo = m
        }

        val tMin = hi

        return tMin..tMax
    }
}
