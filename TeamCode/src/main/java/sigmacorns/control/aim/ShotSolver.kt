package sigmacorns.control.aim

import java.util.PriorityQueue
import kotlin.math.absoluteValue
import kotlin.math.max

interface OmegaMap {
    fun omega(hood: Double, vExit: Double): Double

    // lipschitz bound of omega,
    // if bounds on dOmega/dHood and dOmega/dvExit are know then by triangle inequality
    // Lomega = lOmegaHood*Lhood + lOmegaVexit*lvExit
    fun lipschitzBound(hood: ClosedRange<Double>, vExit: ClosedRange<Double>, lHood: Double, lvExit: Double): Double
}

class ShotSolver(
    val omega: OmegaMap,
    val wOmega: Double, // time per unit change in omega
    val wTheta: Double, // time per unit change in theta
    val wPhi: Double, // time per unit change in phi
    val ballistics: Ballistics
) {
    private fun moveCost(from: Ballistics.ShotState, to: Ballistics.ShotState): Double {
        val fromOmega = omega.omega(from.phi,from.vExit)
        val toOmega = omega.omega(to.phi,to.vExit)
        val dOmega = (fromOmega - toOmega).absoluteValue
        val dPhi = (from.phi - to.phi).absoluteValue
        val dTheta = (from.theta - to.theta).absoluteValue

        return max(wOmega*dOmega, max(wPhi*dPhi,wTheta*dTheta))
    }

    fun optimalAdjust(cur: Ballistics.ShotState, target: Ballistics.Target, tol: Double): Ballistics.ShotState {
        val bounds = ballistics.tBounds(target, tol / 2.0)
        val tMin = bounds.start
        val tMax = bounds.endInclusive
        val tMid = (tMin + tMax) / 2.0

        // Lipschitz bounds for solve() components over [tMin, tMax]
        val lip = ballistics.lipschitzBounds(target, tMid, tMin, tMax)

        // Estimate phi and vExit ranges from endpoints for omega Lipschitz bound
        val sLo = ballistics.solve(target, tMin)
        val sHi = ballistics.solve(target, tMax)
        val phiRange = minOf(sLo.phi, sHi.phi)..maxOf(sLo.phi, sHi.phi)
        val vRange = minOf(sLo.vExit, sHi.vExit)..maxOf(sLo.vExit, sHi.vExit)
        val lOmega = omega.lipschitzBound(phiRange, vRange, lip.lPhi, lip.lVExit)

        // Lipschitz constant of T -> moveCost(cur, solve(target, T))
        val L = max(wOmega * lOmega, max(wPhi * lip.lPhi, wTheta * lip.lTheta))

        // Piyavskii-Shubert 1D Lipschitz minimization
        val samples = sortedMapOf<Double, Double>()
        fun eval(t: Double): Double = moveCost(cur, ballistics.solve(target, t)).also { samples[t] = it }

        eval(tMin)
        eval(tMax)

        var bestT = if (samples[tMin]!! <= samples[tMax]!!) tMin else tMax
        var bestCost = samples[bestT]!!
        var lbGlobal = Double.NEGATIVE_INFINITY

        var iter = 0
        while (bestCost - lbGlobal > tol && iter < 500) {
            iter++

            // Find interval with lowest lower bound and the T that achieves it
            var lbMin = Double.MAX_VALUE
            var nextT = tMid

            val keys = samples.keys.toList()
            for (i in 0 until keys.size - 1) {
                val ta = keys[i]; val tb = keys[i + 1]
                val fa = samples[ta]!!; val fb = samples[tb]!!
                // Minimum of the sawtooth lower envelope in [ta, tb]
                val lb = (fa + fb) / 2.0 - L * (tb - ta) / 2.0
                if (lb < lbMin) {
                    lbMin = lb
                    nextT = ((ta + tb) / 2.0 + (fa - fb) / (2.0 * L)).coerceIn(ta, tb)
                }
            }

            lbGlobal = lbMin

            val cost = eval(nextT)
            if (cost < bestCost) {
                bestCost = cost
                bestT = nextT
            }
        }

        return ballistics.solve(target, bestT)
    }

    /**
     * Minimize J(T1, T2) = moveCost(s1_reduced, s2) where s1_reduced is s1 with its
     * flywheel speed (omega) reduced by omegaDrop, simulating the speed loss after firing.
     *
     * Finds shot parameters robust to flywheel loss: the transition from post-shot
     * speed to the next shot's required speed is minimized.
     *
     * Uses 2D branch-and-bound with Lipschitz lower bounds, bisecting the dimension
     * with the larger Lipschitz-weighted half-width at each step.
     */
    fun optimalRobustShot(
        target1: Ballistics.Target,
        target2: Ballistics.Target,
        omegaDrop: Double,
        tol: Double,
        maxIter: Int = 100
    ): Pair<Ballistics.ShotState, Ballistics.ShotState> {
        val b1 = ballistics.tBounds(target1, tol / 4.0)
        val b2 = ballistics.tBounds(target2, tol / 4.0)
        val tMin1 = b1.start;       val tMax1 = b1.endInclusive
        val tMin2 = b2.start;       val tMax2 = b2.endInclusive
        val tMid1 = (tMin1 + tMax1) / 2.0
        val tMid2 = (tMin2 + tMax2) / 2.0

        val lip1 = ballistics.lipschitzBounds(target1, tMid1, tMin1, tMax1)
        val lip2 = ballistics.lipschitzBounds(target2, tMid2, tMin2, tMax2)

        fun omegaL(lipB: Ballistics.LipschitzBounds, target: Ballistics.Target, tLo: Double, tHi: Double): Double {
            val sLo = ballistics.solve(target, tLo); val sHi = ballistics.solve(target, tHi)
            val phiR = minOf(sLo.phi, sHi.phi)..maxOf(sLo.phi, sHi.phi)
            val vR   = minOf(sLo.vExit, sHi.vExit)..maxOf(sLo.vExit, sHi.vExit)
            return omega.lipschitzBound(phiR, vR, lipB.lPhi, lipB.lVExit)
        }

        val lOmega1 = omegaL(lip1, target1, tMin1, tMax1)
        val lOmega2 = omegaL(lip2, target2, tMin2, tMax2)

        // Subtracting a constant from omega1 doesn't change the Lipschitz constant of |omega1 - omega2|
        val L1 = max(wOmega * lOmega1, max(wPhi * lip1.lPhi, wTheta * lip1.lTheta))
        val L2 = max(wOmega * lOmega2, max(wPhi * lip2.lPhi, wTheta * lip2.lTheta))

        fun J(t1: Double, t2: Double): Double {
            val s1 = ballistics.solve(target1, t1)
            val s2 = ballistics.solve(target2, t2)
            val omega1 = omega.omega(s1.phi, s1.vExit) - omegaDrop
            val omega2 = omega.omega(s2.phi, s2.vExit)
            val dOmega = (omega1 - omega2).absoluteValue
            val dPhi = (s1.phi - s2.phi).absoluteValue
            val dTheta = (s1.theta - s2.theta).absoluteValue
            return max(wOmega * dOmega, max(wPhi * dPhi, wTheta * dTheta))
        }

        data class Rect(val t1Lo: Double, val t1Hi: Double, val t2Lo: Double, val t2Hi: Double, val lb: Double)

        fun makeRect(t1Lo: Double, t1Hi: Double, t2Lo: Double, t2Hi: Double): Rect {
            val tc1 = (t1Lo + t1Hi) / 2.0; val tc2 = (t2Lo + t2Hi) / 2.0
            val lb = J(tc1, tc2) - L1 * (t1Hi - t1Lo) / 2.0 - L2 * (t2Hi - t2Lo) / 2.0
            return Rect(t1Lo, t1Hi, t2Lo, t2Hi, lb)
        }

        val queue = PriorityQueue<Rect>(compareBy { it.lb })
        queue.add(makeRect(tMin1, tMax1, tMin2, tMax2))

        var bestJ = J(tMid1, tMid2)
        var bestT1 = tMid1; var bestT2 = tMid2
        for (t1 in listOf(tMin1, tMax1)) for (t2 in listOf(tMin2, tMax2)) {
            val j = J(t1, t2)
            if (j < bestJ) { bestJ = j; bestT1 = t1; bestT2 = t2 }
        }

        var iter = 0
        while (iter < maxIter) {
            iter++
            val rect = queue.poll() ?: break
            if (bestJ - rect.lb <= tol) break

            val half1 = L1 * (rect.t1Hi - rect.t1Lo) / 2.0
            val half2 = L2 * (rect.t2Hi - rect.t2Lo) / 2.0
            val children = if (half1 >= half2) {
                val tm = (rect.t1Lo + rect.t1Hi) / 2.0
                listOf(makeRect(rect.t1Lo, tm, rect.t2Lo, rect.t2Hi),
                       makeRect(tm, rect.t1Hi, rect.t2Lo, rect.t2Hi))
            } else {
                val tm = (rect.t2Lo + rect.t2Hi) / 2.0
                listOf(makeRect(rect.t1Lo, rect.t1Hi, rect.t2Lo, tm),
                       makeRect(rect.t1Lo, rect.t1Hi, tm, rect.t2Hi))
            }

            for (child in children) {
                val tc1 = (child.t1Lo + child.t1Hi) / 2.0
                val tc2 = (child.t2Lo + child.t2Hi) / 2.0
                val j = J(tc1, tc2)
                if (j < bestJ) { bestJ = j; bestT1 = tc1; bestT2 = tc2 }
                if (child.lb < bestJ) queue.add(child)
            }
        }

        return Pair(ballistics.solve(target1, bestT1), ballistics.solve(target2, bestT2))
    }

    /**
     * Minimize J(T1, T2) = moveCost(s1, s2) + T2 - T1
     * over feasible flight times for two shots at different targets.
     *
     * Uses 2D branch-and-bound with Lipschitz lower bounds, bisecting the
     * dimension with the larger Lipschitz-weighted half-width at each step.
     */
    fun optimalAirShot(
        target1: Ballistics.Target,
        target2: Ballistics.Target,
        tol: Double,
        maxIter: Int = 100
    ): Pair<Ballistics.ShotState, Ballistics.ShotState> {
        val b1 = ballistics.tBounds(target1, tol / 4.0)
        val b2 = ballistics.tBounds(target2, tol / 4.0)
        val tMin1 = b1.start;       val tMax1 = b1.endInclusive
        val tMin2 = b2.start;       val tMax2 = b2.endInclusive
        val tMid1 = (tMin1 + tMax1) / 2.0
        val tMid2 = (tMin2 + tMax2) / 2.0

        // Lipschitz bounds for solve() on each target over its full feasible range
        val lip1 = ballistics.lipschitzBounds(target1, tMid1, tMin1, tMax1)
        val lip2 = ballistics.lipschitzBounds(target2, tMid2, tMin2, tMax2)

        // Omega Lipschitz bounds — sample endpoints to cover the phi/vExit ranges
        fun omegaL(lipB: Ballistics.LipschitzBounds, target: Ballistics.Target, tLo: Double, tHi: Double): Double {
            val sLo = ballistics.solve(target, tLo); val sHi = ballistics.solve(target, tHi)
            val phiR = minOf(sLo.phi, sHi.phi)..maxOf(sLo.phi, sHi.phi)
            val vR   = minOf(sLo.vExit, sHi.vExit)..maxOf(sLo.vExit, sHi.vExit)
            return omega.lipschitzBound(phiR, vR, lipB.lPhi, lipB.lVExit)
        }

        val lOmega1 = omegaL(lip1, target1, tMin1, tMax1)
        val lOmega2 = omegaL(lip2, target2, tMin2, tMax2)

        // Lipschitz constant of J wrt each variable:
        //   moveCost wrt T1: max(wOmega*lΩ1, wPhi*lφ1, wTheta*lθ1)   — only s1 depends on T1
        //   T2 - T1 contributes +1 to L2 and +1 to L1 (via -T1)
        val L1 = max(wOmega * lOmega1, max(wPhi * lip1.lPhi, wTheta * lip1.lTheta)) + 1.0
        val L2 = max(wOmega * lOmega2, max(wPhi * lip2.lPhi, wTheta * lip2.lTheta)) + 1.0

        fun J(t1: Double, t2: Double): Double =
            moveCost(ballistics.solve(target1, t1), ballistics.solve(target2, t2)) + t2 - t1

        // Rectangle in (T1, T2) space with a Lipschitz lower bound on J inside it
        data class Rect(val t1Lo: Double, val t1Hi: Double, val t2Lo: Double, val t2Hi: Double, val lb: Double)

        // Lower bound = J(center) - L1*(halfWidth1) - L2*(halfWidth2)
        fun makeRect(t1Lo: Double, t1Hi: Double, t2Lo: Double, t2Hi: Double): Rect {
            val tc1 = (t1Lo + t1Hi) / 2.0; val tc2 = (t2Lo + t2Hi) / 2.0
            val lb = J(tc1, tc2) - L1 * (t1Hi - t1Lo) / 2.0 - L2 * (t2Hi - t2Lo) / 2.0
            return Rect(t1Lo, t1Hi, t2Lo, t2Hi, lb)
        }

        val queue = PriorityQueue<Rect>(compareBy { it.lb })
        queue.add(makeRect(tMin1, tMax1, tMin2, tMax2))

        // Seed upper bound from center + corners
        var bestJ = J(tMid1, tMid2)
        var bestT1 = tMid1; var bestT2 = tMid2
        for (t1 in listOf(tMin1, tMax1)) for (t2 in listOf(tMin2, tMax2)) {
            val j = J(t1, t2)
            if (j < bestJ) { bestJ = j; bestT1 = t1; bestT2 = t2 }
        }

        var iter = 0
        while (iter < maxIter) {
            iter++
            val rect = queue.poll() ?: break
            if (bestJ - rect.lb <= tol) break

            // Bisect along the dimension with the larger Lipschitz-weighted half-width
            val half1 = L1 * (rect.t1Hi - rect.t1Lo) / 2.0
            val half2 = L2 * (rect.t2Hi - rect.t2Lo) / 2.0
            val children = if (half1 >= half2) {
                val tm = (rect.t1Lo + rect.t1Hi) / 2.0
                listOf(makeRect(rect.t1Lo, tm, rect.t2Lo, rect.t2Hi),
                       makeRect(tm, rect.t1Hi, rect.t2Lo, rect.t2Hi))
            } else {
                val tm = (rect.t2Lo + rect.t2Hi) / 2.0
                listOf(makeRect(rect.t1Lo, rect.t1Hi, rect.t2Lo, tm),
                       makeRect(rect.t1Lo, rect.t1Hi, tm, rect.t2Hi))
            }

            for (child in children) {
                // The child center was already evaluated in makeRect; update best upper bound
                val tc1 = (child.t1Lo + child.t1Hi) / 2.0
                val tc2 = (child.t2Lo + child.t2Hi) / 2.0
                val j = J(tc1, tc2)
                if (j < bestJ) { bestJ = j; bestT1 = tc1; bestT2 = tc2 }
                if (child.lb < bestJ) queue.add(child)
            }
        }

        return Pair(ballistics.solve(target1, bestT1), ballistics.solve(target2, bestT2))
    }
}