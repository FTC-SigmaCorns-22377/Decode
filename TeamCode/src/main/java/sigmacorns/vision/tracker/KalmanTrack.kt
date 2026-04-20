package sigmacorns.vision.tracker

import org.joml.Vector2d

/**
 * Constant-velocity Kalman track for a single ball in the field plane.
 *
 *   State x       = [px, py, vx, vy]          (4x1, row vector stored flat)
 *   Covariance P  = 4x4 row-major, stored flat as `DoubleArray(16)`
 *   Transition F  = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
 *   Process   Q   = sigma_a^2 * [[dt^4/4, 0,      dt^3/2, 0     ],
 *                                [0,      dt^4/4, 0,      dt^3/2],
 *                                [dt^3/2, 0,      dt^2,   0     ],
 *                                [0,      dt^3/2, 0,      dt^2  ]]
 *   Measurement H = [[1,0,0,0],[0,1,0,0]]
 *   Measurement R = caller-supplied 2x2 (from Projection.projectToGroundWithCovariance).
 *
 * Conventions:
 *   - `predict(tNow)` advances state/covariance by `dt = tNow - lastUpdateT`.
 *     It does NOT update `lastUpdateT`; that only advances on `update(...)`.
 *     This matches the spec's "predict on a copy for position queries" pattern.
 *   - `update(z, R, tNow)` runs the KF gain update, sets `lastUpdateT = tNow`,
 *     resets `framesSinceSeen`, and increments `hits`.
 *   - All time arguments are seconds (Double). `dt < 0` is clamped to 0 so
 *     out-of-order detections produce a no-op predict instead of a NaN.
 */
class KalmanTrack(
    val id: Int,
    initialPos: Vector2d,
    initialPosCov: DoubleArray,
    t0: Double,
    initialVelVar: Double,
    private val sigmaAMps2: Double,
) {
    /** State: [px, py, vx, vy]. */
    val state: DoubleArray = doubleArrayOf(initialPos.x, initialPos.y, 0.0, 0.0)

    /** Covariance: 4x4 row-major. */
    val P: DoubleArray = DoubleArray(16).also { P ->
        require(initialPosCov.size == 4) { "initialPosCov must be a 2x2 row-major matrix (size=4)" }
        // Upper-left 2x2 = measurement covariance.
        P[0] = initialPosCov[0]; P[1] = initialPosCov[1]
        P[4] = initialPosCov[2]; P[5] = initialPosCov[3]
        // Lower-right 2x2 = isotropic velocity variance.
        P[10] = initialVelVar
        P[15] = initialVelVar
    }

    var lastUpdateT: Double = t0
        private set
    var framesSinceSeen: Int = 0
        private set
    var hits: Int = 1
        private set

    fun position(): Vector2d = Vector2d(state[0], state[1])
    fun velocity(): Vector2d = Vector2d(state[2], state[3])

    /**
     * Advance state/covariance in place by `dt = tNow - lastUpdateT`. Does not
     * move `lastUpdateT` forward. Safe to call multiple times per tick.
     */
    fun predict(tNow: Double) {
        val dt = (tNow - lastUpdateT).coerceAtLeast(0.0)
        if (dt == 0.0) return

        // x <- F x
        val px = state[0]; val py = state[1]
        val vx = state[2]; val vy = state[3]
        state[0] = px + dt * vx
        state[1] = py + dt * vy
        // vx, vy unchanged

        // P <- F P F^T + Q
        // F is block-upper-triangular with I on diagonal and dt*I on the off-diagonal block,
        // so F P F^T can be done analytically in 4x4 without allocating.
        val p = P  // alias
        // F P: row i of F-times-P is  row_i(P) + (F_i3 * row_2(P)) + (F_i4 * row_3(P))
        // F has nonzero off-diagonal entries only at F[0,2]=dt and F[1,3]=dt.
        // (FP)[0,j] = P[0,j] + dt*P[2,j]
        // (FP)[1,j] = P[1,j] + dt*P[3,j]
        // (FP)[2,j] = P[2,j]
        // (FP)[3,j] = P[3,j]
        // Then (FP F^T)[i,j] = (FP)[i,j] + dt * (FP)[i, j-2 mapped via F^T[2,0]=dt, F^T[3,1]=dt].
        // F^T has F^T[2,0]=dt, F^T[3,1]=dt. So column 0 of F^T-times-anything gets +dt*row2,
        // which rearranges to: (FP F^T)[i, 0] += dt * (FP)[i, 2], and similarly [i,1] += dt * (FP)[i,3].
        // Implementation below inlines all that.

        val fp = DoubleArray(16)
        for (j in 0 until 4) {
            fp[0 * 4 + j] = p[0 * 4 + j] + dt * p[2 * 4 + j]
            fp[1 * 4 + j] = p[1 * 4 + j] + dt * p[3 * 4 + j]
            fp[2 * 4 + j] = p[2 * 4 + j]
            fp[3 * 4 + j] = p[3 * 4 + j]
        }
        for (i in 0 until 4) {
            val base = i * 4
            val col0 = fp[base + 0] + dt * fp[base + 2]
            val col1 = fp[base + 1] + dt * fp[base + 3]
            val col2 = fp[base + 2]
            val col3 = fp[base + 3]
            p[base + 0] = col0
            p[base + 1] = col1
            p[base + 2] = col2
            p[base + 3] = col3
        }

        // P += Q   (sigma_a^2 * block matrix, dense 4x4, most entries nonzero in position/velocity corners)
        val sa2 = sigmaAMps2 * sigmaAMps2
        val dt2 = dt * dt
        val dt3 = dt2 * dt
        val dt4 = dt3 * dt
        val q11 = sa2 * dt4 / 4.0
        val q13 = sa2 * dt3 / 2.0
        val q33 = sa2 * dt2
        p[0 * 4 + 0] += q11
        p[1 * 4 + 1] += q11
        p[0 * 4 + 2] += q13; p[2 * 4 + 0] += q13
        p[1 * 4 + 3] += q13; p[3 * 4 + 1] += q13
        p[2 * 4 + 2] += q33
        p[3 * 4 + 3] += q33
    }

    /**
     * Apply a position measurement. Predicts forward to `tNow` first, then
     * performs the standard KF update. Advances `lastUpdateT` to `tNow`.
     *
     * `rMeas` is row-major 2x2 (the output of `projectToGroundWithCovariance`).
     */
    fun update(z: Vector2d, rMeas: DoubleArray, tNow: Double) {
        require(rMeas.size == 4) { "rMeas must be a 2x2 row-major matrix (size=4)" }
        predict(tNow)

        // Innovation y = z - H x   (H picks out px, py)
        val yx = z.x - state[0]
        val yy = z.y - state[1]

        // S = H P H^T + R = top-left 2x2 of P, plus R.
        val s00 = P[0]  + rMeas[0]
        val s01 = P[1]  + rMeas[1]
        val s10 = P[4]  + rMeas[2]
        val s11 = P[5]  + rMeas[3]
        val sDet = s00 * s11 - s01 * s10
        require(sDet > 1e-30) { "Innovation covariance is singular (det=$sDet)" }
        val inv00 =  s11 / sDet
        val inv01 = -s01 / sDet
        val inv10 = -s10 / sDet
        val inv11 =  s00 / sDet

        // K = P H^T S^{-1}; P H^T is the first two columns of P (4x2).
        val ph00 = P[0];  val ph01 = P[1]
        val ph10 = P[4];  val ph11 = P[5]
        val ph20 = P[8];  val ph21 = P[9]
        val ph30 = P[12]; val ph31 = P[13]

        val k00 = ph00 * inv00 + ph01 * inv10
        val k01 = ph00 * inv01 + ph01 * inv11
        val k10 = ph10 * inv00 + ph11 * inv10
        val k11 = ph10 * inv01 + ph11 * inv11
        val k20 = ph20 * inv00 + ph21 * inv10
        val k21 = ph20 * inv01 + ph21 * inv11
        val k30 = ph30 * inv00 + ph31 * inv10
        val k31 = ph30 * inv01 + ph31 * inv11

        // x <- x + K y
        state[0] += k00 * yx + k01 * yy
        state[1] += k10 * yx + k11 * yy
        state[2] += k20 * yx + k21 * yy
        state[3] += k30 * yx + k31 * yy

        // P <- (I - K H) P
        // (K H) has nonzero columns 0 and 1, where K H = K applied on columns 0,1 of I => 4x4 with
        // first two cols equal to K, rest zero.
        val newP = DoubleArray(16)
        for (i in 0 until 4) {
            val kiRow0 = when (i) { 0 -> k00; 1 -> k10; 2 -> k20; else -> k30 }
            val kiRow1 = when (i) { 0 -> k01; 1 -> k11; 2 -> k21; else -> k31 }
            for (j in 0 until 4) {
                // (I - K H) has rows: row_i = e_i - [kiRow0, kiRow1, 0, 0]
                val khp = kiRow0 * P[0 * 4 + j] + kiRow1 * P[1 * 4 + j]
                newP[i * 4 + j] = P[i * 4 + j] - khp
            }
        }
        System.arraycopy(newP, 0, P, 0, 16)

        lastUpdateT = tNow
        framesSinceSeen = 0
        hits += 1
    }

    /** Mark this tick as a miss. Used by the tracker when no detection associates. */
    fun markMissed() {
        framesSinceSeen += 1
    }

    /**
     * Position predicted to `tNow`, without mutating this track. Useful for
     * the `selectTarget` consumer and for external queries.
     */
    fun positionAt(tNow: Double): Vector2d {
        val dt = (tNow - lastUpdateT).coerceAtLeast(0.0)
        return Vector2d(state[0] + dt * state[2], state[1] + dt * state[3])
    }

    /** Trace of the top-left 2x2 of P — a scalar "position uncertainty" proxy. */
    fun positionCovTrace(): Double = P[0] + P[5]
}
