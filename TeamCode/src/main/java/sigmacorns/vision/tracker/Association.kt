package sigmacorns.vision.tracker

import org.joml.Vector2d

/**
 * Field-coordinate detection with a per-detection 2x2 covariance (row-major).
 * Produced by `Projection.projectToGroundWithCovariance` + the image-space
 * mask. The tracker then hands a list of these to `Association.assign`.
 */
data class FieldDetection(
    val pos: Vector2d,
    val cov: DoubleArray,
    val pixel: Vector2d? = null,
) {
    init {
        require(cov.size == 4) { "cov must be a 2x2 matrix in row-major order (size=4)" }
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is FieldDetection) return false
        return pos == other.pos && cov.contentEquals(other.cov) && pixel == other.pixel
    }

    override fun hashCode(): Int {
        var h = pos.hashCode()
        h = 31 * h + cov.contentHashCode()
        h = 31 * h + (pixel?.hashCode() ?: 0)
        return h
    }
}

/** Result of greedy assignment. Indices refer to positions in the input lists. */
data class AssociationResult(
    val matches: List<Pair<Int, Int>>,   // (trackIdx, detectionIdx)
    val unmatchedTracks: List<Int>,
    val unmatchedDetections: List<Int>,
)

/**
 * Mahalanobis-gated greedy assignment between existing tracks and new
 * field-coordinate detections. Uses S = H P H^T + R_meas_j as the innovation
 * covariance (H = [I_2 | 0]), so only the top-left 2x2 of each track's P is
 * needed. Tracks must be caller-predicted to the detection time before this
 * is invoked — Association is stateless.
 *
 * Greedy: enumerate all (t, d) pairs whose d^2 <= chi2Gate, sort ascending,
 * take each pair whose track and detection are both still unclaimed.
 * Chi-square 2-dof at 99% is 9.21.
 */
object Association {

    fun assign(
        tracks: List<KalmanTrack>,
        detections: List<FieldDetection>,
        chi2Gate: Double,
    ): AssociationResult {
        if (tracks.isEmpty() || detections.isEmpty()) {
            return AssociationResult(
                matches = emptyList(),
                unmatchedTracks = tracks.indices.toList(),
                unmatchedDetections = detections.indices.toList(),
            )
        }

        data class Cand(val t: Int, val d: Int, val dist2: Double)

        val cands = ArrayList<Cand>(tracks.size * detections.size)
        for (i in tracks.indices) {
            val tr = tracks[i]
            val px = tr.state[0]
            val py = tr.state[1]
            val p00 = tr.P[0]
            val p01 = tr.P[1]
            val p10 = tr.P[4]
            val p11 = tr.P[5]
            for (j in detections.indices) {
                val det = detections[j]
                val s00 = p00 + det.cov[0]
                val s01 = p01 + det.cov[1]
                val s10 = p10 + det.cov[2]
                val s11 = p11 + det.cov[3]
                val sDet = s00 * s11 - s01 * s10
                if (sDet <= 0.0) continue
                val invDet = 1.0 / sDet
                val yx = det.pos.x - px
                val yy = det.pos.y - py
                val d2 = invDet * (s11 * yx * yx - (s01 + s10) * yx * yy + s00 * yy * yy)
                if (d2.isFinite() && d2 <= chi2Gate) {
                    cands.add(Cand(i, j, d2))
                }
            }
        }

        cands.sortBy { it.dist2 }

        val tClaimed = BooleanArray(tracks.size)
        val dClaimed = BooleanArray(detections.size)
        val matches = ArrayList<Pair<Int, Int>>()
        for (c in cands) {
            if (tClaimed[c.t] || dClaimed[c.d]) continue
            matches.add(c.t to c.d)
            tClaimed[c.t] = true
            dClaimed[c.d] = true
        }
        val unmatchedTracks = (0 until tracks.size).filter { !tClaimed[it] }
        val unmatchedDets   = (0 until detections.size).filter { !dClaimed[it] }
        return AssociationResult(matches, unmatchedTracks, unmatchedDets)
    }
}
