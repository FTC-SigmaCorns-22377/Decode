package sigmacorns.vision.tracker

import org.joml.Vector2d
import sigmacorns.math.Pose2d

/**
 * Raw pixel detection from the vision pipeline (Limelight, SimulatedCamera).
 * `t` is the CAPTURE timestamp for this frame, not the receive timestamp —
 * the tracker's pose lookup uses it to pull the robot pose at capture time.
 */
data class PixelDetection(
    val u: Double,
    val v: Double,
    val t: Double,
)

/** Policy for picking the "active" target from a set of confirmed tracks. */
enum class SelectionStrategy { Closest }

/**
 * Full tracker pipeline. Pure Kotlin, no I/O:
 *
 *   tick(detectionsPx, pose, t) ->
 *     1. image-space intake mask
 *     2. build T_FC from pose + config.T_RC
 *     3. project each pixel + covariance (drop nulls)
 *     4. field-coord gating (field bounds, ramp, self-footprint)
 *     5. predict all tracks to `t`
 *     6. associate (Mahalanobis-gated greedy)
 *     7. update matched tracks; markMissed unmatched
 *     8. spawn new tracks from unmatched detections
 *     9. delete tracks whose framesSinceSeen > MAX_COAST or pos-cov trace
 *        exceeds a threshold (coasting without seeing anything long enough
 *        that its uncertainty blew past the intended upper bound).
 *    10. return the live track list
 */
class Tracker(val config: TrackerConfig) {

    private val _tracks = mutableListOf<KalmanTrack>()
    val tracks: List<KalmanTrack> get() = _tracks

    private var nextId: Int = 0

    // Discard a track whose position-covariance trace exceeds MAX_RANGE^2.
    // At that point the track is less informative than an untracked detection
    // anywhere on the field.
    private val trackDiscardTrace: Double = config.maxRangeM * config.maxRangeM

    fun tick(
        detectionsPx: List<PixelDetection>,
        poseAtFrameTime: Pose2d,
        t: Double,
    ): List<KalmanTrack> {
        val T_FC = Frames.buildTFC(poseAtFrameTime, config.TRC)
        val ballHeight = config.ballRadiusM

        // 1 + 3. Mask in image space, project survivors.
        val projected = ArrayList<FieldDetection>(detectionsPx.size)
        for (det in detectionsPx) {
            if (Gating.insideIntakeMask(det.v, config.imageHeightPx, config.intakeMaskYMinFrac)) continue
            val result = Projection.projectToGroundWithCovariance(
                u = det.u, v = det.v,
                K = config.intrinsics, T_FC = T_FC,
                h = ballHeight,
                maxRangeM = config.maxRangeM,
                sigmaPx = config.sigmaPx,
            ) ?: continue
            val p = result.point
            // 4. Field-coord gates.
            if (!Gating.insideField(p, config.fieldWidthM, config.fieldHeightM, config.fieldMarginM)) continue
            if (Gating.insideRamp(p, config.rampPolygon, config.rampExpandM)) continue
            if (Gating.insideRobotFootprint(p, poseAtFrameTime, config.robotFootprintR)) continue
            projected.add(FieldDetection(pos = Vector2d(p), cov = result.cov, pixel = Vector2d(det.u, det.v)))
        }

        // 5. Predict each track forward to `t`.
        for (tr in _tracks) tr.predict(t)

        // 6. Associate.
        val assoc = Association.assign(_tracks, projected, config.chi2Gate)

        // 7. Update matched, mark unmatched tracks as missed.
        for ((trIdx, detIdx) in assoc.matches) {
            val det = projected[detIdx]
            _tracks[trIdx].update(det.pos, det.cov, t)
        }
        for (trIdx in assoc.unmatchedTracks) {
            _tracks[trIdx].markMissed()
        }

        // 8. Spawn new tracks from unmatched detections.
        for (detIdx in assoc.unmatchedDetections) {
            val det = projected[detIdx]
            _tracks.add(
                KalmanTrack(
                    id = nextId++,
                    initialPos = det.pos,
                    initialPosCov = det.cov,
                    t0 = t,
                    initialVelVar = config.initVelVar,
                    sigmaAMps2 = config.sigmaAMps2,
                )
            )
        }

        // 9. Delete stale / uncertain tracks.
        _tracks.removeAll { tr ->
            tr.framesSinceSeen > config.maxCoastFrames ||
                    tr.positionCovTrace() > trackDiscardTrace
        }

        // 10.
        return _tracks.toList()
    }

    /** True if the track has received enough associated measurements to be considered real. */
    fun isConfirmed(track: KalmanTrack): Boolean = track.hits >= config.minHitsForConfirmed

    /**
     * Pick the "active" target from the current track list. Closest-confirmed
     * strategy: among confirmed tracks, return the one nearest the robot.
     * Falls back to null if no confirmed track exists.
     */
    fun selectTarget(
        robotPose: Pose2d,
        tNow: Double,
        strategy: SelectionStrategy = SelectionStrategy.Closest,
    ): KalmanTrack? {
        val confirmed = _tracks.filter { isConfirmed(it) }
        if (confirmed.isEmpty()) return null
        return when (strategy) {
            SelectionStrategy.Closest -> confirmed.minBy { tr ->
                val p = tr.positionAt(tNow)
                val dx = p.x - robotPose.v.x
                val dy = p.y - robotPose.v.y
                dx * dx + dy * dy
            }
        }
    }

    fun reset() {
        _tracks.clear()
        nextId = 0
    }
}
