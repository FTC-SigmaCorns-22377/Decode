package sigmacorns.vision.viz

import org.joml.Matrix4d
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.Frames
import sigmacorns.vision.tracker.Intrinsics
import sigmacorns.vision.tracker.PixelDetection
import sigmacorns.vision.tracker.Projection
import sigmacorns.vision.tracker.Tracker
import sigmacorns.vision.tracker.TrackerConfig
import kotlin.math.sqrt

/**
 * Assemble a [TrackerVizState] for one tick. Pure function of the inputs — no
 * mutable state, no I/O. Called by the sim harness each frame right after
 * `tracker.tick(...)` and handed to `SimVizServer.trackerStateProvider`.
 */
object VizStateBuilder {

    fun build(
        scenario: String,
        rmsErrorM: Double?,
        tracker: Tracker,
        config: TrackerConfig,
        robotPose: Pose2d,
        ballsTruthField: List<Vector3d>,
        ballsTruthColors: List<String?> = emptyList(),
        rawDetectionsPx: List<PixelDetection>,
        survivingFieldDetections: List<Vector2d>,
        survivingFieldDetectionCovs: List<DoubleArray>,
        targetId: Int?,
    ): TrackerVizState {
        val T_FC = Frames.buildTFC(robotPose, config.TRC)
        val T_CF = Matrix4d(T_FC).invert()
        val K = config.intrinsics

        val camera = buildCameraViz(config, T_FC)
        val truth = ballsTruthField.mapIndexed { idx, b ->
            val uv = Projection.forwardProject(b, K, T_FC)
            val bc = Vector3d()
            T_CF.transformPosition(b, bc)
            val depth = if (bc.z > 0.0) bc.z else null
            BallTruthViz(
                x = b.x, y = b.y, z = b.z,
                u = uv?.get(0), v = uv?.get(1),
                depth = depth,
                color = ballsTruthColors.getOrNull(idx),
            )
        }
        val detsPx = rawDetectionsPx.map { PixelDetViz(it.u, it.v) }
        val detsField = survivingFieldDetections.mapIndexed { i, p ->
            FieldDetViz(
                x = p.x, y = p.y,
                cov = survivingFieldDetectionCovs[i].toList(),
            )
        }
        val tracks = tracker.tracks.map { tr ->
            val p = tr.position()
            val uv = Projection.forwardProject(Vector3d(p.x, p.y, config.ballRadiusM), K, T_FC)
            val covPx = if (uv != null) reprojectCovToPixels(
                p.x, p.y, config.ballRadiusM, K, T_FC, tr.P,
            ) else null
            TrackViz(
                id = tr.id,
                x = p.x, y = p.y,
                vx = tr.state[2], vy = tr.state[3],
                cov = listOf(tr.P[0], tr.P[1], tr.P[4], tr.P[5]),
                hits = tr.hits,
                confirmed = tracker.isConfirmed(tr),
                u = uv?.get(0), v = uv?.get(1),
                covPx = covPx,
            )
        }
        return TrackerVizState(
            scenario = scenario,
            rmsErrorM = rmsErrorM,
            camera = camera,
            ballTruth = truth,
            detectionsPx = detsPx,
            detectionsField = detsField,
            tracks = tracks,
            targetId = targetId,
        )
    }

    /**
     * Pack T_FC's 3x3 rotation block + translation into a CameraViz, and
     * compute a rough ground-plane frustum polygon by intersecting the four
     * image corners with the floor.
     */
    private fun buildCameraViz(config: TrackerConfig, T_FC: Matrix4d): CameraViz {
        val ox = T_FC.m30(); val oy = T_FC.m31(); val oz = T_FC.m32()
        val Rfc = listOf(
            T_FC.m00(), T_FC.m10(), T_FC.m20(),
            T_FC.m01(), T_FC.m11(), T_FC.m21(),
            T_FC.m02(), T_FC.m12(), T_FC.m22(),
        )
        val frustum = buildFrustumGround(config, T_FC)
        val K = config.intrinsics
        return CameraViz(
            origin = listOf(ox, oy, oz),
            Rfc = Rfc,
            fx = K.fx, fy = K.fy, cx = K.cx, cy = K.cy,
            width = config.imageWidthPx, height = config.imageHeightPx,
            intakeMaskYMinFrac = config.intakeMaskYMinFrac,
            frustumGround = frustum,
        )
    }

    /** Intersect the four image corners with z=0 in field frame. Clips to maxRange. */
    private fun buildFrustumGround(config: TrackerConfig, T_FC: Matrix4d): List<List<Double>> {
        val w = config.imageWidthPx.toDouble()
        val h = config.imageHeightPx.toDouble()
        val corners = listOf(0.0 to 0.0, w to 0.0, w to h, 0.0 to h)
        val K = config.intrinsics
        val out = ArrayList<List<Double>>(4)
        for ((u, v) in corners) {
            val p = Projection.projectToGround(
                u = u, v = v, K = K, T_FC = T_FC,
                h = 0.0,
                maxRangeM = config.maxRangeM,
            )
            if (p != null) {
                out.add(listOf(p.x, p.y))
            } else {
                // Ray points away from the floor — cap at maxRange along the ray
                // so the frustum polygon still closes visually.
                val capped = projectRayAtHorizon(u, v, K, T_FC, config.maxRangeM)
                out.add(capped)
            }
        }
        return out
    }

    /**
     * If a camera ray misses the floor (or points upward), extend it by
     * maxRange along its horizontal projection so the polygon doesn't collapse.
     */
    private fun projectRayAtHorizon(
        u: Double, v: Double,
        K: Intrinsics, T_FC: Matrix4d,
        maxRange: Double,
    ): List<Double> {
        val dNorm = Projection.undistortPixel(u, v, K)
        val dC = Vector3d(dNorm[0], dNorm[1], 1.0)
        val dF = Vector3d()
        T_FC.transformDirection(dC, dF)
        val hx = dF.x
        val hy = dF.y
        val hn = sqrt(hx * hx + hy * hy)
        val sx: Double
        val sy: Double
        if (hn < 1e-9) { sx = 0.0; sy = 0.0 } else { sx = maxRange * hx / hn; sy = maxRange * hy / hn }
        val ox = T_FC.m30()
        val oy = T_FC.m31()
        return listOf(ox + sx, oy + sy)
    }

    /**
     * Propagate a track's position covariance from field (meters) into pixels
     * via central finite differences against [Projection.forwardProject].
     * Matches the Jacobian style used in `Projection.projectToGroundWithCovariance`.
     */
    private fun reprojectCovToPixels(
        x: Double, y: Double, z: Double,
        K: Intrinsics, T_FC: Matrix4d,
        P: DoubleArray,
    ): List<Double>? {
        val delta = 0.01  // 1 cm
        val cx = Projection.forwardProject(Vector3d(x + delta, y, z), K, T_FC) ?: return null
        val mx = Projection.forwardProject(Vector3d(x - delta, y, z), K, T_FC) ?: return null
        val cy = Projection.forwardProject(Vector3d(x, y + delta, z), K, T_FC) ?: return null
        val my = Projection.forwardProject(Vector3d(x, y - delta, z), K, T_FC) ?: return null

        val duDx = (cx[0] - mx[0]) / (2.0 * delta)
        val dvDx = (cx[1] - mx[1]) / (2.0 * delta)
        val duDy = (cy[0] - my[0]) / (2.0 * delta)
        val dvDy = (cy[1] - my[1]) / (2.0 * delta)

        // P_pos in field (2x2) = top-left of P.
        val p00 = P[0]; val p01 = P[1]
        val p10 = P[4]; val p11 = P[5]

        // J = [[duDx, duDy],[dvDx, dvDy]].
        // Cov_px = J * P_pos * J^T  (row-major entries a00, a01, a10, a11)
        val jp00 = duDx * p00 + duDy * p10
        val jp01 = duDx * p01 + duDy * p11
        val jp10 = dvDx * p00 + dvDy * p10
        val jp11 = dvDx * p01 + dvDy * p11
        val a00 = jp00 * duDx + jp01 * duDy
        val a01 = jp00 * dvDx + jp01 * dvDy
        val a10 = jp10 * duDx + jp11 * duDy
        val a11 = jp10 * dvDx + jp11 * dvDy
        return listOf(a00, a01, a10, a11)
    }
}
