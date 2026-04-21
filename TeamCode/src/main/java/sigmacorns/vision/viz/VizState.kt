package sigmacorns.vision.viz

/**
 * Everything the browser needs to render the ball tracker overlays.
 *
 * The server builds one of these each tick from [sigmacorns.vision.tracker.Tracker]
 * + [sigmacorns.vision.sim.SimulatedCamera] + ground-truth ball positions, and
 * attaches it to the SimVizServer broadcast payload under a `"tracker"` key.
 *
 * Kept small and JSON-friendly (lists and primitives only — no JOML types). The
 * browser reconstructs geometry on its side.
 */
data class TrackerVizState(
    /** Pretty label for the HUD. "live", "pixelAndPoseNoise", etc. */
    val scenario: String,

    /** Sim-time RMS position error in meters for the target track vs. truth, or null. */
    val rmsErrorM: Double? = null,

    val camera: CameraViz,
    val ballTruth: List<BallTruthViz>,
    val detectionsPx: List<PixelDetViz>,
    val detectionsField: List<FieldDetViz>,
    val tracks: List<TrackViz>,
    val targetId: Int? = null,
)

/**
 * Robot-camera extrinsics + intrinsics needed to render the pixel-space panel
 * and draw the camera frustum on the 3D field.
 *
 *   origin       = camera position in field frame (meters)
 *   Rfc          = R_field_camera — 3x3 row-major, maps a camera-frame vector
 *                  to field frame. The three columns are the camera axes.
 *                  Camera axes follow OpenCV: +X right, +Y down, +Z forward.
 *   frustumGround = polygon (field frame) where the camera's FOV intersects
 *                  the floor plane (z=0). Closed implicitly; 4 corners for
 *                  rectangular FOV but clients should just draw the polyline.
 */
data class CameraViz(
    val origin: List<Double>,
    val Rfc: List<Double>,
    val fx: Double, val fy: Double, val cx: Double, val cy: Double,
    val width: Int, val height: Int,
    val intakeMaskYMinFrac: Double,
    val frustumGround: List<List<Double>>,
)

/** Ground-truth ball — rendered as a translucent marker on both views. */
data class BallTruthViz(
    val x: Double, val y: Double, val z: Double,
    /** Pre-projected pixel coords for convenience; null if outside the image. */
    val u: Double? = null,
    val v: Double? = null,
)

/** A raw pixel detection this frame (before any gating). */
data class PixelDetViz(val u: Double, val v: Double)

/**
 * A detection that survived gating, in field coordinates, with its 2x2
 * measurement covariance (row-major).
 */
data class FieldDetViz(
    val x: Double, val y: Double,
    val cov: List<Double>,
)

/** A live tracked object. */
data class TrackViz(
    val id: Int,
    val x: Double, val y: Double,
    val vx: Double, val vy: Double,
    /** Position covariance (row-major 2x2). */
    val cov: List<Double>,
    val hits: Int,
    val confirmed: Boolean,
    /** Pre-projected pixel coords for the camera panel, null if off-image. */
    val u: Double? = null,
    val v: Double? = null,
    /**
     * Covariance re-projected into pixel space via the same Jacobian the
     * tracker uses. Row-major 2x2. Null if off-image.
     */
    val covPx: List<Double>? = null,
)
