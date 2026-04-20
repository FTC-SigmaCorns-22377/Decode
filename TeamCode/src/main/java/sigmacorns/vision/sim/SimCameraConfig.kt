package sigmacorns.vision.sim

import sigmacorns.vision.tracker.CameraExtrinsicsR
import sigmacorns.vision.tracker.Intrinsics

/**
 * Everything `SimulatedCamera` needs to forward-project balls into pixel
 * detections with noise, dropouts, and false positives. Extracted from
 * `TrackerConfig` so sim-only knobs live here, not in the tracker's config.
 */
data class SimCameraConfig(
    val intrinsics: Intrinsics,
    val imageWidthPx: Int,
    val imageHeightPx: Int,
    val extrinsics: CameraExtrinsicsR,

    /** Gaussian pixel noise stdev added to each detection. */
    val sigmaPxSim: Double = 0.0,

    /** Per-frame per-ball probability of skipping a visible detection. */
    val pDrop: Double = 0.0,

    /** Per-frame probability of emitting one uniform-random false positive. */
    val pFalsePositive: Double = 0.0,

    /** RNG seed — deterministic by default for reproducible tests. */
    val rngSeed: Long = 42L,
)
