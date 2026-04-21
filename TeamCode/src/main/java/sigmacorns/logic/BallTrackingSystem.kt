package sigmacorns.logic

import org.joml.Vector2d
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.KalmanTrack
import sigmacorns.vision.tracker.PixelDetection
import sigmacorns.vision.tracker.PoseBuffer
import sigmacorns.vision.tracker.SelectionStrategy
import sigmacorns.vision.tracker.Tracker
import sigmacorns.vision.tracker.TrackerConfig

/**
 * Opmode-side wrapper around [Tracker] and [PoseBuffer].
 *
 *   update(t):
 *     1. push io.position() into the pose buffer at the current loop timestamp.
 *     2. pull detections from io.getBallDetections(t, pose).
 *     3. interpolate pose at each detection's capture timestamp (falling back
 *        to the live pose if the buffer doesn't bracket it yet).
 *     4. tracker.tick(detections, poseAtCapture, t).
 *     5. refresh [target].
 *
 * Mirrors the pattern `AimingSystem` uses for its vision pipeline. The Robot
 * reference is avoided — the system only needs an [SigmaIO]. That lets the
 * sim harness construct it without the full subsystem graph, and the real
 * opmode still wires it through [io].
 */
class BallTrackingSystem(
    val config: TrackerConfig,
    private val io: SigmaIO,
    private val strategy: SelectionStrategy = SelectionStrategy.Closest,
    private val poseBufferCapacity: Int = 500,
) {
    val tracker: Tracker = Tracker(config)
    val poseBuffer: PoseBuffer = PoseBuffer(capacity = poseBufferCapacity)

    /** Most recent pixel detections read from IO. Useful for viz / debug. */
    var lastDetectionsPx: List<PixelDetection> = emptyList()
        private set

    /** Currently-selected target track, or null. */
    var target: KalmanTrack? = null
        private set

    /** Target field position (x, y, heading = atan2(y - robot.y, x - robot.x)). */
    var targetBallField: Pose2d? = null
        private set

    /**
     * One tick of the pipeline. `t` must be monotonically non-decreasing and
     * match the robot's loop clock (seconds).
     */
    fun update(t: Double) {
        val pose = io.position()
        poseBuffer.add(t, pose)

        val detections = io.getBallDetections(t, pose)
        lastDetectionsPx = detections

        // Pose-at-capture for each detection; fall back to live pose when the
        // buffer hasn't accumulated bracketing samples.
        val poseAtCapture: Pose2d =
            detections.firstOrNull()?.t?.let { poseBuffer.get(it) } ?: pose

        tracker.tick(detections, poseAtCapture, t)

        val newTarget = tracker.selectTarget(pose, t, strategy)
        target = newTarget

        targetBallField = newTarget?.let { tr ->
            val p = tr.positionAt(t)
            val heading = kotlin.math.atan2(p.y - pose.v.y, p.x - pose.v.x)
            Pose2d(Vector2d(p.x, p.y), heading)
        }
    }

    /** Clear all tracks and pose history. Call before reset/respawn events. */
    fun reset() {
        tracker.reset()
        poseBuffer.clear()
        lastDetectionsPx = emptyList()
        target = null
        targetBallField = null
    }
}
