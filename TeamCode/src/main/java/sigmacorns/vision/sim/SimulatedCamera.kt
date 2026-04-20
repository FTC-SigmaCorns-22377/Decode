package sigmacorns.vision.sim

import org.joml.Matrix4d
import org.joml.Vector3d
import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.Frames
import sigmacorns.vision.tracker.PixelDetection
import sigmacorns.vision.tracker.Projection
import java.util.Random

/**
 * Forward-projects a set of known field-frame ball positions through a
 * simulated camera to produce `PixelDetection`s — the same shape the tracker
 * consumes from the real Limelight pipeline. Deterministic given a seed.
 *
 *   frame(ballsField, t, robotPose) ->
 *     for each ball b_F:
 *       b_C = T_FC^{-1} * b_F
 *       skip if b_C.z <= 0                       (behind camera)
 *       u = fx * b_C.x / b_C.z + cx; v = fy * b_C.y / b_C.z + cy
 *       apply forward distortion model            (OpenCV k1..k3, p1..p2)
 *       skip if outside image bounds
 *       with probability pDrop, skip
 *       else add Gaussian pixel noise and emit
 *     with probability pFalsePositive, emit one uniform-random (u, v)
 */
class SimulatedCamera(val config: SimCameraConfig) {

    private val rng = Random(config.rngSeed)
    private val T_RC: Matrix4d = config.extrinsics.toTRC()

    /**
     * @param ballsField field-frame ball positions (meters). z is the ball center z;
     *        for DECODE artifacts that's the ball radius. `SimulatedCamera` does NOT
     *        know about ball radius — pass positions exactly where you want to project.
     */
    fun frame(
        ballsField: List<Vector3d>,
        t: Double,
        robotPose: Pose2d,
    ): List<PixelDetection> {
        val T_FC = Frames.buildTFC(robotPose, T_RC)
        val K = config.intrinsics

        val out = ArrayList<PixelDetection>(ballsField.size + 1)
        for (b in ballsField) {
            val uv = Projection.forwardProject(b, K, T_FC) ?: continue
            val u = uv[0]
            val v = uv[1]
            if (u < 0.0 || v < 0.0 || u > config.imageWidthPx || v > config.imageHeightPx) continue
            if (config.pDrop > 0.0 && rng.nextDouble() < config.pDrop) continue
            val noiseU = if (config.sigmaPxSim > 0.0) rng.nextGaussian() * config.sigmaPxSim else 0.0
            val noiseV = if (config.sigmaPxSim > 0.0) rng.nextGaussian() * config.sigmaPxSim else 0.0
            out.add(PixelDetection(u + noiseU, v + noiseV, t))
        }

        if (config.pFalsePositive > 0.0 && rng.nextDouble() < config.pFalsePositive) {
            val fpU = rng.nextDouble() * config.imageWidthPx
            val fpV = rng.nextDouble() * config.imageHeightPx
            out.add(PixelDetection(fpU, fpV, t))
        }
        return out
    }
}
