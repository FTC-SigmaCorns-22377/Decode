package sigmacorns.sim

import org.joml.Matrix4d
import org.joml.Vector3d
import sigmacorns.control.aim.AutoAimGTSAM
import sigmacorns.control.aim.VisionFrame
import sigmacorns.control.aim.VisionObservation
import sigmacorns.control.aim.VisionResult
import sigmacorns.control.aim.VisionStatus
import sigmacorns.math.Pose2d
import sigmacorns.sim.viz.Point2D
import sigmacorns.sim.viz.Point3D
import sigmacorns.sim.viz.SimTagDetection
import sigmacorns.sim.viz.SimVisionState
import java.util.Random

/**
 * Simulates AprilTag camera readings by projecting known tag positions
 * through a pinhole camera model with injected Gaussian noise.
 *
 * Projection pipeline:
 *   1. Compute camera pose in world frame from robot pose, turret angle, and camera extrinsics
 *   2. Transform tag corners from tag-local to world frame
 *   3. Transform world corners to camera frame
 *   4. Perspective projection + optional Brown-Conrady distortion
 *   5. Convert to pixel coordinates via intrinsics
 *   6. Add Gaussian noise
 */
class AprilTagSimulator(
    private val landmarks: Map<Int, AutoAimGTSAM.LandmarkSpec>,
    private val config: AutoAimGTSAM.EstimatorConfig = AutoAimGTSAM.EstimatorConfig()
) {
    private val random = Random()
    private val imageWidth = (config.cx * 2).toInt()
    private val imageHeight = (config.cy * 2).toInt()

    /** Last simulation result for visualization (updated on each simulate call). */
    @Volatile var lastVizState: SimVisionState? = null
        private set

    /**
     * Simulate vision readings given current robot state.
     * @param robotPose Robot pose in field frame (x, y, theta)
     * @param turretYawRad Turret yaw angle relative to robot body (radians)
     * @param timestampSeconds Current simulation timestamp
     * @return VisionResult matching the real VisionTracker interface
     */
    fun simulate(
        robotPose: Pose2d,
        turretYawRad: Double,
        timestampSeconds: Double
    ): VisionResult {
        // Compute camera pose in world frame
        val cameraPoseM = computeCameraPose(robotPose, turretYawRad)
        val worldToCamera = Matrix4d(cameraPoseM).invert()

        // Extract camera position for viz
        val cameraPos = Vector3d()
        cameraPoseM.getTranslation(cameraPos)

        val observations = mutableListOf<VisionObservation>()
        val vizDetections = mutableListOf<SimTagDetection>()

        for ((tagId, landmark) in landmarks) {
            val result = projectTag(tagId, landmark, worldToCamera)
            if (result != null) {
                observations.add(result.observation)
                vizDetections.add(result.vizDetection)
            }
        }

        // Store viz state for broadcast
        lastVizState = SimVisionState(
            cameraX = cameraPos.x,
            cameraY = cameraPos.y,
            cameraZ = cameraPos.z,
            detections = vizDetections,
            imageWidth = imageWidth,
            imageHeight = imageHeight
        )

        val status = VisionStatus(
            lastResultValid = true,
            detectedTagCount = observations.size,
            hasVisionTarget = observations.isNotEmpty(),
            trackedTagId = observations.firstOrNull()?.tagId ?: -1,
            rawTxDegrees = 0.0,
            rawTyDegrees = 0.0,
            lastDetectionTimeMs = (timestampSeconds * 1000).toLong()
        )

        val frame = if (observations.isNotEmpty()) {
            VisionFrame(timestampSeconds, observations)
        } else null

        return VisionResult(status, frame)
    }

    private data class ProjectionResult(
        val observation: VisionObservation,
        val vizDetection: SimTagDetection
    )

    private fun projectTag(
        tagId: Int,
        landmark: AutoAimGTSAM.LandmarkSpec,
        worldToCamera: Matrix4d
    ): ProjectionResult? {
        val s = landmark.size / 2.0

        // Tag corners in local frame: TL, TR, BR, BL
        val localCorners = arrayOf(
            Vector3d(-s, -s, 0.0),
            Vector3d( s, -s, 0.0),
            Vector3d( s,  s, 0.0),
            Vector3d(-s,  s, 0.0)
        )

        // Tag pose in world frame
        // LandmarkSpec uses aerospace convention: roll = X rotation,
        // pitch = Y rotation, yaw = Z rotation. Reconstruct as Rz(yaw) * Ry(pitch) * Rx(roll).
        val tagPoseM = Matrix4d()
            .translate(landmark.position.x, landmark.position.y, landmark.position.z)
            .rotateZ(landmark.yaw)
            .rotateY(landmark.pitch)
            .rotateX(landmark.roll)

        val corners = DoubleArray(8)
        val worldCorners = mutableListOf<Point3D>()
        val pixelCorners = mutableListOf<Point2D>()

        for (i in 0 until 4) {
            // Transform corner to world frame
            val worldCorner = tagPoseM.transformPosition(Vector3d(localCorners[i]))
            worldCorners.add(Point3D(worldCorner.x, worldCorner.y, worldCorner.z))

            // Transform to camera frame
            val camPoint = worldToCamera.transformPosition(Vector3d(worldCorner))

            // Point must be in front of camera
            if (camPoint.z <= 0.01) return null

            // Perspective projection
            val x = camPoint.x / camPoint.z
            val y = camPoint.y / camPoint.z

            // Brown-Conrady distortion model
            val r2 = x * x + y * y
            val r4 = r2 * r2
            val r6 = r4 * r2
            val radial = 1.0 + config.k1 * r2 + config.k2 * r4 + config.k3 * r6
            val xd = x * radial + 2.0 * config.p1 * x * y + config.p2 * (r2 + 2.0 * x * x)
            val yd = y * radial + config.p1 * (r2 + 2.0 * y * y) + 2.0 * config.p2 * x * y

            // Pixel coordinates
            val u = config.fx * xd + config.cx
            val v = config.fy * yd + config.cy

            // All 4 corners must be within image bounds
            if (u < 0 || u >= imageWidth || v < 0 || v >= imageHeight) return null

            pixelCorners.add(Point2D(u, v))

            // Inject Gaussian noise
            corners[i * 2]     = u + random.nextGaussian() * config.defaultPixelSigma
            corners[i * 2 + 1] = v + random.nextGaussian() * config.defaultPixelSigma
        }

        val observation = VisionObservation(
            tagId = tagId,
            corners = corners,
            txDeg = 0.0,
            tyDeg = 0.0
        )

        val vizDetection = SimTagDetection(
            tagId = tagId,
            corners = worldCorners,
            pixelCorners = pixelCorners
        )

        return ProjectionResult(observation, vizDetection)
    }

    /**
     * Compute camera pose (world-to-camera-origin transform) as a 4x4 matrix.
     *
     * Chain: robotPose * turretRotation * cameraExtrinsics
     * Matches the GTSAM convention: robot_pose.compose(turret_pose).compose(extrinsics)
     */
    private fun computeCameraPose(robotPose: Pose2d, turretYawRad: Double): Matrix4d {
        return Matrix4d()
            .translate(robotPose.v.x, robotPose.v.y, 0.0)
            .rotateZ(robotPose.rot)
            .rotateZ(turretYawRad)
            .translate(config.cameraOffsetX, config.cameraOffsetY, config.cameraOffsetZ)
            .rotateZ(config.cameraYaw)
            .rotateY(config.cameraPitch)
            .rotateX(config.cameraRoll)
    }
}
