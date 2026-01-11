package sigmacorns.control.aim

import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.io.RerunLogging
import sigmacorns.math.Pose2d
import kotlin.math.cos
import kotlin.math.sin

class GTSAMDebugLogger(
    val recordingId: String = "gtsam_debug",
    val savePath: String = "/sdcard/FIRST/kotlin_gtsam.rrd"
) : AutoCloseable {
    private var rr: RerunLogging? = null
    private val robotPath = mutableListOf<Vector3d>()
    private val fusedPath = mutableListOf<Vector3d>()

    fun enable() {
        rr = RerunLogging.save(recordingId, savePath)
    }

    // Log raw odometry from Pinpoint (before transformation)
    fun logRawOdometry(pose: Pose2d, velocity: Pose2d, timestamp: Double) {
        rr?.apply {
            logScalar("kotlin/raw_odom/x", pose.v.x)
            logScalar("kotlin/raw_odom/y", pose.v.y)
            logScalar("kotlin/raw_odom/theta", pose.rot)
            logScalar("kotlin/raw_odom/vx", velocity.v.x)
            logScalar("kotlin/raw_odom/vy", velocity.v.y)
            
            val pos = Vector3d(pose.v.x, pose.v.y, 0.0)
            logPoints3D("kotlin/raw_odom/position", listOf(pos), 0xFFAAAAAA.toInt(), 0.05f)
            
            updatePath(robotPath, pos, "kotlin/raw_odom/path")
        }
    }

    // Log odometry delta after local frame transformation
    fun logOdometryDelta(dxLocal: Double, dyLocal: Double, dtheta: Double,
                         dxGlobal: Double, dyGlobal: Double, prevTheta: Double) {
        rr?.apply {
            logScalar("kotlin/odom_delta/dx_local", dxLocal)
            logScalar("kotlin/odom_delta/dy_local", dyLocal)
            logScalar("kotlin/odom_delta/dtheta", dtheta)
            logScalar("kotlin/odom_delta/dx_global", dxGlobal)
            logScalar("kotlin/odom_delta/dy_global", dyGlobal)
            logScalar("kotlin/odom_delta/prev_theta", prevTheta)
        }
    }

    // Log raw tag detection from Limelight
    fun logTagDetection(tagId: Int, corners: List<Vector2d>, turretAngle: Double) {
        rr?.apply {
            logPoints2D("kotlin/vision/tag_${tagId}/corners", corners, 0xFF00FF00.toInt())
            logScalar("kotlin/vision/tag_${tagId}/turret_angle", turretAngle)
        }
    }

    // Log landmark configuration
    fun logLandmarks(landmarks: Map<Int, AutoAimGTSAM.LandmarkSpec>) {
        rr?.apply {
            val positions = landmarks.map { (_, spec) -> spec.position }
            logPoints3D("kotlin/landmarks", positions, 0xFFFF8800.toInt())

            val cornerPoints = mutableListOf<Vector3d>()
            landmarks.forEach { (_, spec) ->
                val q = eulerToQuaternion(spec.roll, spec.pitch, spec.yaw)
                val halfSize = spec.size / 2.0
                // Assuming standard AprilTag frame: Z out, X right, Y down? Or similar.
                // Corners: (-x, -y), (x, -y), (x, y), (-x, y)
                // Let's assume corners are in XY plane of the tag.
                val corners = listOf(
                    Vector3d(-halfSize, -halfSize, 0.0),
                    Vector3d(halfSize, -halfSize, 0.0),
                    Vector3d(halfSize, halfSize, 0.0),
                    Vector3d(-halfSize, halfSize, 0.0)
                )
                
                corners.forEach { corner ->
                    // Rotate and translate
                    val rotated = rotate(corner, q)
                    rotated.add(spec.position)
                    cornerPoints.add(rotated)
                }
            }
            logPoints3D("kotlin/landmarks_corners", cornerPoints, 0xFFFFFF00.toInt(), 0.02f)
        }
    }

    // Log fused pose output from C++ and transforms
    fun logFusedPose(pose: Pose2d) {
        rr?.apply {
            logScalar("kotlin/fused/x", pose.v.x)
            logScalar("kotlin/fused/y", pose.v.y)
            logScalar("kotlin/fused/theta", pose.rot)
            
            // Log robot frame transform
            logTransform("world/robot", 
                floatArrayOf(pose.v.x.toFloat(), pose.v.y.toFloat(), 0f),
                axisAngleToQuaternion(0f, 0f, 1f, pose.rot.toFloat())
            )

            val pos = Vector3d(pose.v.x, pose.v.y, 0.0)
            updatePath(fusedPath, pos, "kotlin/fused/path")
        }
    }
    
    fun logReprojectionError(tagId: Int, error: Double) {
        rr?.logScalar("kotlin/vision/tag_${tagId}/reprojection_error", error)
    }

    private fun updatePath(path: MutableList<Vector3d>, newPos: Vector3d, rerunPath: String) {
        if (path.isEmpty() || path.last().distance(newPos) > 0.01) {
            path.add(Vector3d(newPos))
            if (path.size > 1000) path.removeAt(0)
            rr?.logLineStrip(rerunPath, path)
        }
    }
    
    fun logPredictedCorners(tagId: Int, corners: List<Vector2d>) {
        rr?.apply {
            logPoints2D("world/measurements/tag_${tagId}/predicted_corners", corners, 0xFFFF0000.toInt()) // Red
        }
    }
    
    fun logCoordinateFrames(robotPose: Pose2d, turretYaw: Double, cameraOffset: Vector3d, cameraRoll: Double, cameraPitch: Double, cameraYaw: Double) {
        rr?.apply {
            // Turret frame relative to robot (world/robot/turret)
            logTransform("world/robot/turret",
                floatArrayOf(0f, 0f, 0f),
                axisAngleToQuaternion(0f, 0f, 1f, turretYaw.toFloat())
            )
            
            // Camera frame relative to turret (world/robot/turret/camera)
            val qCam = eulerToQuaternion(cameraRoll, cameraPitch, cameraYaw)
            logTransform("world/robot/turret/camera",
                floatArrayOf(cameraOffset.x.toFloat(), cameraOffset.y.toFloat(), cameraOffset.z.toFloat()),
                floatArrayOf(qCam.x.toFloat(), qCam.y.toFloat(), qCam.z.toFloat(), qCam.w.toFloat())
            )

            logScalar("kotlin/turret/yaw", turretYaw)

            // Calculate global positions for validation
            val robotCenter = Vector3d(robotPose.v.x, robotPose.v.y, 0.0)
            logPoints3D("kotlin/debug/robot_center_fused", listOf(robotCenter), 0xFF00FFFF.toInt(), 0.05f)

            // Robot Rotation
            val qRobot = eulerToQuaternion(0.0, 0.0, robotPose.rot)
            
            // Turret Rotation (relative to robot)
            val qTurret = eulerToQuaternion(0.0, 0.0, turretYaw)
            
            // Combined rotation: qRobot * qTurret * qCam
            // Position: robotPos + qRobot * (turretPos + qTurret * cameraPos)
            // turretPos is 0.
            
            // World Rotation of Camera
            // qTotal = qRobot * qTurret * qCam
            val qRobotTurret = multiply(qRobot, qTurret)
            val qTotal = multiply(qRobotTurret, qCam)
            
            // World Position of Camera
            // pCamWorld = pRobot + qRobot * (pTurret + qTurret * pCamLocal)
            // pTurret = 0
            // pCamLocal = cameraOffset
            
            val camOffsetRotatedByTurret = rotate(Vector3d(cameraOffset), qTurret)
            val camOffsetRotatedByRobot = rotate(camOffsetRotatedByTurret, qRobot)
            val camWorldPos = Vector3d(robotCenter).add(camOffsetRotatedByRobot)
            
            logPoints3D("kotlin/debug/camera_center_fused", listOf(camWorldPos), 0xFFFF00FF.toInt(), 0.05f)
            
            // Log Camera Vectors (Forward, Up, Right)
            // Assuming Camera frame: Z forward, Y down, X right? Or Z forward, Y up, X right?
            // Standard standard is usually: -Z forward?
            // Let's assume standard definitions: Forward = (0,0,1) or (1,0,0)?
            // In robotics often X is forward. In camera frames often Z is forward.
            // Let's visualize Unit Z, Unit Y, Unit X.
            
            val axisLen = 0.5
            val fwd = rotate(Vector3d(0.0, 0.0, axisLen), qTotal).add(camWorldPos)
            val right = rotate(Vector3d(axisLen, 0.0, 0.0), qTotal).add(camWorldPos)
            val up = rotate(Vector3d(0.0, -axisLen, 0.0), qTotal).add(camWorldPos) // -Y up if Y is down?
            
            logLineStrip("kotlin/debug/camera_axis_z", listOf(camWorldPos, fwd))
            logLineStrip("kotlin/debug/camera_axis_x", listOf(camWorldPos, right))
            logLineStrip("kotlin/debug/camera_axis_y", listOf(camWorldPos, up))
        }
    }
    
    private fun axisAngleToQuaternion(ax: Float, ay: Float, az: Float, angle: Float): FloatArray {
        val halfAngle = angle / 2.0f
        val s = sin(halfAngle)
        return floatArrayOf(ax * s, ay * s, az * s, cos(halfAngle))
    }
    
    private data class Quat(val x: Double, val y: Double, val z: Double, val w: Double)
    
    private fun eulerToQuaternion(roll: Double, pitch: Double, yaw: Double): Quat {
        val cy = cos(yaw * 0.5)
        val sy = sin(yaw * 0.5)
        val cp = cos(pitch * 0.5)
        val sp = sin(pitch * 0.5)
        val cr = cos(roll * 0.5)
        val sr = sin(roll * 0.5)

        val w = cr * cp * cy + sr * sp * sy
        val x = sr * cp * cy - cr * sp * sy
        val y = cr * sp * cy + sr * cp * sy
        val z = cr * cp * sy - sr * sp * cy
        return Quat(x, y, z, w)
    }
    
    private fun multiply(q1: Quat, q2: Quat): Quat {
        val w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        val x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        val y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        val z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        return Quat(x, y, z, w)
    }

    private fun rotate(v: Vector3d, q: Quat): Vector3d {
        // v' = q * v * q_inv
        // Using formula: v + 2*cross(q.xyz, cross(q.xyz, v) + q.w*v)
        
        val qv = Vector3d(q.x, q.y, q.z)
        val t = Vector3d()
        qv.cross(v, t)
        
        val u = Vector3d(v)
        u.mul(q.w)
        u.add(t) // (q.w * v + cross)
        
        val res = Vector3d()
        qv.cross(u, res) // cross(q.xyz, ...)
        res.mul(2.0)
        
        res.add(v)
        return res
    }

    override fun close() { rr?.close() }
}
