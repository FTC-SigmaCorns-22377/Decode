package sigmacorns.vision.tracker

import org.joml.Matrix4d
import org.joml.Vector3d
import sigmacorns.math.Pose2d
import kotlin.math.cos
import kotlin.math.sin

/**
 * Coordinate frames used by the ball tracker. All `T_AB` matrices are 4x4
 * homogeneous transforms that take a point expressed in frame B and return
 * its coordinates in frame A. JOML matrices are column-major.
 *
 * F (Field):
 *   Origin at one field corner, +X right, +Y up (in the field plane), +Z
 *   out of the floor. Matches sigmacorns.math.Pose2d (heading CCW from +X).
 *
 * R (Robot):
 *   +X forward, +Y left, +Z up. Origin at the chassis geometric center.
 *   Matches sigmacorns.constants.RobotModelConstants.
 *
 * C (Camera):
 *   OpenCV. +X image-right, +Y image-down, +Z optical-axis-forward.
 *
 * EULER ORDER for buildTRC(camPosR, pitchDown, yaw, roll):
 *   R_RC = R_RC_base · R_x(-pitchDown) · R_y(-yaw) · R_z(roll)
 *
 * R_RC_base maps camera-OpenCV axes into robot axes when pitchDown = yaw = roll = 0:
 *   camera +X (right) -> robot -Y
 *   camera +Y (down)  -> robot -Z
 *   camera +Z (fwd)   -> robot +X
 *
 * The pitchDown / yaw signs are NEGATED inside buildTRC because the rotation is
 * applied in the camera's local frame, where +Y is image-DOWN (= robot -Z) and the
 * intuitive "tilt down" / "yaw left" senses are right-handed about the OPPOSITE
 * camera-local axis from the robot-frame axis a human would visualize.
 *
 * Sign conventions (as observed in robot frame):
 *   pitchDown > 0  ->  optical axis tilts toward the floor.
 *   yaw       > 0  ->  optical axis swings toward the robot's left (+Y robot).
 *   roll      > 0  ->  image rotates clockwise as viewed from the scene.
 */
object Frames {

    /**
     * Robot-from-Camera transform. Composed as:
     *   T_RC = translate(camPosR) · R_RC_base · R_x(pitchDown) · R_y(yaw) · R_z(roll)
     */
    fun buildTRC(
        camPosR: Vector3d,
        pitchDown: Double,
        yaw: Double,
        roll: Double,
    ): Matrix4d {
        val baseR = Matrix4d().apply {
            // column-major fill via setters; columns = where camera +X/+Y/+Z land in robot frame.
            m00( 0.0); m01(-1.0); m02( 0.0); m03(0.0)   // camera +X -> (0, -1, 0)
            m10( 0.0); m11( 0.0); m12(-1.0); m13(0.0)   // camera +Y -> (0,  0,-1)
            m20( 1.0); m21( 0.0); m22( 0.0); m23(0.0)   // camera +Z -> (1,  0, 0)
            m30( 0.0); m31( 0.0); m32( 0.0); m33(1.0)
        }
        return Matrix4d()
            .translate(camPosR.x, camPosR.y, camPosR.z)
            .mul(baseR)
            .rotateX(-pitchDown)
            .rotateY(-yaw)
            .rotateZ(roll)
    }

    /**
     * Field-from-Robot transform from a 2D pose. Heading rotates CCW about
     * the field +Z axis; the robot is placed at z=0 in field frame.
     */
    fun buildTFR(robotPose: Pose2d): Matrix4d {
        val cs = cos(robotPose.rot)
        val sn = sin(robotPose.rot)
        return Matrix4d().apply {
            m00( cs); m01( sn); m02(0.0); m03(0.0)
            m10(-sn); m11( cs); m12(0.0); m13(0.0)
            m20(0.0); m21(0.0); m22(1.0); m23(0.0)
            m30(robotPose.v.x); m31(robotPose.v.y); m32(0.0); m33(1.0)
        }
    }

    /** Field-from-Camera composition: T_FC = T_FR · T_RC. */
    fun buildTFC(robotPose: Pose2d, T_RC: Matrix4d): Matrix4d =
        Matrix4d(buildTFR(robotPose)).mul(T_RC)
}

/** Camera mount in robot frame. Loaded from config/ball_tracker.json into TrackerConfig. */
data class CameraExtrinsicsR(
    val camPosR: Vector3d,
    val pitchDownRad: Double,
    val yawRad: Double,
    val rollRad: Double,
) {
    fun toTRC(): Matrix4d = Frames.buildTRC(camPosR, pitchDownRad, yawRad, rollRad)
}
