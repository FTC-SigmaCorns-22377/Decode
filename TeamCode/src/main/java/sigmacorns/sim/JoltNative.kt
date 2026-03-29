package sigmacorns.sim

object JoltNative {
    init {
        System.loadLibrary("jolt_sim_jni")
    }

    @JvmStatic external fun nativeCreate(): Long
    @JvmStatic external fun nativeDestroy(handle: Long)
    @JvmStatic external fun nativeStep(handle: Long, dt: Float)
    @JvmStatic external fun nativeApplyRobotForce(handle: Long, fx: Float, fy: Float, tz: Float)
    @JvmStatic external fun nativeSetRobotPose(handle: Long, x: Float, y: Float, theta: Float)
    @JvmStatic external fun nativeGetRobotState(handle: Long, out: FloatArray)
    @JvmStatic external fun nativeSpawnBall(handle: Long, x: Float, y: Float, z: Float,
                                            vx: Float, vy: Float, vz: Float, color: Int): Int
    @JvmStatic external fun nativeSpawnShotBall(handle: Long, x: Float, y: Float, z: Float,
                                                 vx: Float, vy: Float, vz: Float, color: Int): Int
    @JvmStatic external fun nativeRemoveBall(handle: Long, index: Int)
    @JvmStatic external fun nativeGetBallCount(handle: Long): Int
    @JvmStatic external fun nativeGetBallStates(handle: Long, out: FloatArray)
    @JvmStatic external fun nativeGetBallColors(handle: Long, out: IntArray)
    @JvmStatic external fun nativeSetIntakeRollerOmega(handle: Long, omega: Float)
    @JvmStatic external fun nativeGetIntakeState(handle: Long, out: FloatArray)
    @JvmStatic external fun nativeGetPendingPickups(handle: Long, out: IntArray, max: Int): Int
    @JvmStatic external fun nativeCollectPickups(handle: Long, out: IntArray, max: Int): Int
    @JvmStatic external fun nativeGetGoalStates(handle: Long, out: FloatArray)
    @JvmStatic external fun nativeGetIntakeOverlaps(handle: Long, out: IntArray, max: Int): Int
}
