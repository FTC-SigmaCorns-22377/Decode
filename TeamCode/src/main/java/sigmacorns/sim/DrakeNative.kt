package sigmacorns.sim

object DrakeNative {
    init {
        try {
            System.loadLibrary("native-lib")
        } catch (e: UnsatisfiedLinkError) {
            System.err.println("Failed to load native-lib: ${e.message}")
            // Fallback or handle appropriately. For sim, we might want to throw or ignore if on device without libs.
        }
    }

    external fun createSim(urdfPath: String): Long
    external fun setMecanumParameters(simPtr: Long, params: DoubleArray)
    external fun setMotorParameters(simPtr: Long, params: DoubleArray)
    external fun step(simPtr: Long, dt: Double, inputs: DoubleArray)
    external fun getState(simPtr: Long): DoubleArray
    external fun spawnBall(simPtr: Long, x: Double, y: Double, z: Double)
    external fun spawnBallWithVelocity(simPtr: Long, x: Double, y: Double, z: Double,
                                        vx: Double, vy: Double, vz: Double)
    external fun removeBall(simPtr: Long, index: Int)
    external fun getIntakeContacts(simPtr: Long): IntArray
    external fun setPosition(simPtr: Long, x: Double, y: Double, yaw: Double)
    external fun destroySim(simPtr: Long)
}