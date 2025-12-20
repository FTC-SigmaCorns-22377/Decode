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
    external fun step(simPtr: Long, dt: Double, inputs: DoubleArray)
    external fun getState(simPtr: Long): DoubleArray
    external fun spawnBall(simPtr: Long, x: Double, y: Double, z: Double)
    external fun setPosition(simPtr: Long, x: Double, y: Double, yaw: Double)
    external fun destroySim(simPtr: Long)
}