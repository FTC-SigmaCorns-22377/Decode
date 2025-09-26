package sigmacorns.io

import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.State

class RerunLogging private constructor(
    val name: String,
): AutoCloseable {
    init {
        System.loadLibrary("rerun")
    }

    private external fun connect(name: String, url: String): Long
    private external fun save(name: String, path: String): Long
    private external fun destroy(rec: Long)
    private external fun logState(connection: Long, data: FloatArray)
    private external fun logInput(connection: Long, data: FloatArray)
    private external fun logLineStrip3D(connection: Long, name: String, data: FloatArray)

    private var ptr: Long = 0

    companion object {
        fun connect(name: String, url: String): RerunLogging {
            val rr = RerunLogging(name)
            rr.ptr = rr.connect(name, url)

            require(rr.ptr != 0L)
            return rr
        }

        fun save(name: String, path: String): RerunLogging {
            val rr = RerunLogging(name)
            rr.ptr = rr.save(name, path)

            require(rr.ptr != 0L)
            return rr
        }
    }

    fun logState(state: State) {
        logState(ptr,state.toFloatArray())
    }

    fun logLineStrip(name: String, points: List<Vector3d>) {
        logLineStrip3D(ptr,name, FloatArray(points.size*3) {
            val i = it/3
            val d = if(it%3==0)
                points[i].x
            else if(it%3==1)
                points[i].y
            else
                points[i].z
            d.toFloat()
        })
    }

    fun logLineStrip(name: String, points: List<Vector2d>, z: Number = 0f) {
        logLineStrip3D(ptr,name, FloatArray(points.size*3) {
            val i = it/3
            val d = if(it%3==0)
                points[i].x
            else if(it%3==1)
                points[i].y
            else
                z.toFloat()
            d.toFloat()
        })
    }

    override fun close() {
        require(ptr != 0L)

        destroy(ptr)
        ptr = 0
    }
}