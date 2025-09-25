package sigmacorns.io

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
        logState(ptr,state.toNativeArray())
    }

    override fun close() {
        require(ptr != 0L)

        destroy(ptr)
        ptr = 0
    }
}