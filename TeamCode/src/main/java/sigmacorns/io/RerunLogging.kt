package sigmacorns.io

class RerunLogging {
    init {
        System.loadLibrary("rerun")
    }
    external fun connect(name: String, url: String): Long
    external fun save(name: String, url: String): Long
    external fun destroy(rec: Long)
    external fun logState(connection: Long, data: FloatArray)
}