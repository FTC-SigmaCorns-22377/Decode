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
    private external fun checkConnection(connection: Long, timeout: Double): Boolean
    private external fun save(name: String, path: String): Long
    private external fun destroy(rec: Long)
    private external fun logState(connection: Long, data: FloatArray)
    private external fun logInput(connection: Long, data: FloatArray)
    private external fun logLineStrip3D(connection: Long, name: String, data: FloatArray)
    private external fun logImage(
        connection: Long,
        name: String,
        width: Int,
        height: Int,
        formatOrdinal: Int,
        pixels: ByteArray,
    )
    private external fun logImageMesh(
        connection: Long,
        name: String,
        width: Int,
        height: Int,
        formatOrdinal: Int,
        pixels: ByteArray,
    )
    private external fun logTransform(
        connection: Long,
        name: String,
        translation: FloatArray,
        quaternion: FloatArray,
        scale: FloatArray,
    )

    private var ptr: Long = 0
    private var connectionWarningLogged = false

    var isConnected: Boolean = false
        private set

    private fun logConnectionUnavailable(message: String? = null) {
        if (connectionWarningLogged) {
            return
        }

        val details = message ?: "RerunLogging[$name]: rerun logging disabled (no active connection)."
        System.err.println(details)
        connectionWarningLogged = true
    }

    private inline fun withConnection(block: (Long) -> Unit) {
        val handle = ptr
        if (!isConnected) {
            logConnectionUnavailable()
            return
        }

        block(handle)
    }

    companion object {
        fun connect(name: String, url: String): RerunLogging {
            val rr = RerunLogging(name)
            rr.ptr = rr.connect(name, url)
            if (rr.ptr == 0L) {
                rr.logConnectionUnavailable(
                    "RerunLogging[${rr.name}]: failed to connect to $url; rerun logging disabled.",
                )
                rr.isConnected = false
            } else {
                rr.isConnected = true
                //rr.isConnected = rr.checkConnection(rr.ptr, 3.0)
            }
            return rr
        }

        fun save(name: String, path: String): RerunLogging {
            val rr = RerunLogging(name)
            rr.ptr = rr.save(name, path)

            rr.isConnected = rr.ptr != 0L
            return rr
        }
    }

    fun logState(state: State) {
        withConnection { handle ->
            logState(handle, state.toFloatArray())
        }
    }

    fun logLineStrip(name: String, points: List<Vector3d>) {
        val payload = FloatArray(points.size * 3) {
            val i = it / 3
            val d = if (it % 3 == 0) {
                points[i].x
            } else if (it % 3 == 1) {
                points[i].y
            } else {
                points[i].z
            }
            d.toFloat()
        }

        withConnection { handle ->
            logLineStrip3D(handle, name, payload)
        }
    }

    fun logLineStrip(name: String, points: List<Vector2d>, z: Number = 0f) {
        val payload = FloatArray(points.size * 3) {
            val i = it / 3
            val d = if (it % 3 == 0) {
                points[i].x
            } else if (it % 3 == 1) {
                points[i].y
            } else {
                z.toFloat()
            }
            d.toFloat()
        }

        withConnection { handle ->
            logLineStrip3D(handle, name, payload)
        }
    }

    fun logImage(
        name: String,
        width: Int,
        height: Int,
        format: RerunImageFormat,
        pixels: ByteArray,
    ) {
        val expectedSize = width.toLong() * height.toLong() * format.channels
        require(pixels.size.toLong() == expectedSize) {
            "Expected ${expectedSize}B for ${format.name} image ($width x $height), but received ${pixels.size}B"
        }

        withConnection { handle ->
            logImage(handle, name, width, height, format.ordinal, pixels)
        }
    }

    fun logImageMesh(
        name: String,
        width: Int,
        height: Int,
        format: RerunImageFormat,
        pixels: ByteArray,
    ) {
        require(width > 0 && height > 0) { "Image dimensions must be positive" }

        val expectedSize = width.toLong() * height.toLong() * format.channels
        require(pixels.size.toLong() == expectedSize) {
            "Expected ${expectedSize}B for ${format.name} image ($width x $height), but received ${pixels.size}B"
        }

        withConnection { handle ->
            logImageMesh(handle, name, width, height, format.ordinal, pixels)
        }
    }

    fun logTransform(
        name: String,
        translation: FloatArray,
        quaternion: FloatArray,
        scale: FloatArray = floatArrayOf(1f, 1f, 1f),
    ) {
        require(translation.size == 3) { "Translation must contain exactly 3 floats" }
        require(quaternion.size == 4) { "Quaternion must contain exactly 4 floats" }
        require(scale.size == 3) { "Scale must contain exactly 3 floats" }

        withConnection { handle ->
            logTransform(handle, name, translation, quaternion, scale)
        }
    }

    override fun close() {
        val handle = ptr
        if (handle == 0L) {
            return
        }

        destroy(handle)
        ptr = 0
    }
}

enum class RerunImageFormat(internal val channels: Int) {
    L8(1),
    RGB(3),
    RGBA(4),
}
