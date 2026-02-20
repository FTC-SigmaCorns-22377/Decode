package sigmacorns.io

import dev.frozenmilk.sinister.util.NativeLibraryLoader
import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.State

class RerunLogging private constructor(
    val name: String,
): AutoCloseable {
    init {
        try {
            val resolvedPath = NativeLibraryLoader.resolveLatestHashedLibrary("rerun")
            if (resolvedPath != null) {
                System.load(resolvedPath)
            } else {
                System.loadLibrary("rerun")
            }
        } catch (e: UnsatisfiedLinkError) {
            System.err.println("RerunLogging: failed to load native library 'rerun': ${e.message}")
        }
    }

    private external fun connect(name: String, url: String): Long
    private external fun checkConnection(connection: Long, timeout: Double): Boolean
    private external fun save(name: String, path: String): Long
    private external fun destroy(rec: Long)
    private external fun logState(connection: Long, data: FloatArray)
    private external fun logInputs(connection: Long, data: FloatArray)
    //private external fun logDouble(connection: Long, name: String, value: Double)
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
    private external fun logPoints2D(
        connection: Long,
        name: String,
        points: FloatArray,
        color: Int,
        radius: Double,
    )
    private external fun logPoints3D(
        connection: Long,
        name: String,
        points: FloatArray,
        color: Int,
        radius: Double,
    )
    private external fun logPoints3DWithColors(
        connection: Long,
        name: String,
        points: FloatArray,
        colors: IntArray,
        radius: Double,
    )
    private external fun logText(
        connection: Long,
        name: String,
        text: String,
    )
    private external fun logScalar(
        connection: Long,
        name: String,
        value: Double,
    )

    private var ptr: Long = 0
    private var connectionWarningLogged = false

    var disable: Boolean = false

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
        if(disable) return
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
                rr.isConnected = rr.checkConnection(rr.ptr, 3.0)
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

    fun logInputs(io: SigmaIO) {
        withConnection { handle ->
            logInputs(handle, floatArrayOf(
                io.driveFL.toFloat(), io.driveBL.toFloat(), io.driveBR.toFloat(), io.driveFR.toFloat()
            ))
        }
    }

    fun logScalar(name: String, value: Number) {
        withConnection { handle ->
            logScalar(handle, name, value.toDouble())
        }
    }

    fun logPoints2D(name: String, points: List<Vector2d>, color: Int = 0xFF00FF00.toInt(), radius: Float = 5f) {
        if (points.isEmpty()) return
        val payload = FloatArray(points.size * 2) { i ->
            val pt = points[i / 2]
            if (i % 2 == 0) pt.x.toFloat() else pt.y.toFloat()
        }
        withConnection { handle ->
            logPoints2D(handle, name, payload, color, radius.toDouble())
        }
    }

    fun logPoints3D(name: String, points: List<Vector3d>, color: Int = 0xFFFF8800.toInt(), radius: Float = 0.1f) {
        if (points.isEmpty()) return
        val payload = FloatArray(points.size * 3) { i ->
            val pt = points[i / 3]
            when (i % 3) {
                0 -> pt.x.toFloat()
                1 -> pt.y.toFloat()
                else -> pt.z.toFloat()
            }
        }
        withConnection { handle ->
            logPoints3D(handle, name, payload, color, radius.toDouble())
        }
    }

    fun logPoints3DWithColors(name: String, points: List<Vector3d>, colors: IntArray, radius: Float = 0.1f) {
        if (points.isEmpty()) return
        require(colors.size == points.size) { "Colors array must have same size as points list" }
        val payload = FloatArray(points.size * 3) { i ->
            val pt = points[i / 3]
            when (i % 3) {
                0 -> pt.x.toFloat()
                1 -> pt.y.toFloat()
                else -> pt.z.toFloat()
            }
        }
        withConnection { handle ->
            logPoints3DWithColors(handle, name, payload, colors, radius.toDouble())
        }
    }

    fun logText(name: String, text: String) {
        withConnection { handle ->
            logText(handle, name, text)
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
