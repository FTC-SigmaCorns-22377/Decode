package sigmacorns.test

import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Test
import sigmacorns.constants.Network
import java.nio.file.Files
import kotlin.io.path.deleteIfExists
import sigmacorns.io.RerunImageFormat
import sigmacorns.io.RerunLogging

class RerunLoggingImageTest {
    @Test
    fun `log image succeeds with correct payload`() {
        RerunLogging.connect("image-test", Network.SIM_RERUN).use { rr ->
            val width = 2
            val height = 2
            val pixels = byteArrayOf(
                0, 10, 20,
                30, 40, 50,
                60, 70, 80,
                90, 100, 110,
            )

            rr.logImage(
                name = "camera/frame",
                width = width,
                height = height,
                format = RerunImageFormat.RGB,
                pixels = pixels,
            )
        }
    }

    @Test
    fun `log image mesh renders textured quad`() {
        RerunLogging.connect("image-mesh-test", Network.SIM_RERUN).use { rr ->
            val width = 2
            val height = 2
            val pixels = byteArrayOf(
                255.toByte(), 0, 0,
                0, 255.toByte(), 0,
                0, 0, 255.toByte(),
                255.toByte(), 255.toByte(), 0,
            )

            rr.logImageMesh(
                name = "world/board",
                width = width,
                height = height,
                format = RerunImageFormat.RGB,
                pixels = pixels,
            )
        }
    }

    @Test
    fun `log image throws on size mismatch`() {
        RerunLogging.connect("image-test", Network.SIM_RERUN).use { rr ->
            val width = 2
            val height = 2
            val mismatchedPixels = byteArrayOf(0, 1, 2)

            assertThrows(IllegalArgumentException::class.java) {
                rr.logImage(
                    name = "camera/frame",
                    width = width,
                    height = height,
                    format = RerunImageFormat.RGB,
                    pixels = mismatchedPixels,
                )
            }
        }
    }

    @Test
    fun `log transform requires expected vector lengths`() {
        RerunLogging.connect("transform-test", Network.SIM_RERUN).use { rr ->
            val translation = floatArrayOf(0f, 0f, 0.5f)
            val quaternion = floatArrayOf(0f, 0f, 0f, 1f)
            val scale = floatArrayOf(1f, 1f, 1f)

            rr.logTransform("world/pose", translation, quaternion, scale)

            assertThrows(IllegalArgumentException::class.java) {
                rr.logTransform("world/pose", floatArrayOf(0f), quaternion, scale)
            }

            assertThrows(IllegalArgumentException::class.java) {
                rr.logTransform("world/pose", translation, floatArrayOf(0f, 0f), scale)
            }

            assertThrows(IllegalArgumentException::class.java) {
                rr.logTransform("world/pose", translation, quaternion, floatArrayOf(1f))
            }
        }
    }
}
