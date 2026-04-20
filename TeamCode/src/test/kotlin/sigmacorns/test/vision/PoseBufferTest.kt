package sigmacorns.test.vision

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.PoseBuffer
import kotlin.math.PI

@DisplayName("PoseBuffer")
class PoseBufferTest {

    private val tol = 1e-9

    @Test
    @DisplayName("empty buffer returns null for any t")
    fun emptyReturnsNull() {
        val b = PoseBuffer()
        assertNull(b.get(0.0))
    }

    @Test
    @DisplayName("single sample returns null (needs two to interpolate)")
    fun singleSampleReturnsNull() {
        val b = PoseBuffer()
        b.add(0.0, Pose2d(0.0, 0.0, 0.0))
        assertNull(b.get(0.0))
    }

    @Test
    @DisplayName("exact-sample query returns that sample's pose")
    fun exactSampleQuery() {
        val b = PoseBuffer()
        b.add(0.0, Pose2d(1.0, 2.0, 0.1))
        b.add(1.0, Pose2d(3.0, 4.0, 0.2))
        val p = b.get(1.0)!!
        assertEquals(3.0, p.v.x, tol)
        assertEquals(4.0, p.v.y, tol)
        assertEquals(0.2, p.rot, tol)
    }

    @Test
    @DisplayName("linear interpolation hits the midpoint exactly")
    fun linearMidpoint() {
        val b = PoseBuffer()
        b.add(0.0, Pose2d(0.0, 0.0, 0.0))
        b.add(1.0, Pose2d(2.0, -4.0, 0.4))
        val p = b.get(0.5)!!
        assertEquals(1.0, p.v.x, tol)
        assertEquals(-2.0, p.v.y, tol)
        assertEquals(0.2, p.rot, tol)
    }

    @Test
    @DisplayName("out-of-range t returns null")
    fun outOfRangeReturnsNull() {
        val b = PoseBuffer()
        b.add(0.0, Pose2d(0.0, 0.0, 0.0))
        b.add(1.0, Pose2d(1.0, 0.0, 0.0))
        assertNull(b.get(-0.01))
        assertNull(b.get(1.01))
    }

    @Test
    @DisplayName("heading interpolation takes the short arc across +/-PI")
    fun headingShortArcAcrossPi() {
        val b = PoseBuffer()
        b.add(0.0, Pose2d(0.0, 0.0,  PI - 0.1))
        b.add(1.0, Pose2d(0.0, 0.0, -PI + 0.1))
        val mid = b.get(0.5)!!
        // Short arc crosses PI -> interpolated heading should be near PI (or -PI).
        val wrapped = PoseBuffer.wrapPi(mid.rot)
        val distToPi    = kotlin.math.abs(PoseBuffer.wrapPi(wrapped - PI))
        val distToNegPi = kotlin.math.abs(PoseBuffer.wrapPi(wrapped + PI))
        assertTrue(
            kotlin.math.min(distToPi, distToNegPi) < 1e-9,
            "interpolated heading $wrapped is not near +/-PI (should be the short arc)",
        )
    }

    @Test
    @DisplayName("heading interpolation is the short arc around 0")
    fun headingShortArcAcrossZero() {
        val b = PoseBuffer()
        b.add(0.0, Pose2d(0.0, 0.0, -0.4))
        b.add(1.0, Pose2d(0.0, 0.0,  0.4))
        val mid = b.get(0.5)!!
        assertEquals(0.0, mid.rot, tol)
    }

    @Test
    @DisplayName("out-of-order samples are dropped")
    fun outOfOrderDropped() {
        val b = PoseBuffer()
        b.add(1.0, Pose2d(1.0, 0.0, 0.0))
        b.add(0.5, Pose2d(99.0, 99.0, 99.0))   // dropped — older than tail
        assertEquals(1, b.size)
    }

    @Test
    @DisplayName("capacity trims the oldest sample")
    fun capacityTrims() {
        val b = PoseBuffer(capacity = 3)
        b.add(0.0, Pose2d(0.0, 0.0, 0.0))
        b.add(1.0, Pose2d(1.0, 0.0, 0.0))
        b.add(2.0, Pose2d(2.0, 0.0, 0.0))
        b.add(3.0, Pose2d(3.0, 0.0, 0.0))
        assertEquals(3, b.size)
        assertNull(b.get(0.5))
        val p = b.get(2.5)!!
        assertEquals(2.5, p.v.x, tol)
    }

    @Test
    @DisplayName("dense sweep — every midpoint is exact to 1e-9")
    fun denseSweep() {
        val b = PoseBuffer(capacity = 100)
        for (i in 0..50) {
            val t = i * 0.02
            b.add(t, Pose2d(t, -0.5 * t, 0.01 * t))
        }
        for (i in 0 until 50) {
            val t = i * 0.02 + 0.01
            val p = b.get(t)!!
            assertEquals(t,        p.v.x, tol)
            assertEquals(-0.5 * t, p.v.y, tol)
            assertEquals(0.01 * t, p.rot, tol)
        }
    }

    @Test
    @DisplayName("wrapPi maps everything to [-PI, PI]")
    fun wrapPiBounds() {
        val samples = listOf(-7.0, -4.0, -PI, 0.0, PI, 4.0, 7.0)
        for (a in samples) {
            val w = PoseBuffer.wrapPi(a)
            assertTrue(w >= -PI - 1e-12 && w <= PI + 1e-12, "wrapPi($a) = $w out of range")
        }
    }
}
