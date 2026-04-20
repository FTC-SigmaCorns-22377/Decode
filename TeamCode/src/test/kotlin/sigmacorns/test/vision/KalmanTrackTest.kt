package sigmacorns.test.vision

import org.joml.Vector2d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import sigmacorns.vision.tracker.KalmanTrack

@DisplayName("KalmanTrack")
class KalmanTrackTest {

    private val rMeas = doubleArrayOf(0.01, 0.0, 0.0, 0.01)  // 10 cm stdev isotropic

    private fun newTrack(t0: Double = 0.0, sigmaA: Double = 3.0): KalmanTrack =
        KalmanTrack(
            id = 0,
            initialPos = Vector2d(0.0, 0.0),
            initialPosCov = rMeas,
            t0 = t0,
            initialVelVar = 1.0,
            sigmaAMps2 = sigmaA,
        )

    @Test
    @DisplayName("Initial state is the seed position with zero velocity")
    fun initialStateIsSeed() {
        val t = newTrack()
        assertEquals(0.0, t.state[0], 0.0)
        assertEquals(0.0, t.state[1], 0.0)
        assertEquals(0.0, t.state[2], 0.0)
        assertEquals(0.0, t.state[3], 0.0)
        assertEquals(1, t.hits)
        assertEquals(0, t.framesSinceSeen)
    }

    @Test
    @DisplayName("predict-only covariance trace grows monotonically")
    fun predictOnlyCovGrowsMonotonically() {
        val t = newTrack()
        var prev = t.positionCovTrace()
        for (i in 1..200) {
            t.predict(i * 0.02)
            val now = t.positionCovTrace()
            assertTrue(now >= prev - 1e-12, "trace decreased at step $i: $prev -> $now")
            prev = now
        }
    }

    @Test
    @DisplayName("KF converges on a constant-velocity noisy stream")
    fun cvStreamConverges() {
        val rng = java.util.Random(1234L)
        val vxTrue = 1.2
        val vyTrue = -0.4
        val dt = 0.02
        val sigmaPx = 0.005  // ~5 mm pixel-equivalent noise after projection
        val r = doubleArrayOf(sigmaPx * sigmaPx, 0.0, 0.0, sigmaPx * sigmaPx)

        val t = KalmanTrack(
            id = 1,
            initialPos = Vector2d(0.0, 0.0),
            initialPosCov = r,
            t0 = 0.0,
            initialVelVar = 1.0,
            sigmaAMps2 = 1.0,
        )

        var time = 0.0
        repeat(400) {
            time += dt
            val truePx = vxTrue * time
            val truePy = vyTrue * time
            val noisy = Vector2d(
                truePx + rng.nextGaussian() * sigmaPx,
                truePy + rng.nextGaussian() * sigmaPx,
            )
            t.update(noisy, r, time)
        }

        assertEquals(vxTrue, t.state[2], 0.05, "vx should converge")
        assertEquals(vyTrue, t.state[3], 0.05, "vy should converge")
        assertEquals(vxTrue * time, t.state[0], 0.02, "px should track")
        assertEquals(vyTrue * time, t.state[1], 0.02, "py should track")
    }

    @Test
    @DisplayName("update shrinks position covariance trace")
    fun updateShrinksCov() {
        val t = newTrack()
        t.predict(1.0)
        val traceBefore = t.positionCovTrace()
        t.update(Vector2d(0.5, 0.5), rMeas, 1.0)
        val traceAfter = t.positionCovTrace()
        assertTrue(traceAfter < traceBefore, "update should decrease position-cov trace")
    }

    @Test
    @DisplayName("update advances lastUpdateT and increments hits")
    fun updateBookkeeping() {
        val t = newTrack(t0 = 0.0)
        assertEquals(0.0, t.lastUpdateT, 0.0)
        t.update(Vector2d(0.1, 0.0), rMeas, 0.5)
        assertEquals(0.5, t.lastUpdateT, 0.0)
        assertEquals(2, t.hits)
        assertEquals(0, t.framesSinceSeen)
    }

    @Test
    @DisplayName("markMissed increments framesSinceSeen but leaves lastUpdateT")
    fun markMissedBookkeeping() {
        val t = newTrack(t0 = 0.0)
        t.markMissed(); t.markMissed()
        assertEquals(2, t.framesSinceSeen)
        assertEquals(0.0, t.lastUpdateT, 0.0)
    }

    @Test
    @DisplayName("predict does not advance lastUpdateT")
    fun predictLeavesLastUpdateT() {
        val t = newTrack(t0 = 0.0)
        t.predict(1.0)
        t.predict(2.0)
        assertEquals(0.0, t.lastUpdateT, 0.0)
    }

    @Test
    @DisplayName("positionAt extrapolates without mutating state")
    fun positionAtIsPure() {
        val t = newTrack(t0 = 0.0)
        t.update(Vector2d(0.5, 0.0), rMeas, 1.0)
        t.update(Vector2d(1.0, 0.0), rMeas, 2.0)
        // Velocity is approximately (0.5, 0).
        val pAt3 = t.positionAt(3.0)
        val stateBefore = t.state.copyOf()
        assertTrue(pAt3.x > t.state[0], "extrapolated x should be ahead of last update")
        assertTrue(t.state.contentEquals(stateBefore), "state must be unchanged by positionAt")
    }

    @Test
    @DisplayName("P stays symmetric after predict and update cycles")
    fun pStaysSymmetric() {
        val t = newTrack()
        repeat(50) { i ->
            val time = (i + 1) * 0.1
            t.update(Vector2d(0.1 * time, 0.0), rMeas, time)
            t.predict(time + 0.05)
            for (r in 0 until 4) for (c in 0 until 4) {
                val a = t.P[r * 4 + c]
                val b = t.P[c * 4 + r]
                assertEquals(a, b, 1e-10, "P asymmetric at [$r,$c] step=$i")
            }
        }
    }
}
