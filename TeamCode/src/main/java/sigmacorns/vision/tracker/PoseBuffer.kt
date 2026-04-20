package sigmacorns.vision.tracker

import sigmacorns.math.Pose2d
import kotlin.math.PI

/**
 * Ring buffer of (t, Pose2d) samples with linear interpolation in get(t).
 * Heading is unwrapped before interpolation and re-wrapped to [-PI, PI] on
 * return so that wrapping across +/- PI doesn't produce a 2PI jump.
 *
 * Writers append samples with monotonically-non-decreasing t. If an
 * out-of-order sample arrives it is dropped silently. Capacity trims the
 * oldest sample when full.
 *
 * Thread-safety: not thread-safe. The tracker's consumer (Robot.update) is
 * single-threaded per the PollableDispatcher contract.
 */
class PoseBuffer(private val capacity: Int = 500) {

    private data class Sample(val t: Double, val pose: Pose2d)

    private val samples = ArrayDeque<Sample>(capacity)

    val size: Int get() = samples.size
    fun isEmpty(): Boolean = samples.isEmpty()

    /** Append `pose` captured at time `t`. Dropped if older than the last sample. */
    fun add(t: Double, pose: Pose2d) {
        val tail = samples.lastOrNull()
        if (tail != null && t < tail.t) return
        if (samples.size == capacity) samples.removeFirst()
        samples.addLast(Sample(t, copy(pose)))
    }

    /**
     * Pose interpolated at time `t`. Returns null if fewer than 2 samples
     * or t is outside the buffered interval. No extrapolation — consumers
     * that need ever-newer queries must add a fresh sample each tick.
     */
    fun get(t: Double): Pose2d? {
        if (samples.size < 2) return null
        val first = samples.first()
        val last = samples.last()
        if (t < first.t || t > last.t) return null

        // Binary search for the bracketing pair (a, b) with a.t <= t <= b.t.
        var lo = 0
        var hi = samples.size - 1
        while (hi - lo > 1) {
            val mid = (lo + hi) ushr 1
            if (samples[mid].t <= t) lo = mid else hi = mid
        }
        val a = samples[lo]
        val b = samples[hi]

        val span = b.t - a.t
        if (span <= 0.0) return copy(a.pose)
        val alpha = (t - a.t) / span
        val x = a.pose.v.x + alpha * (b.pose.v.x - a.pose.v.x)
        val y = a.pose.v.y + alpha * (b.pose.v.y - a.pose.v.y)
        val theta = interpAngle(a.pose.rot, b.pose.rot, alpha)
        return Pose2d(x, y, theta)
    }

    fun clear() { samples.clear() }

    companion object {
        /** Linearly interpolate between two angles on the shorter arc. */
        internal fun interpAngle(a: Double, b: Double, alpha: Double): Double {
            val aw = wrapPi(a)
            val bw = wrapPi(b)
            var d = bw - aw
            if (d >  PI) d -= 2.0 * PI
            if (d < -PI) d += 2.0 * PI
            return wrapPi(aw + alpha * d)
        }

        internal fun wrapPi(a: Double): Double {
            var x = a
            while (x >  PI) x -= 2.0 * PI
            while (x < -PI) x += 2.0 * PI
            return x
        }
    }
}

private fun copy(p: Pose2d): Pose2d = Pose2d(p.v.x, p.v.y, p.rot)
