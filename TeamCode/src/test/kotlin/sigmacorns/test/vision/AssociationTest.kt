package sigmacorns.test.vision

import org.joml.Vector2d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import sigmacorns.vision.tracker.Association
import sigmacorns.vision.tracker.FieldDetection
import sigmacorns.vision.tracker.KalmanTrack

@DisplayName("Association")
class AssociationTest {

    private val isoCov = doubleArrayOf(0.01, 0.0, 0.0, 0.01)   // 10 cm stdev isotropic
    private val chi2 = 9.21

    private fun newTrack(id: Int, x: Double, y: Double): KalmanTrack =
        KalmanTrack(
            id = id,
            initialPos = Vector2d(x, y),
            initialPosCov = isoCov,
            t0 = 0.0,
            initialVelVar = 1.0,
            sigmaAMps2 = 3.0,
        )

    @Test
    @DisplayName("empty tracks + empty detections -> empty everything")
    fun emptyEmpty() {
        val r = Association.assign(emptyList(), emptyList(), chi2)
        assertTrue(r.matches.isEmpty())
        assertTrue(r.unmatchedTracks.isEmpty())
        assertTrue(r.unmatchedDetections.isEmpty())
    }

    @Test
    @DisplayName("empty tracks -> all detections unmatched")
    fun emptyTracksAllUnmatched() {
        val d = listOf(FieldDetection(Vector2d(0.0, 0.0), isoCov))
        val r = Association.assign(emptyList(), d, chi2)
        assertTrue(r.matches.isEmpty())
        assertEquals(listOf(0), r.unmatchedDetections)
    }

    @Test
    @DisplayName("empty detections -> all tracks unmatched")
    fun emptyDetectionsAllUnmatched() {
        val t = listOf(newTrack(0, 0.0, 0.0))
        val r = Association.assign(t, emptyList(), chi2)
        assertTrue(r.matches.isEmpty())
        assertEquals(listOf(0), r.unmatchedTracks)
    }

    @Test
    @DisplayName("clean 1:1 match within gate")
    fun clean1to1() {
        val tracks = listOf(newTrack(0, 1.0, 1.0))
        val dets = listOf(FieldDetection(Vector2d(1.02, 0.99), isoCov))
        val r = Association.assign(tracks, dets, chi2)
        assertEquals(listOf(0 to 0), r.matches)
        assertTrue(r.unmatchedTracks.isEmpty())
        assertTrue(r.unmatchedDetections.isEmpty())
    }

    @Test
    @DisplayName("far detection rejected by chi2 gate -> no match, both unmatched")
    fun farDetectionRejected() {
        val tracks = listOf(newTrack(0, 0.0, 0.0))
        val dets = listOf(FieldDetection(Vector2d(5.0, 5.0), isoCov))
        val r = Association.assign(tracks, dets, chi2)
        assertTrue(r.matches.isEmpty())
        assertEquals(listOf(0), r.unmatchedTracks)
        assertEquals(listOf(0), r.unmatchedDetections)
    }

    @Test
    @DisplayName("two nearby tracks crossing paths — each picks its closer detection")
    fun twoTracksCrossing() {
        // Tracks A at (0.0, 0.0), B at (1.0, 0.0). Detections near both.
        val tracks = listOf(
            newTrack(0, 0.0, 0.0),
            newTrack(1, 1.0, 0.0),
        )
        val dets = listOf(
            FieldDetection(Vector2d(0.98, 0.0), isoCov),   // closer to track 1 (B)
            FieldDetection(Vector2d(0.02, 0.0), isoCov),   // closer to track 0 (A)
        )
        val r = Association.assign(tracks, dets, chi2)
        val matchMap = r.matches.toMap()
        assertEquals(1, matchMap[0])
        assertEquals(0, matchMap[1])
        assertTrue(r.unmatchedTracks.isEmpty())
        assertTrue(r.unmatchedDetections.isEmpty())
    }

    @Test
    @DisplayName("greedy picks the closest pair first, leaving looser ones unmatched")
    fun greedyPicksClosestFirst() {
        // Two tracks, one detection. Detection is very close to t0, mid to t1.
        val tracks = listOf(
            newTrack(0, 0.0, 0.0),
            newTrack(1, 0.25, 0.0),
        )
        val dets = listOf(
            FieldDetection(Vector2d(0.02, 0.0), isoCov),
        )
        val r = Association.assign(tracks, dets, chi2)
        assertEquals(listOf(0 to 0), r.matches)
        assertEquals(listOf(1), r.unmatchedTracks)
        assertTrue(r.unmatchedDetections.isEmpty())
    }

    @Test
    @DisplayName("spurious detection far from all tracks stays unmatched")
    fun spuriousDetection() {
        val tracks = listOf(newTrack(0, 0.0, 0.0))
        val dets = listOf(
            FieldDetection(Vector2d(0.01, 0.0), isoCov),
            FieldDetection(Vector2d(10.0, 10.0), isoCov),
        )
        val r = Association.assign(tracks, dets, chi2)
        assertEquals(listOf(0 to 0), r.matches)
        assertTrue(r.unmatchedTracks.isEmpty())
        assertEquals(listOf(1), r.unmatchedDetections)
    }

    @Test
    @DisplayName("each track claims at most one detection (no double-assignment)")
    fun noDoubleAssignment() {
        val tracks = listOf(newTrack(0, 0.0, 0.0))
        val dets = listOf(
            FieldDetection(Vector2d(0.05, 0.0), isoCov),
            FieldDetection(Vector2d(-0.03, 0.0), isoCov),
        )
        val r = Association.assign(tracks, dets, chi2)
        assertEquals(1, r.matches.size)
        // The closer of the two detections wins (dist=0.03 < 0.05).
        assertEquals(1, r.matches[0].second)
        assertEquals(listOf(0), r.unmatchedDetections)
    }

    @Test
    @DisplayName("Mahalanobis respects covariance — elongated covariance gates tighter across its minor axis")
    fun mahalanobisRespectsCovariance() {
        // Covariance elongated along x: measurement uncertainty is high in x, low in y.
        val cov = doubleArrayOf(1.0, 0.0, 0.0, 1e-4)
        val tracks = listOf(newTrack(0, 0.0, 0.0))
        // Displacement of 0.1 m along y (the LOW-variance axis) should be way out of gate
        // because d^2 = 0.01 / (P[1,1] + 1e-4) ~ 0.01 / (0.01 + 1e-4) ~ 0.99 — well inside the gate.
        // Instead pick a much larger y displacement that blows past the tight axis.
        val farY = listOf(FieldDetection(Vector2d(0.0, 1.0), cov))
        val r = Association.assign(tracks, farY, chi2)
        assertTrue(r.matches.isEmpty(), "y displacement dominated by tight axis should reject")

        // Same Euclidean displacement along x, the loose axis, should easily gate in.
        val farX = listOf(FieldDetection(Vector2d(1.0, 0.0), cov))
        val r2 = Association.assign(tracks, farX, chi2)
        assertEquals(1, r2.matches.size, "x displacement along loose axis should match")
    }
}
