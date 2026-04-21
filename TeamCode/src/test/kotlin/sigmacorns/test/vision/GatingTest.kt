package sigmacorns.test.vision

import org.joml.Vector2d
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.Gating
import kotlin.math.PI

@DisplayName("Gating")
class GatingTest {

    // ---- insideIntakeMask ----

    @Test
    @DisplayName("intake mask catches bottom-third pixel rows")
    fun intakeMaskCatchesBottom() {
        assertTrue(Gating.insideIntakeMask(v = 600.0, imageHeight = 720, yMinFrac = 0.66))
    }

    @Test
    @DisplayName("intake mask leaves top of frame alone")
    fun intakeMaskLeavesTop() {
        assertFalse(Gating.insideIntakeMask(v = 100.0, imageHeight = 720, yMinFrac = 0.66))
    }

    @Test
    @DisplayName("intake mask boundary is inclusive at v = h*frac")
    fun intakeMaskBoundaryInclusive() {
        val h = 720
        val f = 0.5
        assertTrue(Gating.insideIntakeMask(v = h * f, imageHeight = h, yMinFrac = f))
    }

    // ---- insideField ----

    @Test
    @DisplayName("field bounds accept the origin (center of the field)")
    fun fieldCenterAccepted() {
        assertTrue(Gating.insideField(Vector2d(0.0, 0.0), 3.6576, 3.6576, 0.1))
    }

    @Test
    @DisplayName("field bounds reject points outside the half-extent")
    fun fieldOutsideRejected() {
        // Half-extent = 1.8288 m; 0.1 m margin → valid |p.x|, |p.y| <= 1.73
        assertFalse(Gating.insideField(Vector2d(2.0, 0.0), 3.6576, 3.6576, 0.1))
        assertFalse(Gating.insideField(Vector2d(-2.0, 0.0), 3.6576, 3.6576, 0.1))
        assertFalse(Gating.insideField(Vector2d(0.0, 2.0), 3.6576, 3.6576, 0.1))
    }

    @Test
    @DisplayName("field inward margin rejects points inside the margin band")
    fun fieldMarginBandRejected() {
        // Valid range is [-1.73, 1.73]; 1.80 is inside the 10 cm inward margin.
        assertFalse(Gating.insideField(Vector2d(1.80, 0.0), 3.6576, 3.6576, 0.1))
        assertFalse(Gating.insideField(Vector2d(-1.80, 0.0), 3.6576, 3.6576, 0.1))
    }

    @Test
    @DisplayName("field bounds accept negative coordinates inside the half-extent")
    fun fieldNegativeCoordsAccepted() {
        // Origin-centered: x=-1.0 is well inside [-1.73, 1.73].
        assertTrue(Gating.insideField(Vector2d(-1.0, -0.5), 3.6576, 3.6576, 0.1))
    }

    // ---- insideRamp ----

    @Test
    @DisplayName("empty ramp polygon never excludes anything")
    fun emptyRampIsNoop() {
        assertFalse(Gating.insideRamp(Vector2d(1.0, 1.0), emptyList(), 0.2))
    }

    @Test
    @DisplayName("ramp polygon excludes interior points")
    fun rampInteriorExcluded() {
        val ramp = listOf(
            Vector2d(0.0, 0.0),
            Vector2d(1.0, 0.0),
            Vector2d(1.0, 1.0),
            Vector2d(0.0, 1.0),
        )
        assertTrue(Gating.insideRamp(Vector2d(0.5, 0.5), ramp, 0.0))
    }

    @Test
    @DisplayName("ramp expansion catches points just outside the boundary")
    fun rampExpansionCatchesOutside() {
        val ramp = listOf(
            Vector2d(0.0, 0.0),
            Vector2d(1.0, 0.0),
            Vector2d(1.0, 1.0),
            Vector2d(0.0, 1.0),
        )
        // 10 cm outside an edge; with 20 cm expansion, should be inside.
        assertTrue(Gating.insideRamp(Vector2d(1.10, 0.5), ramp, 0.20))
        // 30 cm outside; 20 cm expansion rejects.
        assertFalse(Gating.insideRamp(Vector2d(1.30, 0.5), ramp, 0.20))
    }

    // ---- insideRobotFootprint ----

    @Test
    @DisplayName("robot footprint excludes own-chassis points in field frame")
    fun footprintExcludesChassis() {
        val footR = listOf(
            Vector2d( 0.2,  0.2),
            Vector2d( 0.2, -0.2),
            Vector2d(-0.2, -0.2),
            Vector2d(-0.2,  0.2),
        )
        val pose = Pose2d(1.0, 1.0, 0.0)
        assertTrue(Gating.insideRobotFootprint(Vector2d(1.0, 1.0), pose, footR))
        assertFalse(Gating.insideRobotFootprint(Vector2d(2.0, 2.0), pose, footR))
    }

    @Test
    @DisplayName("footprint rotates with heading")
    fun footprintRotates() {
        // Non-square footprint — rectangle 0.6 m long (x_robot) x 0.2 m wide (y_robot).
        val footR = listOf(
            Vector2d( 0.3,  0.1),
            Vector2d( 0.3, -0.1),
            Vector2d(-0.3, -0.1),
            Vector2d(-0.3,  0.1),
        )
        // Heading 0: footprint field-frame x-extent = [-0.3, 0.3], y-extent = [-0.1, 0.1].
        // (0.2, 0.0) is inside; (0.0, 0.2) is outside (y=0.2 > 0.1).
        assertTrue(Gating.insideRobotFootprint(Vector2d(0.2, 0.0), Pose2d(0.0, 0.0, 0.0), footR))
        assertFalse(Gating.insideRobotFootprint(Vector2d(0.0, 0.2), Pose2d(0.0, 0.0, 0.0), footR))

        // Heading PI/2: long axis rotates to point along +Y_field.
        // Field-frame x-extent = [-0.1, 0.1], y-extent = [-0.3, 0.3].
        // Now (0.2, 0.0) is OUTSIDE (x=0.2 > 0.1), (0.0, 0.2) is INSIDE.
        val pose90 = Pose2d(0.0, 0.0, PI / 2.0)
        assertFalse(Gating.insideRobotFootprint(Vector2d(0.2, 0.0), pose90, footR))
        assertTrue(Gating.insideRobotFootprint(Vector2d(0.0, 0.2), pose90, footR))
    }

    // ---- filterDetection (pipeline) ----

    @Test
    @DisplayName("pipeline drops image-mask detections before projection")
    fun pipelineIntakeMaskDrop() {
        val result = Gating.filterDetection(
            pixel = Vector2d(640.0, 700.0),
            imageHeight = 720,
            intakeMaskYMinFrac = 0.66,
            fieldPoint = Vector2d(1.8, 1.8),
            fieldWidthM = 3.6576, fieldHeightM = 3.6576, fieldMarginM = 0.1,
            rampPolygon = emptyList(), rampExpandM = 0.2,
            robotPose = Pose2d(0.0, 0.0, 0.0),
            footprintR = emptyList(),
        )
        assertNull(result)
    }

    @Test
    @DisplayName("pipeline drops null field point (projection failure)")
    fun pipelineNullFieldDrop() {
        val result = Gating.filterDetection(
            pixel = Vector2d(640.0, 200.0),
            imageHeight = 720,
            intakeMaskYMinFrac = 0.66,
            fieldPoint = null,
            fieldWidthM = 3.6576, fieldHeightM = 3.6576, fieldMarginM = 0.1,
            rampPolygon = emptyList(), rampExpandM = 0.2,
            robotPose = Pose2d(0.0, 0.0, 0.0),
            footprintR = emptyList(),
        )
        assertNull(result)
    }

    @Test
    @DisplayName("pipeline passes a clean detection through")
    fun pipelineCleanPass() {
        val result = Gating.filterDetection(
            pixel = Vector2d(640.0, 200.0),
            imageHeight = 720,
            intakeMaskYMinFrac = 0.66,
            fieldPoint = Vector2d(0.5, 0.5),         // comfortably inside half-extent
            fieldWidthM = 3.6576, fieldHeightM = 3.6576, fieldMarginM = 0.1,
            rampPolygon = emptyList(), rampExpandM = 0.2,
            robotPose = Pose2d(0.0, 0.0, 0.0),
            footprintR = emptyList(),
        )
        assertNotNull(result)
    }

    @Test
    @DisplayName("pipeline rejects field-bound violations")
    fun pipelineFieldBoundViolation() {
        val result = Gating.filterDetection(
            pixel = Vector2d(640.0, 200.0),
            imageHeight = 720,
            intakeMaskYMinFrac = 0.66,
            fieldPoint = Vector2d(-0.5, 1.8),
            fieldWidthM = 3.6576, fieldHeightM = 3.6576, fieldMarginM = 0.1,
            rampPolygon = emptyList(), rampExpandM = 0.2,
            robotPose = Pose2d(0.0, 0.0, 0.0),
            footprintR = emptyList(),
        )
        assertNull(result)
    }

    @Test
    @DisplayName("pipeline rejects detections inside expanded ramp")
    fun pipelineRampReject() {
        val ramp = listOf(
            Vector2d(1.0, 1.0),
            Vector2d(2.0, 1.0),
            Vector2d(2.0, 2.0),
            Vector2d(1.0, 2.0),
        )
        val result = Gating.filterDetection(
            pixel = Vector2d(640.0, 200.0),
            imageHeight = 720,
            intakeMaskYMinFrac = 0.66,
            fieldPoint = Vector2d(1.5, 1.5),
            fieldWidthM = 3.6576, fieldHeightM = 3.6576, fieldMarginM = 0.1,
            rampPolygon = ramp, rampExpandM = 0.2,
            robotPose = Pose2d(0.0, 0.0, 0.0),
            footprintR = emptyList(),
        )
        assertNull(result)
    }

    @Test
    @DisplayName("pipeline rejects detections on own chassis")
    fun pipelineFootprintReject() {
        val footR = listOf(
            Vector2d( 0.2,  0.2),
            Vector2d( 0.2, -0.2),
            Vector2d(-0.2, -0.2),
            Vector2d(-0.2,  0.2),
        )
        val result = Gating.filterDetection(
            pixel = Vector2d(640.0, 200.0),
            imageHeight = 720,
            intakeMaskYMinFrac = 0.66,
            fieldPoint = Vector2d(1.0, 1.0),
            fieldWidthM = 3.6576, fieldHeightM = 3.6576, fieldMarginM = 0.1,
            rampPolygon = emptyList(), rampExpandM = 0.0,
            robotPose = Pose2d(1.0, 1.0, 0.0),
            footprintR = footR,
        )
        assertNull(result)
    }
}
