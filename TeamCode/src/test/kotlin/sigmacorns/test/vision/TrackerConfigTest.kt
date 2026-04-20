package sigmacorns.test.vision

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import sigmacorns.vision.tracker.TrackerConfig

@DisplayName("TrackerConfig")
class TrackerConfigTest {

    @Test
    @DisplayName("loadDefault parses config/ball_tracker.json with pinned values")
    fun loadDefaultFromProjectDir() {
        val cfg = TrackerConfig.loadDefault()

        assertEquals(3.6576, cfg.fieldWidthM, 0.0)
        assertEquals(3.6576, cfg.fieldHeightM, 0.0)
        assertEquals(0.10, cfg.fieldMarginM, 0.0)
        assertEquals(0.20, cfg.rampExpandM, 0.0)
        assertEquals(0.035, cfg.ballRadiusM, 0.0)

        assertEquals(900.0, cfg.intrinsics.fx, 0.0)
        assertEquals(900.0, cfg.intrinsics.fy, 0.0)
        assertEquals(640.0, cfg.intrinsics.cx, 0.0)
        assertEquals(360.0, cfg.intrinsics.cy, 0.0)
        assertEquals(1280, cfg.imageWidthPx)
        assertEquals(720, cfg.imageHeightPx)

        assertEquals(0.162, cfg.cameraExtrinsics.camPosR.x, 0.0)
        assertEquals(0.099, cfg.cameraExtrinsics.camPosR.y, 0.0)
        assertEquals(0.242, cfg.cameraExtrinsics.camPosR.z, 0.0)
        assertEquals(-0.199, cfg.cameraExtrinsics.pitchDownRad, 0.0)
        assertEquals(0.0, cfg.cameraExtrinsics.yawRad, 0.0)
        assertEquals(0.0, cfg.cameraExtrinsics.rollRad, 0.0)

        assertEquals(4, cfg.robotFootprintR.size)

        assertEquals(0.66, cfg.intakeMaskYMinFrac, 0.0)
        assertEquals(1.5, cfg.sigmaPx, 0.0)
        assertEquals(8.0, cfg.maxRangeM, 0.0)

        assertEquals(3.0, cfg.sigmaAMps2, 0.0)
        assertEquals(1.0, cfg.initVelVar, 0.0)
        assertEquals(9.21, cfg.chi2Gate, 0.0)
        assertEquals(15, cfg.maxCoastFrames)
        assertEquals(3, cfg.minHitsForConfirmed)
    }

    @Test
    @DisplayName("Cached TRC matches a freshly-built one")
    fun cachedTrcMatchesFresh() {
        val cfg = TrackerConfig.loadDefault()
        val fresh = cfg.cameraExtrinsics.toTRC()
        for (i in 0..3) for (j in 0..3) {
            assertEquals(fresh.get(i, j), cfg.TRC.get(i, j), 0.0, "T_RC[$i][$j]")
        }
    }

    @Test
    @DisplayName("loadFromString round-trips a complete config")
    fun loadFromStringRoundTrips() {
        val json = """
            {
              "FIELD_WIDTH_M": 3.6576,
              "FIELD_HEIGHT_M": 3.6576,
              "FIELD_MARGIN_M": 0.10,
              "RAMP_POLYGON": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
              "RAMP_EXPAND_M": 0.20,
              "BALL_RADIUS_M": 0.035,
              "K": [1000.0, 1000.0, 320.0, 240.0],
              "DIST_COEFFS": [-0.1, 0.05, 0.001, 0.002, 0.0],
              "IMAGE_WIDTH_PX": 640,
              "IMAGE_HEIGHT_PX": 480,
              "CAM_POS_R": [0.1, 0.0, 0.3],
              "CAM_PITCH_DOWN_RAD": 0.25,
              "CAM_YAW_RAD": 0.0,
              "CAM_ROLL_RAD": 0.0,
              "ROBOT_FOOTPRINT_R": [[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]],
              "INTAKE_MASK_Y_MIN_FRAC": 0.5,
              "SIGMA_PX": 2.0,
              "MAX_RANGE_M": 5.0,
              "SIGMA_A_MPS2": 2.5,
              "INIT_VEL_VAR": 0.5,
              "CHI2_GATE": 5.99,
              "MAX_COAST_FRAMES": 10,
              "MIN_HITS_FOR_CONFIRMED": 2
            }
        """.trimIndent()

        val cfg = TrackerConfig.loadFromString(json)
        assertEquals(3, cfg.rampPolygon.size)
        assertEquals(1.0, cfg.rampPolygon[1].x, 0.0)
        assertEquals(1000.0, cfg.intrinsics.fx, 0.0)
        assertEquals(-0.1, cfg.intrinsics.k1, 0.0)
        assertEquals(0.05, cfg.intrinsics.k2, 0.0)
        assertEquals(0.001, cfg.intrinsics.p1, 0.0)
        assertEquals(0.002, cfg.intrinsics.p2, 0.0)
        assertEquals(0.25, cfg.cameraExtrinsics.pitchDownRad, 0.0)
        assertEquals(5.99, cfg.chi2Gate, 0.0)
        assertEquals(10, cfg.maxCoastFrames)
        assertEquals(2, cfg.minHitsForConfirmed)
    }

    @Test
    @DisplayName("Malformed K vector is rejected")
    fun rejectsShortK() {
        val bad = """
            {
              "FIELD_WIDTH_M": 3.6576, "FIELD_HEIGHT_M": 3.6576, "FIELD_MARGIN_M": 0.1,
              "RAMP_POLYGON": [], "RAMP_EXPAND_M": 0.2,
              "BALL_RADIUS_M": 0.035,
              "K": [900.0, 900.0],
              "DIST_COEFFS": [0.0, 0.0, 0.0, 0.0, 0.0],
              "IMAGE_WIDTH_PX": 1280, "IMAGE_HEIGHT_PX": 720,
              "CAM_POS_R": [0.0, 0.0, 0.0], "CAM_PITCH_DOWN_RAD": 0.0, "CAM_YAW_RAD": 0.0, "CAM_ROLL_RAD": 0.0,
              "ROBOT_FOOTPRINT_R": [],
              "INTAKE_MASK_Y_MIN_FRAC": 0.66,
              "SIGMA_PX": 1.5, "MAX_RANGE_M": 8.0,
              "SIGMA_A_MPS2": 3.0, "INIT_VEL_VAR": 1.0, "CHI2_GATE": 9.21,
              "MAX_COAST_FRAMES": 15, "MIN_HITS_FOR_CONFIRMED": 3
            }
        """.trimIndent()
        val ex = assertThrows(IllegalArgumentException::class.java) {
            TrackerConfig.loadFromString(bad)
        }
        assertTrue(ex.message!!.contains("K must have 4 entries"))
    }
}
