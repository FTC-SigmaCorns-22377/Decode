package sigmacorns.vision.tracker

import com.google.gson.Gson
import com.google.gson.JsonArray
import com.google.gson.JsonObject
import org.joml.Matrix4d
import org.joml.Vector2d
import org.joml.Vector3d
import java.io.File
import java.io.FileReader
import java.io.Reader
import java.io.StringReader

/**
 * All ball-tracker tunables. Loaded from `config/ball_tracker.json`.
 *
 * Field keys in the JSON are SCREAMING_SNAKE_CASE to stay readable as a tuning file.
 * This Kotlin object names them in camelCase. Mapping is performed in
 * [fromJson] so renames in either direction do not break silently.
 *
 * Single source of truth for:
 *   - Camera intrinsics + image size.
 *   - Camera extrinsics in robot frame (CAM_POS_R + Euler angles).
 *   - Robot chassis footprint (for self-occlusion gating).
 *   - Field rectangle + margin, ramp polygon + expansion.
 *   - Image-space intake mask fraction.
 *   - Kalman process / measurement noise, gating chi^2, track birth/death knobs.
 */
data class TrackerConfig(
    val fieldWidthM: Double,
    val fieldHeightM: Double,
    val fieldMarginM: Double,

    val rampPolygon: List<Vector2d>,
    val rampExpandM: Double,

    val ballRadiusM: Double,

    val intrinsics: Intrinsics,
    val imageWidthPx: Int,
    val imageHeightPx: Int,

    val cameraExtrinsics: CameraExtrinsicsR,

    val robotFootprintR: List<Vector2d>,

    val intakeMaskYMinFrac: Double,

    val sigmaPx: Double,
    val maxRangeM: Double,

    val sigmaAMps2: Double,
    val initVelVar: Double,
    val chi2Gate: Double,
    val maxCoastFrames: Int,
    val minHitsForConfirmed: Int,
) {
    /** Cached T_RC built from the camera extrinsics. */
    val TRC: Matrix4d by lazy { cameraExtrinsics.toTRC() }

    companion object {
        /**
         * Load from the canonical path `<repoRoot>/config/ball_tracker.json`.
         * Walks up from the `projectDir` system property (set by the test
         * harness to the `:TeamCode` subproject) — and from `user.dir` as a
         * fallback — until a directory containing `config/ball_tracker.json`
         * is found. This lets tests and on-robot callers share one entrypoint.
         */
        fun loadDefault(): TrackerConfig =
            loadFromFile(findDefaultConfigFile())

        internal fun findDefaultConfigFile(): File {
            val starts = listOfNotNull(
                System.getProperty("projectDir"),
                System.getProperty("user.dir"),
            ).map { File(it) }

            for (start in starts) {
                var dir: File? = start
                while (dir != null) {
                    val candidate = File(dir, "config/ball_tracker.json")
                    if (candidate.isFile) return candidate
                    dir = dir.parentFile
                }
            }
            throw IllegalStateException(
                "Could not locate config/ball_tracker.json starting from " +
                        "projectDir=${System.getProperty("projectDir")} " +
                        "user.dir=${System.getProperty("user.dir")}"
            )
        }

        fun loadFromFile(file: File): TrackerConfig {
            require(file.isFile) { "TrackerConfig file not found: ${file.absolutePath}" }
            return FileReader(file).use { fromJson(it) }
        }

        fun loadFromString(json: String): TrackerConfig =
            fromJson(StringReader(json))

        fun fromJson(reader: Reader): TrackerConfig {
            val root = Gson().fromJson(reader, JsonObject::class.java)
            return fromJsonObject(root)
        }

        internal fun fromJsonObject(root: JsonObject): TrackerConfig {
            val k = root.getAsJsonArray("K").toDoubleList()
            require(k.size == 4) { "K must have 4 entries: [fx, fy, cx, cy]. Got ${k.size}." }
            val dist = root.getAsJsonArray("DIST_COEFFS").toDoubleList()
            require(dist.size == 5) { "DIST_COEFFS must have 5 entries: [k1,k2,p1,p2,k3]. Got ${dist.size}." }

            val camPos = root.getAsJsonArray("CAM_POS_R").toDoubleList()
            require(camPos.size == 3) { "CAM_POS_R must be length 3 (x_fwd, y_left, z_up)." }

            return TrackerConfig(
                fieldWidthM = root.getAsJsonPrimitive("FIELD_WIDTH_M").asDouble,
                fieldHeightM = root.getAsJsonPrimitive("FIELD_HEIGHT_M").asDouble,
                fieldMarginM = root.getAsJsonPrimitive("FIELD_MARGIN_M").asDouble,

                rampPolygon = root.getAsJsonArray("RAMP_POLYGON").toVector2dList(),
                rampExpandM = root.getAsJsonPrimitive("RAMP_EXPAND_M").asDouble,

                ballRadiusM = root.getAsJsonPrimitive("BALL_RADIUS_M").asDouble,

                intrinsics = Intrinsics(
                    fx = k[0], fy = k[1], cx = k[2], cy = k[3],
                    k1 = dist[0], k2 = dist[1], p1 = dist[2], p2 = dist[3], k3 = dist[4],
                ),
                imageWidthPx = root.getAsJsonPrimitive("IMAGE_WIDTH_PX").asInt,
                imageHeightPx = root.getAsJsonPrimitive("IMAGE_HEIGHT_PX").asInt,

                cameraExtrinsics = CameraExtrinsicsR(
                    camPosR = Vector3d(camPos[0], camPos[1], camPos[2]),
                    pitchDownRad = root.getAsJsonPrimitive("CAM_PITCH_DOWN_RAD").asDouble,
                    yawRad = root.getAsJsonPrimitive("CAM_YAW_RAD").asDouble,
                    rollRad = root.getAsJsonPrimitive("CAM_ROLL_RAD").asDouble,
                ),

                robotFootprintR = root.getAsJsonArray("ROBOT_FOOTPRINT_R").toVector2dList(),

                intakeMaskYMinFrac = root.getAsJsonPrimitive("INTAKE_MASK_Y_MIN_FRAC").asDouble,

                sigmaPx = root.getAsJsonPrimitive("SIGMA_PX").asDouble,
                maxRangeM = root.getAsJsonPrimitive("MAX_RANGE_M").asDouble,

                sigmaAMps2 = root.getAsJsonPrimitive("SIGMA_A_MPS2").asDouble,
                initVelVar = root.getAsJsonPrimitive("INIT_VEL_VAR").asDouble,
                chi2Gate = root.getAsJsonPrimitive("CHI2_GATE").asDouble,
                maxCoastFrames = root.getAsJsonPrimitive("MAX_COAST_FRAMES").asInt,
                minHitsForConfirmed = root.getAsJsonPrimitive("MIN_HITS_FOR_CONFIRMED").asInt,
            )
        }

        private fun JsonArray.toDoubleList(): List<Double> =
            (0 until size()).map { get(it).asDouble }

        private fun JsonArray.toVector2dList(): List<Vector2d> =
            (0 until size()).map { idx ->
                val pair = this[idx].asJsonArray
                require(pair.size() == 2) { "Polygon vertex must be [x, y]; got ${pair.size()} entries." }
                Vector2d(pair[0].asDouble, pair[1].asDouble)
            }
    }
}
