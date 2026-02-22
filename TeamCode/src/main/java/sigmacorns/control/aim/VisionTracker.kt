package sigmacorns.control.aim

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.hardware.limelightvision.Limelight3A

data class VisionObservation(
    val tagId: Int,
    val corners: DoubleArray,
    val txDeg: Double,
    val tyDeg: Double
)

data class VisionFrame(
    val timestampSeconds: Double,
    val observations: List<VisionObservation>
)

data class VisionStatus(
    val lastResultValid: Boolean,
    val detectedTagCount: Int,
    val hasVisionTarget: Boolean,
    val trackedTagId: Int,
    val rawTxDegrees: Double,
    val rawTyDegrees: Double,
    val lastDetectionTimeMs: Long
)

data class VisionResult(
    val status: VisionStatus,
    val frame: VisionFrame?
)

class VisionTracker(
    private val limelight: Limelight3A?,
    private val allowedTagIds: Set<Int>,
    private val logger: Logger? = null,
    var enableDebugLogging: Boolean = false
) {
    fun read(): VisionResult {
        if (limelight == null) {
            return VisionResult(emptyStatus(), null)
        }

        val result: LLResult = limelight.latestResult
        if (!result.isValid) {
            logDebug("Limelight result invalid")
            return VisionResult(emptyStatus(), null)
        }

        val fiducials: List<FiducialResult> = result.fiducialResults
        val detectedTagCount = fiducials.size
        logDebug("Limelight fiducials=$detectedTagCount")

        var sawMatchingTag = false
        var trackedTagId = -1
        var rawTxDegrees = 0.0
        var rawTyDegrees = 0.0
        val observations = ArrayList<VisionObservation>(fiducials.size)

        for (fiducial in fiducials) {
            val tagId = fiducial.fiducialId
            if (tagId !in allowedTagIds) continue
            sawMatchingTag = true

            if (trackedTagId == -1) {
                trackedTagId = tagId
                rawTxDegrees = fiducial.targetXDegrees
                rawTyDegrees = fiducial.targetYDegrees
            }

            println("RAW CORNERS: ${fiducial.targetCorners}")
            val cornersArray = extractCornerArray(fiducial.targetCorners, tagId) ?: continue
            println("EXTRACTED CORNERS ${cornersArray.contentToString()}")
            observations.add(
                VisionObservation(
                    tagId = tagId,
                    corners = cornersArray,
                    txDeg = fiducial.targetXDegrees,
                    tyDeg = fiducial.targetYDegrees
                )
            )
        }

        val lastDetectionTimeMs = result.controlHubTimeStamp
        val status = VisionStatus(
            lastResultValid = true,
            detectedTagCount = detectedTagCount,
            hasVisionTarget = sawMatchingTag,
            trackedTagId = if (sawMatchingTag) trackedTagId else -1,
            rawTxDegrees = if (sawMatchingTag) rawTxDegrees else 0.0,
            rawTyDegrees = if (sawMatchingTag) rawTyDegrees else 0.0,
            lastDetectionTimeMs = if (sawMatchingTag) lastDetectionTimeMs else 0L
        )

        val frame = if (observations.isNotEmpty()) {
            VisionFrame(
                timestampSeconds = lastDetectionTimeMs / 1000.0,
                observations = observations
            )
        } else {
            null
        }

        if (sawMatchingTag && observations.isEmpty()) {
            logDebug("Matching tag(s) seen but not enqueued")
        }

        return VisionResult(status, frame)
    }

    private fun emptyStatus(): VisionStatus {
        return VisionStatus(
            lastResultValid = false,
            detectedTagCount = 0,
            hasVisionTarget = false,
            trackedTagId = -1,
            rawTxDegrees = 0.0,
            rawTyDegrees = 0.0,
            lastDetectionTimeMs = 0L
        )
    }

    private fun extractCornerArray(corners: List<List<Double>>, tagId: Int): DoubleArray? {
        val corners = corners.reversed()
        if (corners.size >= 4 && corners.all { it.size >= 2 }) {
            val cornersArray = DoubleArray(8)
            for (i in 0 until 4) {
                val corner = corners[i]
                cornersArray[i * 2] = corner[0]
                cornersArray[i * 2 + 1] = corner[1]
            }
            return cornersArray
        }

        val flat = corners.flatMap { it.asIterable() }
        if (flat.size >= 8) {
            return DoubleArray(8) { i -> flat[i] }
        }

        logDebug(
            "Skipping tag $tagId (invalid corner data size=${corners.size} " +
                "lens=${corners.joinToString(prefix = "\"[\"", postfix = "\"]\"") { it.size.toString() }})"
        )
        return null
    }

    private fun logDebug(message: String) {
        if (enableDebugLogging) {
            logger?.log(LogLevel.DEBUG, message)
        }
    }
}
