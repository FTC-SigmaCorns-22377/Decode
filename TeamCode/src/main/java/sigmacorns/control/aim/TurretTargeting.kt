package sigmacorns.control.aim

import org.joml.Vector2d
import sigmacorns.math.Pose2d
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.PI

class TurretTargeting(private val goalPosition: Vector2d) {
    data class TargetAngles(
        val distance: Double,
        val fieldAngle: Double,
        val robotAngle: Double
    )

    fun computeAngles(fusedPose: Pose2d): TargetAngles {
        val dx = goalPosition.x - fusedPose.v.x
        val dy = goalPosition.y - fusedPose.v.y
        val distance = hypot(dx, dy)
        val fieldAngle = atan2(dy, dx)
        val robotAngle = normalizeAngle(fieldAngle - fusedPose.rot)
        return TargetAngles(distance, fieldAngle, robotAngle)
    }

    fun hasTarget(
        gtsamInitialized: Boolean,
        lastDetectionTimeMs: Long,
        predictionTimeoutMs: Long
    ): Boolean {
        if (gtsamInitialized) {
            return true
        }
        val timeSinceDetection = System.currentTimeMillis() - lastDetectionTimeMs
        return timeSinceDetection < predictionTimeoutMs
    }

    fun usingPrediction(
        hasVisionTarget: Boolean,
        uncertainty: Double,
        maxAcceptableUncertainty: Double
    ): Boolean {
        return !hasVisionTarget || uncertainty >= maxAcceptableUncertainty
    }

    fun turretAdjustment(
        enabled: Boolean,
        hasVisionTarget: Boolean,
        targetTxRad: Double,
        aimConfig: AimConfig
    ): Double {
        if (!enabled || !hasVisionTarget) {
            return 0.0
        }
        val adjustedTx = if (abs(targetTxRad) < aimConfig.txDeadband) {
            0.0
        } else {
            targetTxRad * aimConfig.txSignMultiplier
        }
        return adjustedTx
    }

    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle
        while (normalized > PI) normalized -= 2 * PI
        while (normalized < -PI) normalized += 2 * PI
        return normalized
    }
}
