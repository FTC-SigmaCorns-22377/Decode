package sigmacorns.control.aim.tune

import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import sigmacorns.constants.flywheelRadius
import sigmacorns.logic.AimConfig
import sigmacorns.constants.turretPos
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Generates physics-estimated [SpeedPoint]s from projectile motion equations.
 *
 * For each target distance, sweeps hood angles and computes the minimum launch
 * velocity needed to reach that distance at the goal height. Converts to
 * flywheel omega and picks the energy-optimal hood angle.
 *
 * All points are marked [SpeedPoint.physicsEstimate] = true.
 */
object PhysicsEstimateGenerator {

    private const val G = 9.81
    private const val RESOURCE_PATH = "/shot_tuning_data.json"

    /**
     * Load pre-computed physics estimates from the bundled resource file.
     * Falls back to [generate] if the resource is unavailable.
     */
    fun loadFromResources(): List<SpeedPoint> {
        return try {
            val json = PhysicsEstimateGenerator::class.java
                .getResourceAsStream(RESOURCE_PATH)
                ?.bufferedReader()
                ?.readText()
                ?: return generate()

            val type = object : TypeToken<List<SpeedPoint>>() {}.type
            Gson().fromJson<List<SpeedPoint>>(json, type) ?: generate()
        } catch (_: Exception) {
            generate()
        }
    }

    /**
     * Generate physics-estimated data points spanning [minDistance] to [maxDistance].
     *
     * Uses flat-earth projectile motion:
     *   h = d * tan(phi) - g * d^2 / (2 * v^2 * cos^2(phi))
     * solved for v^2:
     *   v^2 = g * d^2 / (2 * cos^2(phi) * (d * tan(phi) - h))
     *
     * where h = goalHeight - launchHeight.
     *
     * @param numPoints number of evenly spaced distances to evaluate
     * @param minDistance closest target distance in metres
     * @param maxDistance farthest target distance in metres
     * @return list of [SpeedPoint]s, possibly fewer than [numPoints] if some distances are infeasible
     */
    fun generate(
        numPoints: Int = 50,
        minDistance: Double = 0.5,
        maxDistance: Double = 5.0,
    ): List<SpeedPoint> {
        val h = AimConfig.goalHeight - turretPos.z  // height difference (m)
        val maxVExit = AimConfig.vMax
        val maxOmega = maxVExit / (flywheelRadius * AimConfig.launchEfficiency)

        val step = (maxDistance - minDistance) / (numPoints - 1).coerceAtLeast(1)
        val results = mutableListOf<SpeedPoint>()

        val phiMinRad = Math.toRadians(ShooterConfig.minAngleDeg)
        val phiMaxRad = Math.toRadians(ShooterConfig.maxAngleDeg)
        val phiSteps = 100

        for (i in 0 until numPoints) {
            val distance = minDistance + i * step

            var bestVExit = Double.MAX_VALUE
            var bestPhiDeg = ShooterConfig.defaultAngleDeg

            for (j in 0..phiSteps) {
                val phiRad = phiMinRad + (phiMaxRad - phiMinRad) * j / phiSteps
                val cosPhi = cos(phiRad)
                val tanPhi = tan(phiRad)

                val denom = distance * tanPhi - h
                if (denom <= 0.0) continue // angle too shallow to reach height

                val vSq = G * distance * distance / (2.0 * cosPhi * cosPhi * denom)
                if (vSq <= 0.0) continue

                val v = sqrt(vSq)
                if (v > maxVExit) continue

                // Verify this is a lob (ball descending at target)
                val vy = v * sin(phiRad)
                val t = distance / (v * cosPhi)
                val vyAtTarget = vy - G * t
                if (vyAtTarget >= 0.0) continue // not a lob

                if (v < bestVExit) {
                    bestVExit = v
                    bestPhiDeg = Math.toDegrees(phiRad)
                }
            }

            if (bestVExit == Double.MAX_VALUE) continue

            val omega = bestVExit / (flywheelRadius * AimConfig.launchEfficiency)
            val clampedHood = bestPhiDeg.coerceIn(ShooterConfig.minAngleDeg, ShooterConfig.maxAngleDeg)
            val clampedOmega = omega.coerceIn(0.0, maxOmega)

            results.add(
                SpeedPoint(
                    distance = distance,
                    speed = clampedOmega,
                    hoodAngle = clampedHood,
                    physicsEstimate = true,
                )
            )
        }

        return results
    }
}
