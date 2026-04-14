package sigmacorns.test

import com.google.gson.GsonBuilder
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import sigmacorns.constants.flywheelRadius
import sigmacorns.control.aim.tune.PhysicsEstimateGenerator
import sigmacorns.logic.AimConfig
import sigmacorns.subsystem.ShooterConfig
import sigmacorns.constants.turretPos
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class PhysicsEstimateTest {

    private val points = PhysicsEstimateGenerator.generate()

    @Test
    fun generatesExpectedCount() {
        assertTrue(points.size >= 30, "Expected at least 30 points, got ${points.size}")
        assertTrue(points.size <= 50, "Expected at most 50 points, got ${points.size}")
    }

    @Test
    fun allMarkedAsPhysicsEstimate() {
        for (p in points) {
            assertTrue(p.physicsEstimate, "Point at d=${p.distance}m not marked as physics estimate")
        }
    }

    @Test
    fun distancesSpanRange() {
        val first = points.first().distance
        val last = points.last().distance
        // Short distances may be infeasible for lob trajectories with the hood angle constraints
        assertTrue(first <= 1.5, "First distance should be under 1.5m, was $first")
        assertTrue(last >= 4.5, "Last distance should be near 5.0m, was $last")
    }

    @Test
    fun hoodAnglesInRange() {
        for (p in points) {
            assertTrue(
                p.hoodAngle in ShooterConfig.minAngleDeg..ShooterConfig.maxAngleDeg,
                "Hood angle ${p.hoodAngle} out of range [${ShooterConfig.minAngleDeg}, ${ShooterConfig.maxAngleDeg}] at d=${p.distance}m"
            )
        }
    }

    @Test
    fun flywheelSpeedsAchievable() {
        val maxOmega = AimConfig.vMax / (flywheelRadius * AimConfig.launchEfficiency)
        for (p in points) {
            assertTrue(p.speed > 0.0, "Speed should be positive at d=${p.distance}m")
            assertTrue(
                p.speed <= maxOmega * 1.01,
                "Speed ${p.speed} rad/s exceeds max $maxOmega rad/s at d=${p.distance}m"
            )
        }
    }

    @Test
    fun roundTripDistanceVerification() {
        val h = AimConfig.goalHeight - turretPos.z
        val g = 9.81

        for (p in points) {
            val vExit = p.speed * flywheelRadius * AimConfig.launchEfficiency
            val phiRad = Math.toRadians(p.hoodAngle)

            val vy = vExit * sin(phiRad)
            val vx = vExit * cos(phiRad)
            val disc = vy * vy - 2.0 * g * h
            if (disc < 0.0) {
                fail<Unit>("No solution at d=${p.distance}m")
                continue
            }
            // Descending branch (lob) — use the larger root
            val t = (vy + sqrt(disc)) / g
            val predicted = vx * t

            val error = abs(predicted - p.distance)
            assertTrue(
                error < 0.05,
                "Round-trip distance error ${error}m too large at d=${p.distance}m (predicted=$predicted)"
            )
        }
    }

    @Test
    fun printGeneratedJson() {
        val gson = GsonBuilder().setPrettyPrinting().create()
        println("=== Generated ${points.size} physics estimate points ===")
        println(gson.toJson(points))
        println()
        println("=== CSV format ===")
        println("distance_m,speed_rad_s,speed_rpm,hood_angle_deg")
        for (p in points) {
            val rpm = p.speed * 60.0 / (2.0 * Math.PI)
            println("%.3f,%.1f,%.0f,%.1f".format(p.distance, p.speed, rpm, p.hoodAngle))
        }
    }
}
