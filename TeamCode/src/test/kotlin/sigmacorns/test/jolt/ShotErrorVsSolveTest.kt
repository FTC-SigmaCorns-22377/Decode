package sigmacorns.test.jolt

import org.joml.Vector2d
import org.joml.Vector3d
import org.junit.jupiter.api.Test
import sigmacorns.control.aim.Ballistics
import kotlin.math.hypot
import kotlin.math.sqrt

class ShotErrorVsSolveTest {

    private val rH = 0.0785
    private val turretZ = 0.313
    private val goalHeight = 1.14

    private val ballistics = Ballistics(
        rH = rH,
        vMax = 15.0,
        phiMin = Math.toRadians(35.0),
        phiMax = Math.toRadians(63.0),
        g = 9.81
    )

    @Test
    fun testShotErrorEqualsZeroForSolvedShot() {
        // Test cases: (turretX, turretY, goalX, goalY)
        data class TestCase(val tX: Double, val tY: Double, val gX: Double, val gY: Double)
        val cases = listOf(
            // Close zone
            TestCase(0.0, 0.0, -1.58, 1.6),
            TestCase(-1.2, 0.5, -1.58, 1.6),
            // Far zone
            TestCase(0.0, -1.2, -1.58, 1.6),
            TestCase(0.0, -1.5, -1.58, 1.6),
            TestCase(0.0, -1.8, -1.58, 1.6),
            TestCase(0.5, -1.5, -1.58, 1.6),
        )

        println("=== shotError(solve(target, T), target) test ===")
        println("%-40s %8s %8s %12s %12s".format("Case", "dist(m)", "T(s)", "shotError", "phiDeg"))

        for (c in cases) {
            val tX = c.tX; val tY = c.tY; val gX = c.gX; val gY = c.gY
            val dist = hypot(gX - tX, gY - tY)
            val target = Ballistics.Target(
                target = Vector3d(gX, gY, goalHeight),
                turret = Vector3d(tX, tY, turretZ),
                vR = Vector2d(0.0, 0.0)
            )
            try {
                val bounds = ballistics.tBounds(target, 0.001)
                if (bounds.start >= bounds.endInclusive) {
                    println("($tX,$tY)->($gX,$gY): INFEASIBLE")
                    continue
                }

                for (frac in listOf(0.25, 0.5, 0.75)) {
                    val T = bounds.start + frac * (bounds.endInclusive - bounds.start)
                    val shot = ballistics.solve(target, T)
                    val err = ballistics.shotError(shot, target)
                    println(
                        "(%+.1f,%+.1f)->(%+.1f,%+.1f) dist=%5.2fm T=%.3fs err=%12.9f phi=%5.1f vExit=%5.2f"
                            .format(tX, tY, gX, gY, dist, T, err, Math.toDegrees(shot.phi), shot.vExit)
                    )
                }
            } catch (e: Exception) {
                println("($tX,$tY)->($gX,$gY): Exception ${e.message}")
            }
        }

        // Also test with robot velocity
        println("\n=== With robot velocity (far zone) ===")
        val vRobots = listOf(
            Vector2d(0.0, 0.0),
            Vector2d(1.0, 0.0),
            Vector2d(0.0, 1.0),
            Vector2d(-1.0, 1.0),
        )
        for (vR in vRobots) {
            val target = Ballistics.Target(
                target = Vector3d(-1.58, 1.6, goalHeight),
                turret = Vector3d(0.0, -1.5, turretZ),
                vR = vR
            )
            try {
                val bounds = ballistics.tBounds(target, 0.001)
                if (bounds.start >= bounds.endInclusive) { continue }
                val T = (bounds.start + bounds.endInclusive) / 2.0
                val shot = ballistics.solve(target, T)
                val err = ballistics.shotError(shot, target)
                println("vR=(%.1f,%.1f) T=%.3fs err=%12.9f phi=%.1f° vExit=%.2f"
                    .format(vR.x, vR.y, T, err, Math.toDegrees(shot.phi), shot.vExit))
            } catch (e: Exception) {
                println("vR=(${vR.x},${vR.y}): Exception ${e.message}")
            }
        }
    }
}
