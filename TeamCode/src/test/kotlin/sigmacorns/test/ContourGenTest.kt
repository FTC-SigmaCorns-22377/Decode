package sigmacorns.test

import dev.nullftc.choreolib.Choreo
import dev.nullftc.choreolib.sample.MecanumSample
import org.jetbrains.kotlinx.kandy.dsl.plot
import org.jetbrains.kotlinx.kandy.letsplot.export.save
import org.jetbrains.kotlinx.kandy.letsplot.layers.path
import org.junit.jupiter.api.Test
import sigmacorns.io.MPCClient
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class ContourGenTest {
    @Test
    fun testParsing() {
        val contours = MPCClient.load(Choreo().loadTrajectory<MecanumSample>("New Path (2)").get())

        plot {
            path {
                x(contours.map { it.pos.v.x })
                y(contours.map { it.pos.v.y })
            }

            for (t in contours) {
                val p = t.pos
                val theta = t.tangent
                val cx = p.v.x + t.r * sin(theta)
                val cy = p.v.y - t.r * cos(theta)
                val res = 10
                val dist = 1.0
                val theta0 = atan2(p.v.y - cy, p.v.x - cx)

                val xs = (0 until res).map { j ->
                    cx + abs(t.r) * cos(theta0 + j / res.toDouble() * dist / -t.r)
                }
                val ys = (0 until res).map { j ->
                    cy + abs(t.r) * sin(theta0 + j / res.toDouble() * dist / -t.r)
                }

                path {
                    x(xs)
                    y(ys)
                }
            }
        }.save("MPCCTestContours.png")
    }
}