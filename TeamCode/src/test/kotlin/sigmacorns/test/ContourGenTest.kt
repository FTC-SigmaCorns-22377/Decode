package sigmacorns.test

import org.jetbrains.kotlinx.kandy.dsl.plot
import org.jetbrains.kotlinx.kandy.letsplot.export.save
import org.jetbrains.kotlinx.kandy.letsplot.layers.path
import org.jetbrains.kotlinx.kandy.letsplot.layers.points
import org.junit.jupiter.api.Test
import sigmacorns.control.mpc.MPCClient
import sigmacorns.control.mpc.TrajoptLoader
import java.io.File
import kotlin.math.cos
import kotlin.math.sin

class ContourGenTest {
    @Test
    fun testParsing() {
        // Load from trajopt directory
        val trajoptDir = File(System.getProperty("trajoptDir") ?: "trajopt")
        val projectFiles = TrajoptLoader.findProjectFiles(trajoptDir)
        if (projectFiles.isEmpty()) {
            println("No trajopt project files found in $trajoptDir, skipping test")
            return
        }

        val traj = TrajoptLoader.loadFirstTrajectory(projectFiles.first())
        if (traj == null) {
            println("No trajectory found in project file, skipping test")
            return
        }

        val contours = MPCClient.load(traj)

        plot {
            // Plot the path
            path {
                x(contours.map { it.lineP.x })
                y(contours.map { it.lineP.y })
            }

            // Plot direction arrows at each contour point
            for (contour in contours) {
                val arrowLen = 0.05
                val xs = listOf(
                    contour.lineP.x,
                    contour.lineP.x + arrowLen * contour.lineD.x
                )
                val ys = listOf(
                    contour.lineP.y,
                    contour.lineP.y + arrowLen * contour.lineD.y
                )
                path {
                    x(xs)
                    y(ys)
                }

                // Also show heading direction
                val headingLen = 0.03
                val headingXs = listOf(
                    contour.lineP.x,
                    contour.lineP.x + headingLen * cos(contour.targetTheta)
                )
                val headingYs = listOf(
                    contour.lineP.y,
                    contour.lineP.y + headingLen * sin(contour.targetTheta)
                )
                path {
                    x(headingXs)
                    y(headingYs)
                }
            }

            // Plot contour points
            points {
                x(contours.map { it.lineP.x })
                y(contours.map { it.lineP.y })
            }
        }.save("MPCCTestContours.png")
    }
}
