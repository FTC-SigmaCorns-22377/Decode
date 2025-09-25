package sigmacorns.test

import org.joml.Vector2d
import org.junit.jupiter.api.Test
import sigmacorns.State
import sigmacorns.io.RerunLogging
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.RobotModel
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class MecanumSimTest {
    @Test
    fun testForward() {
        val io = SimIO()

        io.driveFL = 1.0
        io.driveBL = 1.0
        io.driveBR = 1.0
        io.driveFR = 1.0

        for (i in 0..100) {
            io.update()
        }
    }

    @Test
    fun testCircleOpenLoop() {
        val io = SimIO()
        val model = RobotModel()

        val rr = RerunLogging()
        val connection = rr.connect("mecanumTestCircleOpenLoop", "rerun+http://127.0.0.1:9876/proxy")

        val state = State(
            0.0,
            Pose2d(0.0,0.0,0.0),
            Pose2d(0.0,0.0,0.0),
            Pose2d(0.0,0.0,0.0),
            0.0,
            0.0
        )

        for (i in 0..100) {
            val angle = i.toDouble()/50.0* 2*PI

            val v = Vector2d(cos(angle),sin(angle))

            val powers = model.drivetrain.mecanumInverseKinematics(Pose2d(v, 0.0))

            io.driveFL = powers[0]
            io.driveBL = powers[1]
            io.driveBR = powers[2]
            io.driveFR = powers[3]

            io.update()
            state.update(io)
            rr.logState(connection,state.toNativeArray())
        }

        rr.destroy(connection)
    }
}