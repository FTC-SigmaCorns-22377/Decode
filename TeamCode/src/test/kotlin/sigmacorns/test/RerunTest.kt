package sigmacorns.test

import org.junit.jupiter.api.Test
import sigmacorns.State
import sigmacorns.io.RerunLogging
import sigmacorns.math.Pose2d

class RerunTest {
    @Test
    fun testStateLogging() {
        val state = State(
            1.0,
            Pose2d(2.0,3.0,4.0),
            Pose2d(5.0,6.0,7.0),
            Pose2d(8.0,9.0,10.0),
            11.0,
            12.0
        )

        val rr = RerunLogging()
        val connection = rr.connect("sim", "rerun+http://127.0.0.1:9876/proxy")
        rr.logState(connection,state.toNativeArray())
        rr.destroy(connection)
    }
}