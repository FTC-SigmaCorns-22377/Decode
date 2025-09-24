package sigmacorns.test

import org.junit.jupiter.api.Test
import sigmacorns.io.SimIO

class MecanumSimTest {
    @Test
    fun testForward() {
        val io = SimIO()

        io.driveFL = 1.0
        io.driveBL = 1.0
        io.driveBR = 1.0
        io.driveFR = 1.0

        for (i in 0..1000) {
            io.update()
        }
    }
}