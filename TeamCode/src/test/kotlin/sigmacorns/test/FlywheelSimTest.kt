package sigmacorns.test

import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import sigmacorns.State
import sigmacorns.constants.flywheelParameters
import sigmacorns.io.SimIO
import kotlin.time.Duration.Companion.seconds

class FlywheelSimTest {

    @Test
    fun spinUpApproachesSteadyState() {
        val io = SimIO()

        io.flyWheel0 = 1.0
        io.flyWheel1 = 1.0

        val state = State(io)

        while(io.time() < 3.seconds) {
            io.update()
            state.update(io)

        }

        val omega = io.flywheelVelocity()
        val freeSpeed = flywheelParameters.motor.freeSpeed

        assertTrue(omega > 0.75 * freeSpeed, "flywheel should spin up near free speed, but was $omega rad/s")
        assertTrue(omega < freeSpeed * 1.01, "flywheel should not exceed free speed significantly, but was $omega rad/s")
    }

    @Test
    fun flywheelRespondsToPowerChanges() {
        val io = SimIO()

        io.flyWheel0 = 1.0
        io.flyWheel1 = 1.0

        repeat(400) {
            io.update()
        }

        val peak = io.flywheelVelocity()

        io.flyWheel0 = 0.0
        io.flyWheel1 = 0.0

        repeat(200) {
            io.update()
        }

        val coast = io.flywheelVelocity()
        assertTrue(coast in 0.0..peak, "flywheel should coast down when unpowered")

        io.flyWheel0 = -1.0
        io.flyWheel1 = -1.0

        repeat(400) {
            io.update()
        }

        val reversed = io.flywheelVelocity()
        assertTrue(reversed < -200.0, "flywheel should spin in reverse under negative power")
    }
}
