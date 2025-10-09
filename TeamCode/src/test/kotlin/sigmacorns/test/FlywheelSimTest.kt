package sigmacorns.test

import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import sigmacorns.State
import sigmacorns.constants.flywheelParameters
import sigmacorns.io.SimIO
import kotlin.time.Duration.Companion.seconds

private val flywheelFreeSpeed = flywheelParameters.motor.freeSpeed

class FlywheelSimTest {

    @Test
    fun spinUpApproachesSteadyState() {
        val io = SimIO()

        io.shooter = 1.0

        val state = State(io)

        while (io.time() < 3.seconds) {
            io.update()
            state.update(io)
        }

        val omega = io.flywheelVelocity()

        assertTrue(omega > 0.75 * flywheelFreeSpeed, "flywheel should spin up near free speed, but was $omega rad/s")
        assertTrue(omega < flywheelFreeSpeed * 1.01, "flywheel should not exceed free speed significantly, but was $omega rad/s")
    }

    @Test
    fun flywheelRespondsToPowerChanges() {
        val io = SimIO()

        io.shooter = 1.0

        repeat(400) {
            io.update()
        }

        val peak = io.flywheelVelocity()

        io.shooter = 0.0

        repeat(200) {
            io.update()
        }

        val coast = io.flywheelVelocity()
        assertTrue(coast in 0.0..peak, "flywheel should coast down when unpowered")

        io.shooter = -1.0

        repeat(400) {
            io.update()
        }

        val reversed = io.flywheelVelocity()
        assertTrue(reversed < -0.95 * flywheelFreeSpeed, "flywheel should spin in reverse under negative power, but was $reversed rad/s")
    }
}
