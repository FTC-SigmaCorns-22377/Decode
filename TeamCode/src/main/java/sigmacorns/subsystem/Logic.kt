package sigmacorns.subsystem

import kotlinx.coroutines.delay
import kotlinx.coroutines.yield
import sigmacorns.Robot
import sigmacorns.io.SigmaIO

class Logic(val robot: Robot) {

    var shootWhileMoveEnabled = false;

    suspend fun shootWhileMove() {
        shootWhileMoveEnabled = true;

        while (shootWhileMoveEnabled) {
            // calculate and update turntable feedforward constant
            // calculate and update flywheel target
            yield()
        }
    }

}