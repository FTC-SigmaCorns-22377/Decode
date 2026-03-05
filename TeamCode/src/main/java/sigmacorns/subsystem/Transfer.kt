package sigmacorns.subsystem

import sigmacorns.Robot
import sigmacorns.io.SigmaIO
import kotlin.time.Duration

class Transfer(val robot: Robot) {

    var isRunning = false

    fun update(dt: Duration) {



    }

    suspend fun disengageBlocker() {
        // set servo to not block artifact transferring
    }

    suspend fun engageBlocker() {
        // set servo to block artifact transferring
    }

    suspend fun startTransfer() {
        isRunning = true
        robot.intake.isRunning = true
        disengageBlocker()
    }

    suspend fun stopTransfer() {
        isRunning = false
        engageBlocker()
    }

}