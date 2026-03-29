package sigmacorns.subsystem

import kotlinx.coroutines.delay
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.drivetrainParameters
import sigmacorns.math.PolygonOverlapDetection
import kotlin.time.Duration

/**
 * Transfer subsystem: manages the blocker servo and the transfer state
 * that feeds balls from the intake into the shooter.
 *
 * The blocker prevents balls from accidentally entering the shooter.
 * During intaking the blocker is engaged (blocking). To shoot, disengage
 * the blocker, wait for it to physically move, then start the transfer.
 */
class Transfer(val robot: Robot) {

    /** Whether the transfer pathway is actively running (feeding balls to shooter). */
    var isRunning = false

    /** When true, auto-disengages blocker when robot enters a shooting zone. */
    var autoShoot: Boolean = false

    companion object {
        const val BLOCKER_ENGAGED = 0.0   // servo position: blocks transfer path
        const val BLOCKER_DISENGAGED = 1.0 // servo position: opens transfer path
        const val TRANSFER_POWER = 1.0    // intake motor power when transferring
        const val BLOCKER_MOVE_DELAY_MS = 300L // ms to wait for blocker to physically move
    }

    fun update(dt: Duration) {
        if (autoShoot) {
            if (isRobotInShootingZone()) {
                disengageBlocker()
            } else {
                engageBlocker()
            }
        }

        // When transfer is running, drive the intake motor to push balls through
        if (isRunning) {
            robot.io.intake = TRANSFER_POWER
        }
    }

    fun isRobotInShootingZone(): Boolean {
        val pos = robot.aim.autoAim.fusedPose

        val robotCorners = PolygonOverlapDetection.robotCorners(
            pos.v.x(),
            pos.v.y(),
            pos.rot,
            drivetrainParameters.lx*2.0,
            drivetrainParameters.ly*2.0,
            0.0
        )

        return (
            PolygonOverlapDetection.doPolygonsOverlap(robotCorners, FieldLandmarks.farZoneCorners)
                ||
            PolygonOverlapDetection.doPolygonsOverlap(robotCorners, FieldLandmarks.goalZoneCorners)
        )
    }

    fun disengageBlocker() {
        robot.io.blocker = BLOCKER_DISENGAGED
    }

    fun engageBlocker() {
        robot.io.blocker = BLOCKER_ENGAGED
    }

    /**
     * Begin transferring balls to the shooter.
     * Disengages blocker first and waits 300ms for it to physically move
     * before activating the transfer motor.
     */
    suspend fun startTransfer() {
        disengageBlocker()
        delay(BLOCKER_MOVE_DELAY_MS)
        isRunning = true
    }

    /** Stop transferring. Re-engages blocker. */
    suspend fun stopTransfer() {
        isRunning = false
        engageBlocker()
    }
}
