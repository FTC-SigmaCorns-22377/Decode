package sigmacorns.subsystem

import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.drivetrainParameters
import sigmacorns.io.SigmaIO
import sigmacorns.math.PolygonOverlapDetection
import kotlin.time.Duration

class Transfer(val robot: Robot) {

    var isRunning = false
    var autoShoot: Boolean = false

    fun update(dt: Duration) {

        if (autoShoot) {
            if (isRobotInShootingZone()) {
                disengageBlocker()
            } else {
                engageBlocker()
            }
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
        // set servo to not block artifact transferring
    }

    fun engageBlocker() {
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