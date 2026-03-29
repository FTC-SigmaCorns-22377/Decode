package sigmacorns.subsystem

import kotlinx.coroutines.yield
import sigmacorns.Robot
import sigmacorns.constants.ShootWhileMoveConstants

class Logic(val robot: Robot) {

    var shootWhileMoveEnabled = false

    suspend fun shootWhileMove() {
        shootWhileMoveEnabled = true

        while (shootWhileMoveEnabled) {
            robot.aim.updateVelocityComponents()
            val turretLeadAngle = robot.aim.turretLeadAngle
            val radialVelocity = robot.aim.radialVelocity

            // update turret PID and setpoint
            robot.turret.targetAngleOffset = turretLeadAngle

            // set flywheel setpoint to radialVelocity*flywheelLookAheadTime
            robot.flywheel.target = Flywheel.calculateTargetRPM(robot.aim.targetDistance + radialVelocity*ShootWhileMoveConstants.flywheelLookAheadTime)

            yield()
        }

        robot.turret.targetAngleOffset = 0.0
    }

}