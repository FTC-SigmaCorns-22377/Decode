package sigmacorns.logic

import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.ShootWhileMoveConstants
import sigmacorns.constants.drivetrainParameters
import sigmacorns.math.PolygonOverlapDetection
import sigmacorns.subsystem.Flywheel
import kotlin.time.Duration

/**
 * Cross-subsystem coordinator for the shooting pipeline.
 *
 * Bridges the aiming system with downstream subsystems (hood, flywheel, turret)
 * and handles auto-shoot zone detection and shoot-while-move compensation.
 */
class ShooterCoordinator(val robot: Robot) {

    var shootWhileMoveEnabled = false
    var autoShootEnabled = false

    /**
     * Called every loop before subsystem updates.
     * Feeds aiming data into hood/flywheel and handles auto-shoot + shoot-while-move.
     */
    fun update(dt: Duration) {
        // Feed hood inputs from aiming system
        robot.hood.targetDistance = robot.aim.targetDistance
        robot.hood.recommendedAngleDeg = robot.aim.adaptiveTuner
            .getRecommendedHoodAngle(robot.aim.targetDistance)

        // Feed flywheel target from aiming system
        if (robot.aimFlywheel) {
            robot.flywheel.target = robot.aim.getRecommendedFlywheelVelocity() ?: 0.0
        }

        // Auto-shoot zone detection
        if (autoShootEnabled) {
            if (isRobotInShootingZone()) {
                robot.intakeTransfer.disengageBlocker()
            } else {
                robot.intakeTransfer.engageBlocker()
            }
        }

        // Shoot-while-move turret lead and flywheel compensation
        if (shootWhileMoveEnabled) {
            robot.aim.updateVelocityComponents()
            robot.turret.targetAngleOffset = robot.aim.turretLeadAngle
            robot.flywheel.target = Flywheel.calculateTargetRPM(
                robot.aim.targetDistance + robot.aim.radialVelocity * ShootWhileMoveConstants.flywheelLookAheadTime
            )
        } else {
            robot.turret.targetAngleOffset = 0.0
        }
    }

    /**
     * Check if the robot's current position overlaps a shooting zone.
     * Moved from the old Transfer subsystem.
     */
    private fun isRobotInShootingZone(): Boolean {
        val pos = robot.aim.autoAim.fusedPose

        val robotCorners = PolygonOverlapDetection.robotCorners(
            pos.v.x(),
            pos.v.y(),
            pos.rot,
            drivetrainParameters.lx * 2.0,
            drivetrainParameters.ly * 2.0,
            0.0
        )

        return (
            PolygonOverlapDetection.doPolygonsOverlap(robotCorners, FieldLandmarks.farZoneCorners)
                ||
            PolygonOverlapDetection.doPolygonsOverlap(robotCorners, FieldLandmarks.goalZoneCorners)
        )
    }
}
