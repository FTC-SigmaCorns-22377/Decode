package sigmacorns.logic

import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.drivetrainParameters
import sigmacorns.math.PolygonOverlapDetection
import sigmacorns.subsystem.IntakeTransfer

/**
 * Cross-subsystem coordinator for the intake-transfer and beam break subsystems.
 *
 * Enforces rules that span both subsystems:
 * - Feeds beam break full-state into IntakeTransfer
 * - Auto-stops intake when 3 balls are held
 * - Coordinates blocker engagement when starting intake
 * - Auto-shoot zone detection (disengages blocker when in shooting zone)
 */
class IntakeCoordinator(val robot: Robot) {

    /** When true, auto-disengages blocker when robot enters a shooting zone. */
    var autoShootEnabled: Boolean = false

    /**
     * Called every loop before subsystem updates.
     * Propagates sensor state and enforces cross-subsystem rules.
     */
    fun update() {
        robot.intakeTransfer.isFull = robot.beamBreak.isFull

        if (robot.beamBreak.isFull) {
            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        }

        // Auto-shoot zone detection: disengage blocker when in shooting zone
        if (autoShootEnabled) {
            if (isRobotInShootingZone()) {
                robot.intakeTransfer.disengageBlocker()
            } else {
                robot.intakeTransfer.engageBlocker()
            }
        }
    }

    /**
     * Coordinated intake start: engages blocker to prevent accidental shots,
     * stops any active transfer, then starts intake.
     */
    fun startIntake() {
        if (robot.beamBreak.isFull) return
        robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        robot.intakeTransfer.engageBlocker()
        robot.intakeTransfer.startIntake()
    }

    /**
     * Check if the robot's current position overlaps a shooting zone.
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
