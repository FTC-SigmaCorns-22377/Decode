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
 * - Auto-shoot zone detection (transitions to READY_TO_SHOOT state when in zone)
 *
 * Only sets state on IntakeTransfer — never writes IO directly.
 */
class IntakeCoordinator(val robot: Robot) {

    /** When true, transitions to READY_TO_SHOOT state when robot enters a shooting zone. */
    var autoShootEnabled: Boolean = false

    /**
     * Called every loop before subsystem updates.
     * Propagates sensor state and enforces cross-subsystem rules.
     */
    fun update() {
        robot.intakeTransfer.isFull = robot.beamBreak.isFull

        if (robot.beamBreak.isFull && robot.intakeTransfer.state == IntakeTransfer.State.INTAKING) {
            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        }

        // Auto-shoot zone detection: open blocker via READY_TO_SHOOT state
        if (autoShootEnabled) {
            if (isRobotInShootingZone()) {
                robot.intakeTransfer.state = IntakeTransfer.State.READY_TO_SHOOT
            } else if (robot.intakeTransfer.state == IntakeTransfer.State.READY_TO_SHOOT) {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }
        }
    }

    /**
     * Coordinated intake start: ensures clean state, then starts intake.
     */
    fun startIntake() {
        if (robot.beamBreak.isFull) return
        robot.intakeTransfer.state = IntakeTransfer.State.INTAKING
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
