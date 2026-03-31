package sigmacorns.logic

import sigmacorns.Robot
import sigmacorns.subsystem.IntakeTransfer

/**
 * Cross-subsystem coordinator for the intake-transfer and beam break subsystems.
 *
 * Enforces rules that span both subsystems:
 * - Feeds beam break full-state into IntakeTransfer
 * - Auto-stops intake when 3 balls are held
 * - Coordinates blocker engagement when starting intake
 */
class IntakeCoordinator(val robot: Robot) {

    /**
     * Called every loop before subsystem updates.
     * Propagates sensor state and enforces cross-subsystem rules.
     */
    fun update() {
        robot.intakeTransfer.isFull = robot.beamBreak.isFull

        if (robot.beamBreak.isFull) {
            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
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
}
