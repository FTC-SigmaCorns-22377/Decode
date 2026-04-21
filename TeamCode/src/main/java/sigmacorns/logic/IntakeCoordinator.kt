package sigmacorns.logic

import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.robotSize
import sigmacorns.constants.turretPos
import sigmacorns.math.PolygonOverlapDetection
import sigmacorns.subsystem.IntakeTransfer
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Cross-subsystem coordinator for the intake-transfer and beam break subsystems.
 *
 * Enforces rules that span both subsystems:
 * - Coordinates blocker engagement when starting intake
 * - Auto-shoot zone detection (transitions to READY_TO_SHOOT state when in zone)
 *
 * Only sets state on IntakeTransfer — never writes IO directly.
 */
class IntakeCoordinator(val robot: Robot) {

    /** When true, transitions to READY_TO_SHOOT state when robot enters a shooting zone. */
    var autoShootEnabled: Boolean = false

    /** Set by the opmode when the driver is manually holding a shoot trigger — prevents coordinator cleanup from overriding TRANSFERRING. */
    var manualShooting: Boolean = false

    /** Cached result of the shooting zone check, updated once per [update] call. */
    var inShootingZone: Boolean = false
        private set

    /**
     * Called every loop before subsystem updates.
     * Propagates sensor state and enforces cross-subsystem rules.
     */
    fun update() {
        inShootingZone = computeInShootingZone()

        // Always feed shotRequested so NativeAutoAim keeps tracking turret/hood/flywheel
        if (autoShootEnabled && robot.aimFlywheel) {
            robot.aim.shotRequested = inShootingZone
        }

        // Manual shoot holds TRANSFERRING — skip all state writes, auto-aim still runs above
        if (manualShooting) return

        // Zone-based blocker control (non-aimFlywheel path)
        if (autoShootEnabled && !robot.aimFlywheel) {
            val cur = robot.intakeTransfer.state
            if (inShootingZone && cur == IntakeTransfer.State.IDLE) {
                robot.intakeTransfer.state = IntakeTransfer.State.READY_TO_SHOOT
            } else if (!inShootingZone && (cur == IntakeTransfer.State.READY_TO_SHOOT
                    || cur == IntakeTransfer.State.TRANSFERRING)) {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }
        }

        if (robot.aimFlywheel && robot.aim.shotRequested) {
            val cur = robot.intakeTransfer.state
            if (robot.aim.readyToShoot && cur == IntakeTransfer.State.IDLE) {
                robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
            } else if (cur == IntakeTransfer.State.TRANSFERRING && robot.intakeTransfer.blockerReady && !robot.aim.readyToShoot) {
                robot.intakeTransfer.state = IntakeTransfer.State.READY_TO_SHOOT
            } else if (cur == IntakeTransfer.State.IDLE) {
                robot.intakeTransfer.state = IntakeTransfer.State.READY_TO_SHOOT
            }
        } else if ((robot.intakeTransfer.state == IntakeTransfer.State.READY_TO_SHOOT
            || robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING)
            && robot.aimFlywheel) {
            robot.intakeTransfer.state = IntakeTransfer.State.IDLE
        }
    }

    /**
     * Coordinated intake start: ensures clean state, then starts intake.
     * Always allowed when driver explicitly requests (trigger held) — zone/flywheel
     * rules are enforced reactively in update().
     */
    fun startIntake() {
        robot.intakeTransfer.state = IntakeTransfer.State.INTAKING
    }

    private fun computeInShootingZone(): Boolean {
        val turretCenter = robot.aim.autoAim.fusedPose
        val heading = turretCenter.rot

        // fusedPose is the turret center; offset back to robot body center
        val robotCenterX = turretCenter.v.x() - (turretPos.x * cos(heading) - turretPos.y * sin(heading))
        val robotCenterY = turretCenter.v.y() - (turretPos.x * sin(heading) + turretPos.y * cos(heading))

        val robotCorners = PolygonOverlapDetection.robotCorners(
            robotCenterX,
            robotCenterY,
            heading,
            robotSize.x,
            robotSize.y,
            0.0
        )

        return (
            PolygonOverlapDetection.doPolygonsOverlap(robotCorners, FieldLandmarks.farZoneCorners)
                ||
            PolygonOverlapDetection.doPolygonsOverlap(robotCorners, FieldLandmarks.goalZoneCorners)
        )
    }
}
