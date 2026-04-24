package sigmacorns.logic

import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.robotSize
import sigmacorns.constants.turretPos
import sigmacorns.math.PolygonOverlapDetection
import sigmacorns.subsystem.IntakeTransfer
import kotlin.time.Duration
import kotlin.math.cos
import kotlin.math.sin
import kotlin.time.Duration.Companion.seconds

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

    companion object {
        /** Flywheel velocity (rad/s) above which intake is suppressed. */
        const val FLYWHEEL_MIN_VEL = 50.0
        /** Minimum flywheel target (rad/s) considered a real shot — below this, auto-shoot is disabled (e.g. idle always-on). */
        const val MIN_SHOT_TARGET = 300.0
        val TRANSFER_DURATION = 3.seconds
    }

    /** When true, transitions to READY_TO_SHOOT state when robot enters a shooting zone. */
    var autoShootEnabled: Boolean = false

    /** Cached result of the shooting zone check, updated once per [update] call. */
    var inShootingZone: Boolean = false
        private set

    var overrideShot: Boolean = false

    /** Timestamp when native auto-aim first started transferring; null if not in burst. */
    private var transferStartTime: Duration? = null

    /**
     * Called every loop before subsystem updates.
     * Propagates sensor state and enforces cross-subsystem rules.
     */
    fun update() {
        val flywheelVel = robot.io.flywheelVelocity()
        val flywheelTarget = robot.shooter.flywheelTarget

        inShootingZone = computeInShootingZone()


        // Auto-shoot zone detection
        if (autoShootEnabled) {
            if(robot.aimFlywheel) {
                robot.aim.shotRequested = inShootingZone
            } else {
                val shooterAtSpeed = flywheelTarget >= MIN_SHOT_TARGET &&
                        kotlin.math.abs(flywheelVel - flywheelTarget) < FLYWHEEL_MIN_VEL

                if (inShootingZone && shooterAtSpeed) {
                    // In zone and ready — transfer immediately (blocker already open from pre-open below)
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                } else if (shooterAtSpeed) {
                    // Flywheel at speed but not yet in zone — pre-open blocker so entry is instant
                    robot.intakeTransfer.state = IntakeTransfer.State.READY_TO_SHOOT
                } else if (inShootingZone) {
                    // In zone but shooter still spinning up — hold blocker open, wait for speed
                    robot.intakeTransfer.state = IntakeTransfer.State.READY_TO_SHOOT
                } else if (robot.intakeTransfer.state == IntakeTransfer.State.READY_TO_SHOOT
                    || robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING
                    || (flywheelVel > FLYWHEEL_MIN_VEL && robot.intakeTransfer.state == IntakeTransfer.State.INTAKING)) {
                    robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                }
            }
        }

        if(robot.aimFlywheel && robot.aim.shotRequested) {
            robot.intakeTransfer.transferPowerOverride = robot.aim.transferPowerCommand.toDouble()
            val cur = robot.intakeTransfer.state
            if(robot.aim.readyToShoot && cur != IntakeTransfer.State.INTAKING) {
                if (transferStartTime == null) transferStartTime = robot.io.time()
                robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
            } else if(cur == IntakeTransfer.State.TRANSFERRING && robot.intakeTransfer.blockerReady && !robot.aim.readyToShoot) {
                val elapsed = robot.io.time() - (transferStartTime ?: robot.io.time())
                if (elapsed >= TRANSFER_DURATION) {
                    transferStartTime = null
                    robot.intakeTransfer.state = IntakeTransfer.State.READY_TO_SHOOT
                }
                // else: keep transferring until 3s elapsed
            } else if(cur != IntakeTransfer.State.TRANSFERRING && cur != IntakeTransfer.State.INTAKING) {
                // Not yet committed — pre-open blocker while waiting for readyToShoot
                robot.intakeTransfer.state = IntakeTransfer.State.READY_TO_SHOOT
            }
            // If already TRANSFERRING (!blockerReady) or user is INTAKING, don't override
        } else {
            transferStartTime = null
            robot.intakeTransfer.transferPowerOverride = null
            if ((robot.intakeTransfer.state == IntakeTransfer.State.READY_TO_SHOOT
                || robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING)
                && (autoShootEnabled || robot.aimFlywheel)) {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
            }
        }

        if(overrideShot) {
            robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
            transferStartTime = null
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
