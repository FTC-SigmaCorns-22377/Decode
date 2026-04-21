package sigmacorns.logic

import org.joml.Vector2d
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.Drivetrain
import sigmacorns.subsystem.IntakeTransfer
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Full-cycle auto-chase state machine:
 *
 *   CHASE → intake running, blocker engaged, drive straight at the nearest
 *   confirmed ball track at full speed until beam-breaks report 3 balls held
 *   (or the caller-supplied override condition).
 *
 *   DRIVE_TO_SHOOT_ZONE → drive straight to `shootingZone` at full speed.
 *   Intake stops, flywheel spins up in parallel so it's ready on arrival.
 *
 *   SHOOT → halt, open blocker, run transfer forward, flywheel at shoot speed,
 *   wait until the held-ball count drops to zero. Then back to CHASE.
 *
 * Designed to be reusable between the sim viz harness and the hardware
 * `BallChaseTeleOp`. The FSM only touches the [Drivetrain] and the IO's
 * intake / blocker / flywheel fields — it does not reach into the full
 * [sigmacorns.Robot] subsystem graph, so it's safe to instantiate anywhere a
 * SigmaIO + Drivetrain are available.
 *
 * Tune:
 *   - `maxSpeed` — fraction of mecanum max, 0..1. The chase is "instantaneous":
 *     full throttle until contact.
 *   - `shootingZone` — field-frame (x, y, heading) to drive to before shooting.
 *     Heading is NOT enforced by the current mecanum controller in this class
 *     (translation only); rotate externally if your goal alignment needs it.
 *   - `arrivalRadiusM` — distance at which DRIVE_TO_SHOOT_ZONE transitions
 *     to SHOOT. Keep small (15-20 cm) to actually settle.
 *   - `chasePullInRadiusM` — below this, stop yawing and commit to a
 *     straight-line intake approach (prevents over-correction right before
 *     the ball enters the intake).
 *   - `flywheelShootSpeed` — flywheel target (units match SigmaIO.flywheel,
 *     typically 0..1 motor power). TODO: accept a rad/s target if a shot
 *     solver is wired in downstream.
 *   - `heldBallCountFn` — returns the current number of held balls. Default
 *     reads the three beam-break sensors. Sim callers that want to cheat the
 *     count (e.g. JoltSimIO.heldBalls.size) can pass their own.
 */
class BallChaseAutoFSM(
    private val tracking: BallTrackingSystem,
    private val drivetrain: Drivetrain,
    private val io: SigmaIO,
    private val shootingZone: Pose2d,
    private val maxSpeed: Double = 1.0,
    private val arrivalRadiusM: Double = 0.18,
    private val chasePullInRadiusM: Double = 0.30,
    private val flywheelShootSpeed: Double = 1.0,
    private val headingP: Double = 2.0,
    private val heldBallCountFn: () -> Int = { countHeldFromBeamBreaks(io) },
) {
    enum class Phase { IDLE, CHASE, DRIVE_TO_SHOOT_ZONE, SHOOT }

    var phase: Phase = Phase.IDLE
        private set

    /** True while commanding; set to false to stop the machine and zero outputs. */
    var enabled: Boolean = false

    /** Reports which ball the robot currently thinks it's chasing (field pose). */
    val currentTarget: Pose2d? get() =
        if (phase == Phase.CHASE) tracking.targetBallField else null

    fun update() {
        if (!enabled) {
            idleAll()
            phase = Phase.IDLE
            return
        }

        // First activation -> jump straight into CHASE. Instantaneous by design.
        if (phase == Phase.IDLE) phase = Phase.CHASE

        when (phase) {
            Phase.IDLE -> idleAll()
            Phase.CHASE -> {
                val heldCount = heldBallCountFn()
                if (heldCount >= 3) {
                    phase = Phase.DRIVE_TO_SHOOT_ZONE
                    // Stop intake; let flywheel begin spooling up.
                    io.intake = 0.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                    io.flywheel = flywheelShootSpeed
                    return
                }

                // Intake running with blocker engaged — collect balls into the
                // transfer path but don't feed them to the flywheel yet.
                io.intake = 1.0
                io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                io.flywheel = 0.0

                val target = tracking.targetBallField
                if (target == null) {
                    drivetrain.drive(Pose2d(0.0, 0.0, 0.0), io)
                    return
                }
                driveTowardField(target.v, yawToTargetAllowed = true)
            }
            Phase.DRIVE_TO_SHOOT_ZONE -> {
                io.intake = 0.0
                io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                io.flywheel = flywheelShootSpeed

                val pose = io.position()
                val dx = shootingZone.v.x - pose.v.x
                val dy = shootingZone.v.y - pose.v.y
                val d = hypot(dx, dy)
                if (d < arrivalRadiusM) {
                    phase = Phase.SHOOT
                    return
                }
                driveTowardField(shootingZone.v, yawToTargetAllowed = false, desiredHeading = shootingZone.rot)
            }
            Phase.SHOOT -> {
                // Halt, flywheel max, blocker open, transfer running forward.
                drivetrain.drive(Pose2d(0.0, 0.0, 0.0), io)
                io.flywheel = flywheelShootSpeed
                io.blocker = IntakeTransfer.BLOCKER_DISENGAGED
                io.intake = 1.0

                if (heldBallCountFn() <= 0) {
                    phase = Phase.CHASE
                    io.flywheel = 0.0
                    io.blocker = IntakeTransfer.BLOCKER_ENGAGED
                }
            }
        }
    }

    private fun driveTowardField(
        goalField: Vector2d,
        yawToTargetAllowed: Boolean,
        desiredHeading: Double? = null,
    ) {
        val pose = io.position()
        val dx = goalField.x - pose.v.x
        val dy = goalField.y - pose.v.y
        val d = hypot(dx, dy)
        if (d < 1e-6) {
            drivetrain.drive(Pose2d(0.0, 0.0, 0.0), io)
            return
        }

        val cosH = cos(pose.rot)
        val sinH = sin(pose.rot)
        val dirX = dx / d
        val dirY = dy / d
        val bodyX =  cosH * dirX + sinH * dirY
        val bodyY = -sinH * dirX + cosH * dirY

        val headingTarget = desiredHeading ?: if (yawToTargetAllowed) atan2(dy, dx) else pose.rot
        val yaw = if (yawToTargetAllowed || desiredHeading != null) {
            if (d < chasePullInRadiusM && yawToTargetAllowed) {
                0.0  // stop yawing right before intake so we don't swing the mouth off the ball
            } else {
                var err = headingTarget - pose.rot
                while (err >  Math.PI) err -= 2.0 * Math.PI
                while (err < -Math.PI) err += 2.0 * Math.PI
                (headingP * err).coerceIn(-maxSpeed, maxSpeed)
            }
        } else 0.0

        drivetrain.drive(
            Pose2d(maxSpeed * bodyX, maxSpeed * bodyY, yaw),
            io,
        )
    }

    private fun idleAll() {
        drivetrain.drive(Pose2d(0.0, 0.0, 0.0), io)
        io.intake = 0.0
        io.blocker = IntakeTransfer.BLOCKER_ENGAGED
        io.flywheel = 0.0
    }

    companion object {
        fun countHeldFromBeamBreaks(io: SigmaIO): Int {
            var n = 0
            if (io.beamBreak1()) n++
            if (io.beamBreak2()) n++
            if (io.beamBreak3()) n++
            return n
        }
    }
}
