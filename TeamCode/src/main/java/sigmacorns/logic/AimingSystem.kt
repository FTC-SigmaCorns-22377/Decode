package sigmacorns.logic

import org.joml.Vector2d
import org.joml.Vector3d
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.ballExitRadius
import sigmacorns.constants.flywheelMotor
import sigmacorns.constants.flywheelRadius
import sigmacorns.constants.turretPos
import sigmacorns.control.aim.Ballistics
import sigmacorns.control.aim.OmegaMap
import sigmacorns.control.aim.ShotSolver
import sigmacorns.control.localization.GTSAMEstimator
import sigmacorns.control.localization.VisionTracker
import sigmacorns.io.HardwareIO
import sigmacorns.io.rotate
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

/**
 * Aiming system: vision + sensor fusion + turret targeting + shooter coordination.
 *
 * Owns the full aiming pipeline from vision input to turret/shooter output:
 * - Vision tracking and GTSAM sensor fusion
 * - Auto-aim turret targeting via ShotSolver (ballistic trajectory optimization)
 * - Directly sets turret [fieldTargetAngle], shooter [hoodAngle], and [flywheelTarget]
 *   from the solver's optimal [Ballistics.ShotState]
 * - Robot motion compensation is inherent: robot velocity is passed to [Ballistics.Target]
 *
 * Opmodes can disable turret targeting by setting robot.aimTurret = false,
 * and disable flywheel/hood targeting via robot.aimFlywheel = false.
 * Manual hood override uses shooter.autoAdjust = false + shooter.manualHoodAngle.
 */
class AimingSystem(
    private val robot: Robot,
    private val blue: Boolean,
) {
    val turret get() = robot.turret
    lateinit var autoAim: GTSAMEstimator
        private set
    var visionTracker: VisionTracker? = null
        private set

    private lateinit var ballistics: Ballistics
    private lateinit var shotSolver: ShotSolver

    var goalPosition: Vector2d = FieldLandmarks.goalPosition(blue)

    var positionOverride: Double? = null

    /** Distance from fused pose to goal, updated each [updateVision] call. */
    var targetDistance: Double = 3.0
        private set

    /**
     * Set to true to arm a shot. AimingSystem will fire (TRANSFERRING) as soon as all
     * actuators are within tolerance of the solver target, or after 2 seconds. Cleared automatically after firing.
     */
    var shotRequested: Boolean = false

    private var shotRequestedTime: Duration? = null

    /**
     * True when turret, hood, and flywheel are all within tolerance of the current solver target.
     * Updated every loop; readable by opmodes for telemetry.
     */
    var readyToShoot: Boolean = false
        private set

    private var lastShotState: Ballistics.ShotState? = null

    /**
     * Initialize all subsystems. Call once before the main loop.
     */
    fun init(initialPose: Pose2d, apriltagTracking: Boolean) {
        autoAim = GTSAMEstimator(
            landmarkPositions = FieldLandmarks.landmarks,
            initialPose = initialPose,
        )

        val limelight = (robot.io as? HardwareIO).takeIf { apriltagTracking }?.limelight
        visionTracker = VisionTracker(
            limelight = limelight,
            allowedTagIds = FieldLandmarks.landmarkTagIds
        )

        ballistics = Ballistics(
            rH = ballExitRadius,
            vMax = AimConfig.vMax,
            phiMin = Math.toRadians(ShooterConfig.minAngleDeg),
            phiMax = Math.toRadians(ShooterConfig.maxAngleDeg),
        )


        shotSolver = ShotSolver(
            omega = AimConfig.omegaMap,
            wOmega = ShotSolverConfig.wOmega,
            wTheta = ShotSolverConfig.wTheta,
            wPhi = ShotSolverConfig.wPhi,
            ballistics = ballistics,
        )

        autoAim.enabled = true
    }

    /**
     * Read vision, update sensor fusion, and recompute target distance.
     * Call at the start of each loop iteration.
     */
    fun updateVision() {
        val visionResult = visionTracker?.read()
        autoAim.update(robot.io.position(), robot.io.velocity(), robot.io.turretPosition(), visionResult)

        val fusedPose = autoAim.fusedPose
        targetDistance = hypot(goalPosition.x - fusedPose.v.x, goalPosition.y - fusedPose.v.y)
    }

    /**
     * Set turret heading and angular velocity from fused pose.
     * Does NOT call turret.update() — that is done by Robot.update().
     */
    fun setTurretInputs(dt: Duration) {
        robot.turret.robotHeading = autoAim.fusedPose.rot
        robot.turret.robotAngularVelocity = robot.io.velocity().rot
    }

    /**
     * Solve for the optimal shot and directly set turret, hood, and flywheel targets.
     *
     * Builds a [Ballistics.Target] from the current fused pose and robot velocity,
     * then calls [ShotSolver.optimalAdjust] to find the flight time that minimizes
     * actuator movement. Sets subsystem targets from the resulting [Ballistics.ShotState].
     *
     * Robot velocity enters via [Ballistics.Target.vR], providing inherent shoot-while-move
     * compensation without a separate lead-angle calculation.
     *
     * @param aimTurret if true, overrides turret field target from solver output
     */
    fun setShooterInputs(aimTurret: Boolean) {
        val shooter = robot.shooter
        val fusedPose = autoAim.fusedPose
        val odoVel = Vector2d(robot.io.velocity().v)

        val vel = odoVel.rotate(fusedPose.rot - robot.io.position().rot)

        // Offset goal position by velocity * transfer delay to account for robot motion during shot
        val goalPos = FieldLandmarks.goalPosition3d(blue, AimConfig.goalHeight)
        val offsetGoalPos = Vector3d(
            goalPos.x + vel.x * AimConfig.transferDelay,
            goalPos.y + vel.y * AimConfig.transferDelay,
            goalPos.z
        )

        val target = Ballistics.Target(
            target = offsetGoalPos,
            turret = Vector3d(fusedPose.v.x, fusedPose.v.y, turretPos.z),
            vR = Vector2d(vel.x, vel.y),
        )

        val currentVexit = AimConfig.omegaInv(robot.io.flywheelVelocity(),robot.shooter.hoodAngle)
        val currentState = Ballistics.ShotState(
            theta = turret.pos + fusedPose.rot,
            phi = shooter.computedHoodAngle,
            vExit = currentVexit,
        )

        val curShotErr = ballistics.shotError(currentState,target)

        if (shotRequested) {
            if (shotRequestedTime == null) {
                shotRequestedTime = robot.io.time()
            }

            val elapsed = robot.io.time() - shotRequestedTime!!
            val shouldFire = curShotErr < AimConfig.shotTolerance || elapsed >= 2.seconds

            if (shouldFire) {
                shotRequested = false
                shotRequestedTime = null
                robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
            }
        }

        val isFallback = try {
            val optimal = shotSolver.optimalAdjust(currentState, target, ShotSolverConfig.tolerance)
            lastShotState = optimal
            false
        } catch (e: Exception) {
            // No optimal solution found; fix vExit to vMax and find the hood angle closest to goal
            var bestShot: Ballistics.ShotState? = null
            var bestError = Double.POSITIVE_INFINITY

            val phiSteps = 20
            for (i in 0..phiSteps) {
                val phi = ShooterConfig.minAngleDeg + (ShooterConfig.maxAngleDeg - ShooterConfig.minAngleDeg) * i / phiSteps
                val phiRad = Math.toRadians(phi)

                // Use current turret position for theta
                val candidate = Ballistics.ShotState(
                    theta = turret.pos + autoAim.fusedPose.rot,
                    phi = phiRad,
                    vExit = AimConfig.vMax
                )

                val error = ballistics.shotError(candidate, target)
                if (error < bestError) {
                    bestError = error
                    bestShot = candidate
                }
            }

            lastShotState = bestShot ?: ballistics.solve(target, 1.0)
            true
        }

        val shotState = lastShotState ?: return

        if (aimTurret) {
            turret.fieldRelativeMode = true
            turret.fieldTargetAngle = shotState.theta
        }

        if (robot.aimFlywheel) {
            shooter.hoodAngle = shotState.phi
            shooter.flywheelTarget = AimConfig.omegaMap.omega(shotState.phi, shotState.vExit)
        }

        // Only consider ready to shoot if we're using the optimal solution
        readyToShoot = !isFallback && curShotErr < AimConfig.shotTolerance
    }

    /**
     * Convenience: full pipeline update (vision -> turret inputs -> solver targets).
     * positionOverride is applied last and always wins over solver output.
     */
    fun update(dt: Duration, aimTurret: Boolean = true) {
        updateVision()
        setTurretInputs(dt)
        setShooterInputs(aimTurret)

        positionOverride?.let {
            if (turret.fieldRelativeMode) turret.fieldTargetAngle = it
            else turret.targetAngle = it
        }
    }

    fun close() {
        autoAim.close()
    }
}

object AimConfig {
    @JvmField var goalHeight = 1.14

    /** Time (seconds) from when shot is requested until ball leaves shooter */
    @JvmField var transferDelay = 0.2

    @JvmField var launchEfficiency = 0.20
    val omegaMap = object : OmegaMap {
        override fun omega(hood: Double, vExit: Double) =
            vExit / (flywheelRadius * launchEfficiency)

        override fun lipschitzBound(
            hood: ClosedRange<Double>, vExit: ClosedRange<Double>,
            lHood: Double, lvExit: Double
        ) = lvExit / (flywheelRadius * launchEfficiency)
    }

    val omegaInv = { omega: Double, hood: Double -> omega*flywheelRadius*launchEfficiency }

    @JvmField var vMax = flywheelMotor.freeSpeed * launchEfficiency * flywheelRadius

    // shots area allowed when the ball will pass < shotTolerance distance from the target when the ball is at the same height as the target
    @JvmField var shotTolerance = 0.05 // m
}

object ShotSolverConfig {
    @JvmField var wOmega = 0.01   // s per rad/s flywheel change
    @JvmField var wTheta = 0.1    // s per rad turret change
    @JvmField var wPhi = 0.1      // s per rad hood change
    @JvmField var tolerance = 0.05
}
