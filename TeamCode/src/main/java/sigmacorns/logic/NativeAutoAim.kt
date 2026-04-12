package sigmacorns.logic

import org.joml.Vector2d
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.ballExitRadius
import sigmacorns.constants.flywheelMotor
import sigmacorns.constants.flywheelRadius
import sigmacorns.constants.turretPos
import sigmacorns.control.aim.AutoAim
import sigmacorns.control.aim.Ballistics
import sigmacorns.control.aim.TurretPlannerBridge
import sigmacorns.control.aim.TurretPlannerBridge.Robust3ShotPlanIdx
import sigmacorns.control.localization.GTSAMEstimator
import sigmacorns.control.localization.VisionTracker
import sigmacorns.io.HardwareIO
import sigmacorns.io.rotate
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.time.Duration

/**
 * Auto-aim implementation backed by the C++ turret_planner library (JNI).
 *
 * Uses the trajectory-aware `robust3ShotPlan` solver to plan up to 3
 * consecutive shots. During a burst, tracks which shot is next and
 * switches flywheel/hood/turret targets immediately after each ball exits.
 */
class NativeAutoAim(
    private val robot: Robot,
    private val blue: Boolean,
) : AutoAim {

    // ------------------------------------------------------------------
    // Sensor fusion
    // ------------------------------------------------------------------

    override lateinit var autoAim: GTSAMEstimator
        private set
    private var visionTracker: VisionTracker? = null

    // ------------------------------------------------------------------
    // AutoAim state
    // ------------------------------------------------------------------

    override var goalPosition: Vector2d = FieldLandmarks.goalPosition(blue)
    override var positionOverride: Double? = null
    override var shotRequested: Boolean = false
    override var readyToShoot: Boolean = false
        private set
    override var targetDistance: Double = 3.0
        private set

    // ------------------------------------------------------------------
    // Native bridge
    // ------------------------------------------------------------------

    private val bridge = TurretPlannerBridge()

    /** Solver outputs from the last feasible frame (the NEXT shot to fire). */
    private var lastTheta: Float = 0f
    private var lastPhi:   Float = 0f
    private var lastOmega: Float = 0f

    override var primaryShotState: Ballistics.ShotState? = null
        private set
    override var secondaryShotState: Ballistics.ShotState? = null
        private set
    override var tertiaryShotState: Ballistics.ShotState? = null
        private set
    override var isRobustActive: Boolean = false
        private set
    override var isPrepositionActive: Boolean = false
        private set

    // ------------------------------------------------------------------
    // Burst sequencing state
    // ------------------------------------------------------------------

    /** True while we're firing a multi-shot burst. */
    private var burstActive: Boolean = false

    /** Ball count when the burst started. Ball exits are detected by comparing current count. */
    private var burstStartBallCount: Int = 0

    /** How many balls have exited so far in this burst. */
    private var burstShotsFired: Int = 0

    /** Total shots intended in this burst. */
    private var burstTotalShots: Int = 0

    /** Timestamp of the most recent ball exit. Null before the first ball fires. */
    private var lastBallExitTime: Duration? = null

    // ------------------------------------------------------------------
    // Native config arrays
    // ------------------------------------------------------------------

    private lateinit var physConfig:  FloatArray
    private lateinit var bounds:      FloatArray
    private lateinit var weights:     FloatArray
    private lateinit var omegaCoeffs: FloatArray

    // ------------------------------------------------------------------
    // Tolerances for readyToShoot
    // ------------------------------------------------------------------

    private val turretTol = 0.05   // rad
    private val hoodTol   = 0.05   // rad
    private val omegaTol  = 50.0   // rad/s

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    override fun init(initialPose: Pose2d, apriltagTracking: Boolean) {
        autoAim = GTSAMEstimator(
            landmarkPositions = FieldLandmarks.landmarks,
            initialPose = initialPose,
        )

        val limelight = (robot.io as? HardwareIO).takeIf { apriltagTracking }?.limelight
        visionTracker = VisionTracker(
            limelight = limelight,
            allowedTagIds = FieldLandmarks.landmarkTagIds
        )

        autoAim.enabled = true

        val omegaMax = flywheelMotor.freeSpeed
        val vMax = omegaMax * flywheelRadius * AimConfig.launchEfficiency

        physConfig = floatArrayOf(
            AimConfig.g.toFloat(),
            ballExitRadius.toFloat(),
            AimConfig.dragK.toFloat()
        )

        bounds = floatArrayOf(
            (-PI).toFloat(),
            PI.toFloat(),
            Math.toRadians(ShooterConfig.minAngleDeg).toFloat(),
            Math.toRadians(ShooterConfig.maxAngleDeg).toFloat(),
            vMax.toFloat(),
            omegaMax.toFloat()
        )

        weights = floatArrayOf(
            ShotSolverConfig.wTheta.toFloat(),
            ShotSolverConfig.wPhi.toFloat(),
            ShotSolverConfig.wOmega.toFloat()
        )

        val c1 = (1.0 / (flywheelRadius * AimConfig.launchEfficiency)).toFloat()
        omegaCoeffs = floatArrayOf(0f, c1, 0f, 0f, 0f, 0f)

    }

    override fun update(dt: Duration, aimTurret: Boolean) {
        updateVision()
        setTurretInputs()
        setShooterInputs(aimTurret)

        positionOverride?.let { override ->
            if (robot.turret.fieldRelativeMode) robot.turret.fieldTargetAngle = override
            else robot.turret.targetAngle = override
        }
    }

    override fun close() {
        autoAim.close()
    }

    // ------------------------------------------------------------------
    // Internal pipeline
    // ------------------------------------------------------------------

    private fun updateVision() {
        val visionResult = visionTracker?.read()
        autoAim.update(robot.io.position(), robot.io.velocity(), robot.io.turretPosition(), visionResult)

        val fusedPose = autoAim.fusedPose
        targetDistance = hypot(goalPosition.x - fusedPose.v.x, goalPosition.y - fusedPose.v.y)
    }

    private fun setTurretInputs() {
        robot.turret.robotHeading = autoAim.fusedPose.rot
        robot.turret.robotAngularVelocity = robot.io.velocity().rot
    }

    private fun setShooterInputs(aimTurret: Boolean) {
        val shooter = robot.shooter
        val turret  = robot.turret
        val fusedPose = autoAim.fusedPose
        val now = robot.io.time()

        val nBalls = robot.beamBreak.ballCount

        // ── Burst sequencing: detect ball exits ──────────────────────────
        if (burstActive) {
            val ballsFiredSoFar = burstStartBallCount - nBalls
            if (ballsFiredSoFar > burstShotsFired) {
                // A ball just exited
                burstShotsFired = ballsFiredSoFar
                lastBallExitTime = now
                // Immediately start transferring the next ball if any remain
                if (burstShotsFired < burstTotalShots && nBalls > 0) {
                    robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
                }
            }
            // Burst complete
            if (burstShotsFired >= burstTotalShots || nBalls == 0) {
                burstActive = false
                burstShotsFired = 0
                lastBallExitTime = null
            }
        }

        // ── Compute t_remaining ──────────────────────────────────────────
        // Time until the next ball can physically exit the shooter.
        //   Not in burst: transferDelay (ball needs full transfer to reach flywheel)
        //   In burst, first ball not yet fired: transferDelay (same)
        //   In burst, after a ball exited: transferDelay - elapsed since exit
        val tRemaining: Float = if (burstActive && lastBallExitTime != null) {
            val elapsed = (now - lastBallExitTime!!).inWholeMilliseconds / 1000.0
            (AimConfig.transferDelay - elapsed).coerceAtLeast(0.0).toFloat()
        } else {
            AimConfig.transferDelay.toFloat()
        }

        // How many balls remain to plan for
        val remainingBalls = if (burstActive) {
            (burstTotalShots - burstShotsFired).coerceIn(1, nBalls)
        } else {
            nBalls.coerceAtLeast(1)
        }

        // Robot velocity in field frame
        val odoVel = Vector2d(robot.io.velocity().v)
        val vel = odoVel.rotate(fusedPose.rot - robot.io.position().rot)

        val tX = fusedPose.v.x.toFloat()
        val tY = fusedPose.v.y.toFloat()
        val tZ = turretPos.z.toFloat()

        val goal = FieldLandmarks.goalPosition3d(blue, AimConfig.goalHeight)
        val gX = goal.x.toFloat()
        val gY = goal.y.toFloat()
        val gZ = goal.z.toFloat()

        val robotVx = vel.x.toFloat()
        val robotVy = vel.y.toFloat()

        val curTheta = (turret.pos + fusedPose.rot).toFloat()
        val curPhi   = shooter.computedHoodAngle.toFloat()
        val curOmega = robot.io.flywheelVelocity().toFloat()

        // ── Launch-zone: compute effective t_remaining ────────────────────
        // When outside a defined zone, inflate t_remaining so urgency drops
        // to ~0 and the solver optimizes purely for inter-shot robustness.
        // The turret slews toward s1 naturally over the approach time.
        val zoneNx = AimConfig.launchZoneNx
        val zoneNy = AimConfig.launchZoneNy
        val zoneD  = AimConfig.launchZoneD
        val zoneDefined = zoneNx != 0.0 || zoneNy != 0.0

        val zoneZ0 = if (zoneDefined) zoneNx * fusedPose.v.x + zoneNy * fusedPose.v.y - zoneD
                     else Double.POSITIVE_INFINITY
        val inZone = zoneZ0 >= 0.0

        val effectiveTRemaining: Float = if (zoneDefined && !inZone && !burstActive) {
            // Outside the zone: estimate time until zone entry, add transfer delay.
            // With high t_remaining, w_urgency → 0 so J_0 doesn't matter.
            val nDotV = zoneNx * vel.x + zoneNy * vel.y
            val tUntilZone = if (nDotV > 1e-3) (-zoneZ0 / nDotV).coerceAtLeast(0.0)
                             else AimConfig.prepositionTAvailable
            (tUntilZone + AimConfig.transferDelay).toFloat()
        } else {
            tRemaining
        }

        var solved = false
        isRobustActive = false
        isPrepositionActive = false
        secondaryShotState = null
        tertiaryShotState = null

        // ── Trajectory-aware robust solve (used everywhere) ──────────────
        try {
            val horizon = AimConfig.transferDelay * remainingBalls + AimConfig.predictionHorizon
            val step = AimConfig.predictionStep.toFloat()
            val nSteps = (horizon / step).toInt().coerceIn(2, 60)
            val trajArray = FloatArray(nSteps * 6)
            for (i in 0 until nSteps) {
                val t = i * step
                trajArray[i * 6 + 0] = t
                trajArray[i * 6 + 1] = tX + robotVx * t
                trajArray[i * 6 + 2] = tY + robotVy * t
                trajArray[i * 6 + 3] = fusedPose.rot.toFloat()
                trajArray[i * 6 + 4] = robotVx
                trajArray[i * 6 + 5] = robotVy
            }

            val r = bridge.robust3ShotPlan(
                trajArray, nSteps,
                tZ,
                gX, gY, gZ,
                curTheta, curPhi, curOmega,
                remainingBalls,
                effectiveTRemaining,
                AimConfig.transferDelay.toFloat(),
                AimConfig.dropFraction.toFloat(),
                AimConfig.urgencyLambda.toFloat(),
                weights, bounds, physConfig, omegaCoeffs
            )

            if (r[Robust3ShotPlanIdx.FEASIBLE] > 0.5f) {
                // The solver found optimal flight times from future trajectory
                // positions. Re-solve s1 from the CURRENT position using T1 so
                // the turret theta is correct for right now (not for the future
                // position). Phi and omega are nearly the same; theta changes
                // significantly because it depends on the turret-to-goal angle.
                val T1 = r[Robust3ShotPlanIdx.T1]
                val nowShot = bridge.solve(
                    tX, tY, tZ,
                    gX, gY, gZ,
                    robotVx, robotVy,
                    T1,
                    physConfig, omegaCoeffs
                )
                lastTheta = nowShot[TurretPlannerBridge.SolveIdx.THETA]
                lastPhi   = nowShot[TurretPlannerBridge.SolveIdx.PHI]
                lastOmega = nowShot[TurretPlannerBridge.SolveIdx.OMEGA]
                solved = true

                if (remainingBalls >= 2) {
                    isRobustActive = true
                    secondaryShotState = Ballistics.ShotState(
                        theta = r[Robust3ShotPlanIdx.S2_THETA].toDouble(),
                        phi   = r[Robust3ShotPlanIdx.S2_PHI].toDouble(),
                        vExit = r[Robust3ShotPlanIdx.S2_OMEGA].toDouble() * flywheelRadius * AimConfig.launchEfficiency,
                    )
                }
                if (remainingBalls >= 3) {
                    tertiaryShotState = Ballistics.ShotState(
                        theta = r[Robust3ShotPlanIdx.S3_THETA].toDouble(),
                        phi   = r[Robust3ShotPlanIdx.S3_PHI].toDouble(),
                        vExit = r[Robust3ShotPlanIdx.S3_OMEGA].toDouble() * flywheelRadius * AimConfig.launchEfficiency,
                    )
                }
            }
        } catch (_: Exception) {
            // leaves last* untouched; readyToShoot set false below
        }

        if (!solved) {
            readyToShoot = false
            primaryShotState = null
            secondaryShotState = null
            tertiaryShotState = null
            return
        }

        primaryShotState = Ballistics.ShotState(
            theta = lastTheta.toDouble(),
            phi   = lastPhi.toDouble(),
            vExit = lastOmega.toDouble() * flywheelRadius * AimConfig.launchEfficiency,
        )

        if (aimTurret) {
            turret.fieldRelativeMode = true
            turret.fieldTargetAngle = lastTheta.toDouble()
        }

        if (robot.aimFlywheel) {
            shooter.hoodAngle = lastPhi.toDouble()
            shooter.flywheelTarget = lastOmega.toDouble()
        }

        if (solved) {
            val thetaErr = abs(wrapAngle(turret.pos + fusedPose.rot - lastTheta))
            val phiErr   = abs(shooter.computedHoodAngle - lastPhi)
            val omegaErr = abs(robot.io.flywheelVelocity() - lastOmega)
            readyToShoot = thetaErr < turretTol && phiErr < hoodTol && omegaErr < omegaTol
        } else {
            readyToShoot = false
        }

        // ── Fire / burst trigger ─────────────────────────────────────────
        if (shotRequested && readyToShoot) {
            if (!burstActive) {
                burstActive = true
                burstShotsFired = 0
                burstStartBallCount = nBalls
                burstTotalShots = nBalls
                lastBallExitTime = null
            }
            shotRequested = false
            robot.intakeTransfer.state = IntakeTransfer.State.TRANSFERRING
        }
    }

    private fun wrapAngle(a: Double): Double {
        var r = a % (2 * PI)
        if (r > PI)  r -= 2 * PI
        if (r < -PI) r += 2 * PI
        return r
    }
}
