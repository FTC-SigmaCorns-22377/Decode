package sigmacorns.logic

import org.joml.Vector2d
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.ballExitRadius
import sigmacorns.constants.flywheelRadius
import sigmacorns.constants.turretPos
import sigmacorns.control.aim.ApproachPrepositioner
import sigmacorns.control.aim.AutoAim
import sigmacorns.control.aim.Ballistics
import sigmacorns.control.aim.TurretPlannerBridge
import sigmacorns.control.aim.TurretPlannerBridge.OptimalTIdx
import sigmacorns.control.aim.TurretPlannerBridge.PrepositionIdx
import sigmacorns.control.aim.TurretPlannerBridge.RobustShotIdx
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
 * Uses the same GTSAM sensor fusion and VisionTracker as [AimingSystem], but
 * replaces the Kotlin Piyavskii-Shubert solver with the native [TurretPlannerBridge].
 * On the first frame (or after a large target change) a cold Piyavskii-Shubert search
 * is run; subsequent frames use a warm Newton refinement for low latency.
 *
 * Swap in [sigmacorns.Robot] by passing `useNativeAim = true`.
 */
class NativeAutoAim(
    private val robot: Robot,
    private val blue: Boolean,
) : AutoAim {

    // ------------------------------------------------------------------
    // Sensor fusion (same as AimingSystem)
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

    /** Approach-prepositioning helper, lazily created when [AimConfig.prepositionHorizon] > 0. */
    private var prepositioner: ApproachPrepositioner? = null

    /** Warm-start T* from the previous frame; null → cold start. */
    private var lastTStar: Float? = null

    /** Solver outputs from the last feasible frame. */
    private var lastTheta: Float = 0f
    private var lastPhi:   Float = 0f
    private var lastOmega: Float = 0f

    /** Second-shot leg for the most recent robust solve. */
    private var lastTheta2: Float = 0f
    private var lastPhi2:   Float = 0f
    private var lastOmega2: Float = 0f

    override var primaryShotState: Ballistics.ShotState? = null
        private set
    override var secondaryShotState: Ballistics.ShotState? = null
        private set
    override var isRobustActive: Boolean = false
        private set
    override var isPrepositionActive: Boolean = false
        private set

    // ------------------------------------------------------------------
    // Native config arrays (built once in init, constant thereafter)
    // ------------------------------------------------------------------

    private lateinit var physConfig:  FloatArray   // [g, rH]
    private lateinit var bounds:      FloatArray   // [thetaMin, thetaMax, phiMin, phiMax, vExitMax, omegaMax]
    private lateinit var weights:     FloatArray   // [wTheta, wPhi, wOmega]
    private lateinit var omegaCoeffs: FloatArray   // [c0..c5] for ω(φ,v) = c1·v  (linear in vExit)

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

        val vMax = AimConfig.vMax
        val omegaMax = vMax / (flywheelRadius * AimConfig.launchEfficiency)

        physConfig = floatArrayOf(
            AimConfig.g.toFloat(),
            ballExitRadius.toFloat()
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

        // omega(phi, vExit) = c1 * vExit  where c1 = 1 / (flywheelRadius * launchEfficiency)
        val c1 = (1.0 / (flywheelRadius * AimConfig.launchEfficiency)).toFloat()
        omegaCoeffs = floatArrayOf(0f, c1, 0f, 0f, 0f, 0f)

        prepositioner = if (AimConfig.prepositionHorizon > 0.0) {
            ApproachPrepositioner(bridge, weights, bounds, physConfig, omegaCoeffs)
        } else {
            null
        }
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

        // Robot velocity in field frame
        val odoVel = Vector2d(robot.io.velocity().v)
        val vel = odoVel.rotate(fusedPose.rot - robot.io.position().rot)

        // Turret position in field frame (ignore small robot-frame offset)
        val tX = fusedPose.v.x.toFloat()
        val tY = fusedPose.v.y.toFloat()
        val tZ = turretPos.z.toFloat()

        val goal = FieldLandmarks.goalPosition3d(blue, AimConfig.goalHeight)
        val gX = goal.x.toFloat()
        val gY = goal.y.toFloat()
        val gZ = goal.z.toFloat()

        val robotVx = vel.x.toFloat()
        val robotVy = vel.y.toFloat()

        // Current actuator state
        val curTheta = (turret.pos + fusedPose.rot).toFloat()
        val curPhi   = shooter.computedHoodAngle.toFloat()
        val curOmega = robot.io.flywheelVelocity().toFloat()

        val flywheelDrop = AimConfig.flywheelDrop.toFloat()

        // ── Launch-zone split ─────────────────────────────────────────────
        // Half-plane: nx*x + ny*y >= d means "inside the launch zone".
        // When both normal components are 0, the feature is disabled and we
        // fall through to the legacy instant / robustShot path unconditionally.
        val zoneNx = AimConfig.launchZoneNx
        val zoneNy = AimConfig.launchZoneNy
        val zoneD  = AimConfig.launchZoneD
        val zoneDefined = zoneNx != 0.0 || zoneNy != 0.0

        val zoneZ0 = if (zoneDefined) zoneNx * fusedPose.v.x + zoneNy * fusedPose.v.y - zoneD
                     else Double.POSITIVE_INFINITY
        val inZone = zoneZ0 >= 0.0

        var solved = false
        var robustSolved = false

        var prepositioned = false
        isRobustActive = false
        isPrepositionActive = false
        if (zoneDefined && !inZone) {
            // ── Outside the launch zone: preposition with t_until_zone ────
            // Half-plane approach time: z0 = n·p − d, z0(t) = z0 + (n·v)·t.
            // We want z0(t_cross) = 0, so t_cross = −z0 / (n·v), valid when
            // the robot is actually moving toward the zone (n·v > 0).
            val nDotV = zoneNx * vel.x + zoneNy * vel.y
            val tUntilZone = if (nDotV > 1e-3) {
                (-zoneZ0 / nDotV).coerceAtLeast(0.0)
            } else {
                // Not heading into the zone — give a generous slew budget so
                // the preposition doesn't over-clamp. This keeps the turret
                // prepared in case the driver reverses course.
                AimConfig.prepositionTAvailable
            }
            val prep = prepositioner?.compute(
                robotPose = fusedPose,
                robotVel = Vector2d(vel.x, vel.y),
                goal = goal,
                turretZ = turretPos.z,
                curTheta = curTheta.toDouble(),
                curPhi = curPhi.toDouble(),
                curOmega = curOmega.toDouble(),
                omegaDrop = AimConfig.flywheelDrop,
                horizonSeconds = AimConfig.prepositionHorizon,
                steps = AimConfig.prepositionSteps,
                lambdaDecay = AimConfig.prepositionLambda,
                tAvailable = tUntilZone,
            )
            if (prep != null) {
                lastTStar = null
                lastTheta = prep[PrepositionIdx.THETA]
                lastPhi   = prep[PrepositionIdx.PHI]
                lastOmega = prep[PrepositionIdx.OMEGA]
                prepositioned = true
                isPrepositionActive = true
                // `solved` stays false — preposition is a pre-aim, not a shot.
            }
        } else if (zoneDefined) {
            // ── Inside the launch zone: robustAdjust for fastest two-ball burst
            try {
                val r = bridge.robustAdjust(
                    tX, tY, tZ,
                    gX, gY, gZ,
                    gX, gY, gZ,
                    robotVx, robotVy,
                    curTheta, curPhi, curOmega,
                    flywheelDrop,
                    weights, bounds, physConfig, omegaCoeffs
                )
                if (r[RobustShotIdx.FEASIBLE] > 0.5f) {
                    lastTStar = r[RobustShotIdx.T1]
                    lastTheta = r[RobustShotIdx.S1_THETA]
                    lastPhi   = r[RobustShotIdx.S1_PHI]
                    lastOmega = r[RobustShotIdx.S1_OMEGA]
                    lastTheta2 = r[RobustShotIdx.S2_THETA]
                    lastPhi2   = r[RobustShotIdx.S2_PHI]
                    lastOmega2 = r[RobustShotIdx.S2_OMEGA]
                    solved = true
                    robustSolved = true
                }
            } catch (e: Exception) {
                // fall through — leaves last* untouched, readyToShoot=false below
            }
        } else {
            // ── No launch zone defined: legacy behavior ───────────────────
            val useRobust = flywheelDrop > 0f
            try {
                if (useRobust) {
                    val r = bridge.robustShot(
                        tX, tY, tZ,
                        gX, gY, gZ,
                        gX, gY, gZ,
                        robotVx, robotVy,
                        flywheelDrop,
                        weights, bounds, physConfig, omegaCoeffs
                    )
                    if (r[RobustShotIdx.FEASIBLE] > 0.5f) {
                        lastTStar = r[RobustShotIdx.T1]
                        lastTheta = r[RobustShotIdx.S1_THETA]
                        lastPhi   = r[RobustShotIdx.S1_PHI]
                        lastOmega = r[RobustShotIdx.S1_OMEGA]
                        lastTheta2 = r[RobustShotIdx.S2_THETA]
                        lastPhi2   = r[RobustShotIdx.S2_PHI]
                        lastOmega2 = r[RobustShotIdx.S2_OMEGA]
                        solved = true
                        robustSolved = true
                    }
                } else {
                    val tStar = lastTStar
                    val result = if (tStar != null) {
                        bridge.optimalTWarm(
                            tX, tY, tZ,
                            gX, gY, gZ,
                            robotVx, robotVy,
                            tStar,
                            curTheta, curPhi, curOmega,
                            weights, bounds, physConfig, omegaCoeffs
                        )
                    } else {
                        bridge.optimalTCold(
                            tX, tY, tZ,
                            gX, gY, gZ,
                            robotVx, robotVy,
                            curTheta, curPhi, curOmega,
                            weights, bounds, physConfig, omegaCoeffs
                        )
                    }
                    if (result[OptimalTIdx.FEASIBLE] > 0.5f) {
                        lastTStar = result[OptimalTIdx.T_STAR]
                        lastTheta = result[OptimalTIdx.THETA]
                        lastPhi   = result[OptimalTIdx.PHI]
                        lastOmega = result[OptimalTIdx.OMEGA]
                        solved = true
                    }
                }
            } catch (e: Exception) {
                // leaves last* untouched; readyToShoot set false below
            }
        }

        if (!solved && !prepositioned) {
            // No feasible solve this tick and no preposition update — bail out.
            lastTStar = null
            readyToShoot = false
            primaryShotState = null
            secondaryShotState = null
            return
        }

        // Expose the active target(s) as ShotState(theta, phi, vExit) for viz.
        primaryShotState = Ballistics.ShotState(
            theta = lastTheta.toDouble(),
            phi   = lastPhi.toDouble(),
            vExit = lastOmega.toDouble() * flywheelRadius * AimConfig.launchEfficiency,
        )
        if (robustSolved) {
            isRobustActive = true
            secondaryShotState = Ballistics.ShotState(
                theta = lastTheta2.toDouble(),
                phi   = lastPhi2.toDouble(),
                vExit = lastOmega2.toDouble() * flywheelRadius * AimConfig.launchEfficiency,
            )
        } else {
            secondaryShotState = null
        }

        if (aimTurret) {
            turret.fieldRelativeMode = true
            turret.fieldTargetAngle = lastTheta.toDouble()
        }

        if (robot.aimFlywheel) {
            shooter.hoodAngle = lastPhi.toDouble()
            shooter.flywheelTarget = lastOmega.toDouble()
        }

        // Readiness only counts when we have a real shot solve — a preposition
        // target is a pre-aim, not an alignment to an actual feasible shot.
        if (solved) {
            val thetaErr = abs(wrapAngle(turret.pos + fusedPose.rot - lastTheta))
            val phiErr   = abs(shooter.computedHoodAngle - lastPhi)
            val omegaErr = abs(robot.io.flywheelVelocity() - lastOmega)
            readyToShoot = thetaErr < turretTol && phiErr < hoodTol && omegaErr < omegaTol
        } else {
            readyToShoot = false
        }

        if (shotRequested && readyToShoot) {
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
