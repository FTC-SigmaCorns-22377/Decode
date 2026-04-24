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
import sigmacorns.control.aim.tune.BisectionSample
import sigmacorns.control.aim.tune.BisectionStore
import sigmacorns.control.aim.tune.OmegaCoefFitter
import sigmacorns.control.localization.GTSAMEstimator
import sigmacorns.control.localization.VisionTracker
import sigmacorns.io.HardwareIO
import sigmacorns.io.rotate
import sigmacorns.math.Pose2d
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import sigmacorns.subsystem.TurretServoConfig
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.time.Duration

/**
 * Auto-aim backed by the C++ turret_planner (JNI).
 *
 * Solves from the current robot state each loop using robust3ShotPlan with a
 * single-state trajectory. The solver always returns a feasible plan (no hard
 * inter-shot constraints); J_12 and J_23 are returned so [transferPowerCommand]
 * can be set to slow the transfer motor if the flywheel needs more recovery time.
 *
 * During burst (transfer motor running), switches to 1-ball solve so each ball
 * is aimed independently. Transfer is paused by IntakeCoordinator whenever
 * [readyToShoot] is false.
 */
class NativeAutoAim(
    private val robot: Robot,
    private val blue: Boolean,
) : AutoAim {

    override lateinit var autoAim: GTSAMEstimator
        private set
    private var visionTracker: VisionTracker? = null

    override var goalPosition: Vector2d = FieldLandmarks.goalPosition(blue)
    override var positionOverride: Double? = null
    override var shotRequested: Boolean = false
    override var readyToShoot: Boolean = false
        private set
    override var targetDistance: Double = 3.0
        private set

    private val bridge = TurretPlannerBridge()

    private var lastTheta: Float = 0f
    private var lastPhi:   Float = 0f
    private var lastOmega: Float = 0f
    private var lastT1:    Float = 0f

    // Diagnostic fields for telemetry
    var lastSolveException: String = "none"
        private set
    var lastSolveFeasible: Float = -1f
        private set
    var omegaDataPoints: Int = 0
        private set
    var diagPhi05: Float = Float.NaN       // phi from direct solve at T=0.5s
        private set
    var diagVexit05: Float = Float.NaN     // v_exit from direct solve at T=0.5s
        private set
    var diagGoalDist: Float = Float.NaN
        private set
    var diagColdFeasible: Float = -1f      // feasible flag from optimalTCold directly
        private set
    var diagColdPhi: Float = Float.NaN     // phi at T* from optimalTCold
        private set
    var diagColdTstar: Float = Float.NaN   // T* from optimalTCold
        private set

    override var primaryShotState: Ballistics.ShotState? = null
        private set
    override var secondaryShotState: Ballistics.ShotState? = null
        private set
    override var tertiaryShotState: Ballistics.ShotState? = null
        private set
    override var isRobustActive: Boolean = false
        private set

    /** Transfer motor power [0.1, 1.0] derived from solver J_12/J_23 each loop. */
    override var transferPowerCommand: Float = 1.0f
        private set

    // ── Zones (mirrored for alliance) ────────────────────────────────
    private lateinit var goalZone: List<Vector2d>
    private lateinit var farZone: List<Vector2d>

    // ── Native config arrays ─────────────────────────────────────────
    private lateinit var physConfig:  FloatArray
    private lateinit var bounds:      FloatArray
    private lateinit var weights:     FloatArray
    private lateinit var omegaCoeffs: FloatArray

    // ── Lifecycle ────────────────────────────────────────────────────

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
            (-PI).toFloat(), PI.toFloat(),
            Math.toRadians(ShooterConfig.minAngleDeg).toFloat(),
            Math.toRadians(ShooterConfig.maxAngleDeg).toFloat(),
            vMax.toFloat(), omegaMax.toFloat()
        )
        weights = floatArrayOf(
            ShotSolverConfig.wTheta.toFloat(),
            ShotSolverConfig.wPhi.toFloat(),
            ShotSolverConfig.wOmega.toFloat(),
            AimConfig.interShotAlpha.toFloat()
        )

        val tuningStore = BisectionStore().also { it.load() }
        val makes = tuningStore.madeSamples()
        omegaDataPoints = makes.size
        omegaCoeffs = buildOmegaData(makes)

        goalZone = FieldLandmarks.goalZoneCorners.map {
            if (blue) Vector2d(-it.x, it.y) else Vector2d(it)
        }
        farZone = FieldLandmarks.farZoneCorners.map {
            if (blue) Vector2d(-it.x, it.y) else Vector2d(it)
        }
    }

    override fun update(dt: Duration, aimTurret: Boolean) {
        updateVision()
        setTurretInputs()
        setShooterInputs(aimTurret)
        positionOverride?.let {
            if (robot.turret.fieldRelativeMode) robot.turret.fieldTargetAngle = it
            else robot.turret.targetAngle = it
        }
    }

    override fun close() { autoAim.close() }

    // ── Internal ─────────────────────────────────────────────────────

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
        val nBalls = robot.beamBreak.ballCount.coerceAtLeast(if (shotRequested) 1 else 0)

        val odoVel = Vector2d(robot.io.velocity().v)
        val vel = odoVel.rotate(fusedPose.rot - robot.io.position().rot)

        val tX = fusedPose.v.x.toFloat()
        val tY = fusedPose.v.y.toFloat()
        val tZ = turretPos.z.toFloat()
        val goal = FieldLandmarks.goalPosition3d(blue, AimConfig.goalHeight)
        val gX = goal.x.toFloat(); val gY = goal.y.toFloat(); val gZ = goal.z.toFloat()
        val curTheta = (turret.pos + fusedPose.rot).toFloat()
        val curPhi   = shooter.computedHoodAngle.toFloat()
        val curOmega = robot.io.flywheelVelocity().toFloat()
        val robotVx = vel.x.toFloat()
        val robotVy = vel.y.toFloat()

        diagGoalDist = hypot((gX - tX).toDouble(), (gY - tY).toDouble()).toFloat()

        // Direct ballistics probe at T=0.5s (bypasses robust solver — for diagnostics)
        try {
            val probe = bridge.solve(tX, tY, tZ, gX, gY, gZ, robotVx, robotVy, 0.5f, physConfig, omegaCoeffs)
            diagPhi05   = Math.toDegrees(probe[1].toDouble()).toFloat()
            diagVexit05 = probe[2]
        } catch (e: Exception) {
            diagPhi05   = Float.NaN
            diagVexit05 = Float.NaN
        }

        // optimalTCold: exact same path as flight_time_cold inside solve_1ball
        try {
            val cold = bridge.optimalTCold(tX, tY, tZ, gX, gY, gZ, robotVx, robotVy,
                curTheta, curPhi, curOmega, weights, bounds, physConfig, omegaCoeffs)
            diagColdFeasible = cold[TurretPlannerBridge.OptimalTIdx.FEASIBLE]
            diagColdTstar    = cold[TurretPlannerBridge.OptimalTIdx.T_STAR]
            diagColdPhi      = Math.toDegrees(cold[TurretPlannerBridge.OptimalTIdx.PHI].toDouble()).toFloat()
        } catch (e: Exception) {
            diagColdFeasible = -2f
        }

        val inZone = robot.intakeCoordinator.inShootingZone

        // During burst (transfer running), solve only for the current ball
        val inBurst = robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING
        val solveNBalls = if (inBurst) 1 else nBalls.coerceAtLeast(1)

        // Single-state trajectory: current robot state only
        val traj = floatArrayOf(0f, tX, tY, fusedPose.rot.toFloat(), robotVx, robotVy)

        var solved = false
        isRobustActive = false
        secondaryShotState = null
        tertiaryShotState = null
        transferPowerCommand = 1.0f

        try {
            val r = bridge.robust3ShotPlan(
                traj, 1, tZ,
                gX, gY, gZ,
                curTheta, curPhi, curOmega,
                solveNBalls,
                0f,
                AimConfig.transferDelay.toFloat(),
                AimConfig.dropFraction.toFloat(),
                weights, bounds, physConfig, omegaCoeffs
            )

            lastSolveFeasible = r[Robust3ShotPlanIdx.FEASIBLE]
            lastSolveException = "none"

            if (r[Robust3ShotPlanIdx.FEASIBLE] > 0.5f) {
                var solverTheta = r[Robust3ShotPlanIdx.S1_THETA].toDouble()
                val relAngle = wrapAngle(solverTheta - fusedPose.rot)
                if (abs(relAngle) > TurretServoConfig.maxAngle) {
                    solverTheta = wrapAngle(
                        relAngle.coerceIn(TurretServoConfig.minAngle, TurretServoConfig.maxAngle)
                                + fusedPose.rot
                    )
                }
                lastTheta = solverTheta.toFloat()
                lastPhi   = r[Robust3ShotPlanIdx.S1_PHI]
                lastOmega = r[Robust3ShotPlanIdx.S1_OMEGA]
                lastT1    = r[Robust3ShotPlanIdx.T1]
                solved = true

                if (!inBurst && solveNBalls >= 2) {
                    isRobustActive = true
                    secondaryShotState = Ballistics.ShotState(
                        theta = r[Robust3ShotPlanIdx.S2_THETA].toDouble(),
                        phi   = r[Robust3ShotPlanIdx.S2_PHI].toDouble(),
                        vExit = r[Robust3ShotPlanIdx.S2_OMEGA].toDouble() * flywheelRadius * AimConfig.launchEfficiency,
                    )
                    // Compute required inter-shot interval; slow transfer if needed
                    val j12 = r[Robust3ShotPlanIdx.J_12]
                    val j23 = if (solveNBalls >= 3) r[Robust3ShotPlanIdx.J_23] else 0f
                    val requiredDt = maxOf(j12, j23).coerceAtLeast(AimConfig.transferDelay.toFloat())
                    transferPowerCommand = (AimConfig.transferDelay.toFloat() / requiredDt).coerceIn(0.5f, 1.0f)
                }
                if (!inBurst && solveNBalls >= 3) {
                    tertiaryShotState = Ballistics.ShotState(
                        theta = r[Robust3ShotPlanIdx.S3_THETA].toDouble(),
                        phi   = r[Robust3ShotPlanIdx.S3_PHI].toDouble(),
                        vExit = r[Robust3ShotPlanIdx.S3_OMEGA].toDouble() * flywheelRadius * AimConfig.launchEfficiency,
                    )
                }
            }
        } catch (e: Exception) {
            lastSolveException = e.javaClass.simpleName + ": " + (e.message ?: "")
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

        val prespin = nBalls > 0 && (inZone || shotRequested)
        if (robot.aimFlywheel) {
            shooter.hoodAngle = lastPhi.toDouble()
            if (prespin) shooter.flywheelTarget = lastOmega.toDouble()
        }

        // readyToShoot: forward-simulate from current actuator state
        val actualTheta = (turret.pos + fusedPose.rot).toFloat()
        val actualPhi   = shooter.computedHoodAngle.toFloat()
        val actualOmega = robot.io.flywheelVelocity().toFloat()
        val actualVexit = vExitFromOmega(actualPhi, actualOmega)
        val missDistance = bridge.shotError(
            tX, tY, tZ, gX, gY, gZ, robotVx, robotVy,
            actualTheta, actualPhi, actualVexit, actualOmega,
            lastT1, physConfig
        )
        readyToShoot = missDistance < AimConfig.shotTolerance && inZone
    }

    /** Invert the IDW omega map: find v_exit such that omegaMapEval(phi, v_exit) == omega. */
    private fun vExitFromOmega(phi: Float, omega: Float): Float {
        var lo = 0f
        var hi = (AimConfig.vMax).toFloat()
        repeat(30) {
            val mid = (lo + hi) / 2f
            if (bridge.omegaMapEval(phi, mid, omegaCoeffs) < omega) lo = mid else hi = mid
        }
        return (lo + hi) / 2f
    }

    private fun wrapAngle(a: Double): Double {
        var r = a % (2 * PI)
        if (r > PI)  r -= 2 * PI
        if (r < -PI) r += 2 * PI
        return r
    }

    /**
     * Build the IDW omega data array for the native solver.
     *
     * Converts real-world OmegaTuner make samples directly to (phi, v_exit, omega) IDW points.
     * When no real samples exist, falls back to physics-model anchor points spanning the
     * operating range so IDW interpolation is well-conditioned and returns achievable omegas.
     *
     * Format: [N, phi_0, v_exit_0, omega_0, phi_1, v_exit_1, omega_1, ...]
     */
    private fun buildOmegaData(makes: List<BisectionSample>): FloatArray {
        val pts = makes.mapNotNull { s ->
            val phi = Math.toRadians(s.hoodAngleDeg)
            val v = OmegaCoefFitter.distanceHoodToVExit(s.distance, phi) ?: return@mapNotNull null
            Triple(phi.toFloat(), v.toFloat(), s.omegaMeasured.toFloat())
        }

        if (pts.isNotEmpty()) {
            val arr = FloatArray(1 + pts.size * 3)
            arr[0] = pts.size.toFloat()
            pts.forEachIndexed { i, (phi, v, omega) ->
                arr[1 + i*3 + 0] = phi
                arr[1 + i*3 + 1] = v
                arr[1 + i*3 + 2] = omega
            }
            return arr
        }

        // No real data yet: physics-model anchor points omega = v / (radius * launchEfficiency).
        // Three points spanning the hood and speed range so IDW interpolates (not just constant).
        val omegaMax = flywheelMotor.freeSpeed.toFloat()
        val vMaxF = (omegaMax * flywheelRadius * AimConfig.launchEfficiency).toFloat()
        val phiLo = Math.toRadians(ShooterConfig.minAngleDeg).toFloat()
        val phiHi = Math.toRadians(ShooterConfig.maxAngleDeg).toFloat()
        val phiMid = (phiLo + phiHi) / 2f
        return floatArrayOf(
            3f,
            phiLo, vMaxF * 0.4f, omegaMax * 0.4f,
            phiMid, vMaxF * 0.7f, omegaMax * 0.7f,
            phiHi, vMaxF * 0.95f, omegaMax * 0.95f,
        )
    }
}
