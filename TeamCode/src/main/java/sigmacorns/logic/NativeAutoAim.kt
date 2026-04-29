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
import sigmacorns.opmode.SigmaOpMode.Companion.SIM
import sigmacorns.subsystem.IntakeTransfer
import sigmacorns.subsystem.ShooterConfig
import sigmacorns.subsystem.TurretServoConfig
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

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
    private var shotRequestedTime: kotlin.time.Duration? = null
    override var readyToShoot: Boolean = false
        private set
    override var transferPowerCommand: Float = 1.0f
        private set
    override var targetDistance: Double = 3.0
        private set

    private val bridge = TurretPlannerBridge()

    // Native handle to parsed OmegaMapParams — created at init, avoids per-call unpack.
    private var omegaHandle: Long = 0L

    // Accumulates time since last optimalTColdH diagnostic call (throttled to ~1 Hz).
    private var diagColdAccumSec: Double = 10.0

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
    var missDistance: Float = Float.NaN
    private set

    var solved = false
    private set

    /** Snapshot of last robust-solve output for telemetry/visualizer. Null until first solve attempt. */
    data class SolverDiag(
        val feasible: Boolean,
        val heightOk: Boolean,
        val T1: Float,
        val T2: Float,
        val J: Float,
        val J12: Float,
        val J23: Float,
        val s1Theta: Float,
        val s1Phi: Float,
        val s1Omega: Float,
        val s2Theta: Float,
        val s2Phi: Float,
        val s2Omega: Float,
        val omAfterDrop: Float,
        val missDistance: Float,
        val exception: String,
    )
    var lastSolverDiag: SolverDiag? = null
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

        val makes = if (SIM) emptyList() else BisectionStore().also { it.load() }.madeSamples()
        omegaDataPoints = makes.size
        omegaCoeffs = buildOmegaData(makes)

        goalZone = FieldLandmarks.goalZoneCorners.map {
            if (blue) Vector2d(-it.x, it.y) else Vector2d(it)
        }
        farZone = FieldLandmarks.farZoneCorners.map {
            if (blue) Vector2d(-it.x, it.y) else Vector2d(it)
        }

        omegaHandle = bridge.createOmegaMap(omegaCoeffs)
    }

    override fun update(dt: Duration, aimTurret: Boolean) {
        diagColdAccumSec += dt.toDouble(DurationUnit.SECONDS)
        updateVision()
        setTurretInputs()
        setShooterInputs(aimTurret)
        positionOverride?.let {
            if (robot.turret.fieldRelativeMode) robot.turret.fieldTargetAngle = it
            else robot.turret.targetAngle = it
        }
    }

    override fun close() {
        autoAim.close()
        if (omegaHandle != 0L) {
            bridge.destroyOmegaMap(omegaHandle)
            omegaHandle = 0L
        }
    }

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

    var burstStart = 0.seconds
    var burst = false

    private fun setShooterInputs(aimTurret: Boolean) {
        val shooter = robot.shooter
        val turret  = robot.turret
        val fusedPose = autoAim.fusedPose

        val nBalls = robot.beamBreak.ballCount.coerceAtLeast(if (shotRequested) {
            val tBurst = robot.io.time() - burstStart
            if(!burst || tBurst < AimConfig.transferDelay.seconds*0.5) 3
            else if( tBurst < AimConfig.transferDelay.seconds*1.5) 2
            else if( tBurst < AimConfig.transferDelay.seconds*2.5) 1
            else {
                burst = false
                1
            }
        } else 0)

        val odoVel = Vector2d(robot.io.velocity().v)
        val vel = odoVel.rotate(fusedPose.rot - robot.io.position().rot)

        // Turret pivot in field frame: robot center + chassis offset along robot heading.
        // JoltSimIO.shootBall launches from this same point (RobotModelConstants.turretPos.x).
        val cosRot = kotlin.math.cos(fusedPose.rot)
        val sinRot = kotlin.math.sin(fusedPose.rot)
        val tX = (fusedPose.v.x + turretPos.x * cosRot).toFloat()
        val tY = (fusedPose.v.y + turretPos.x * sinRot).toFloat()
        val tZ = turretPos.z.toFloat()
        val goal = FieldLandmarks.goalPosition3d(blue, AimConfig.goalHeight)
        val gX = goal.x.toFloat(); val gY = goal.y.toFloat(); val gZ = goal.z.toFloat()
        val curTheta = (turret.pos + fusedPose.rot).toFloat()
        val curPhi   = shooter.computedHoodAngle.toFloat()
        val curOmega = robot.io.flywheelVelocity().toFloat()
        val robotVx = vel.x.toFloat()
        val robotVy = vel.y.toFloat()

        diagGoalDist = hypot((gX - tX).toDouble(), (gY - tY).toDouble()).toFloat()

        // Direct ballistics probe at T=0.5s (diagnostics only — uses cached handle)
        try {
            val probe = bridge.solveH(tX, tY, tZ, gX, gY, gZ, robotVx, robotVy, 0.5f, physConfig, omegaHandle)
            diagPhi05   = Math.toDegrees(probe[1].toDouble()).toFloat()
            diagVexit05 = probe[2]
        } catch (e: Exception) {
            diagPhi05   = Float.NaN
            diagVexit05 = Float.NaN
        }

        // optimalTCold: throttled to ~1 Hz — full Piyavskii-Shubert is expensive
        if (diagColdAccumSec >= 10000000.0) {
            diagColdAccumSec = 0.0
            try {
                val cold = bridge.optimalTColdH(tX, tY, tZ, gX, gY, gZ, robotVx, robotVy,
                    curTheta, curPhi, curOmega, weights, bounds, physConfig, omegaHandle)
                diagColdFeasible = cold[TurretPlannerBridge.OptimalTIdx.FEASIBLE]
                diagColdTstar    = cold[TurretPlannerBridge.OptimalTIdx.T_STAR]
                diagColdPhi      = Math.toDegrees(cold[TurretPlannerBridge.OptimalTIdx.PHI].toDouble()).toFloat()
            } catch (e: Exception) {
                diagColdFeasible = -2f
            }
        }

        val inZone = robot.intakeCoordinator.inShootingZone

        // During burst (transfer running), solve only for the current ball
        val inBurst = robot.intakeTransfer.state == IntakeTransfer.State.TRANSFERRING
        val solveNBalls = if (inBurst) 1 else nBalls.coerceAtLeast(1)

        // Build a predicted trajectory with one state per ball, spaced by transferDelay.
        // Constant-velocity projection; gives solve_2ball / solve_3ball real future states
        // instead of the degenerate single-state case where they immediately bail out.
        val transferDelayF = AimConfig.transferDelay.toFloat()
        val heading = fusedPose.rot.toFloat()
        val traj = FloatArray(solveNBalls * 6) { 0f }
        for (i in 0 until solveNBalls) {
            val dt = i * transferDelayF
            val base = i * 6
            traj[base + 0] = dt
            traj[base + 1] = tX + robotVx * dt
            traj[base + 2] = tY + robotVy * dt
            traj[base + 3] = heading
            traj[base + 4] = robotVx
            traj[base + 5] = robotVy
        }

        solved = false
        isRobustActive = false
        secondaryShotState = null
        tertiaryShotState = null
        transferPowerCommand = 1.0f

        try {
            val r = bridge.robust3ShotPlanH(
                traj, solveNBalls, tZ,
                gX, gY, gZ,
                curTheta, curPhi, curOmega,
                solveNBalls,
                0f,
                transferDelayF,
                AimConfig.dropFraction.toFloat(),
                weights, bounds, physConfig, omegaHandle
            )

            lastSolveFeasible = r[Robust3ShotPlanIdx.FEASIBLE]
            lastSolveException = "none"

            val s1Om = r[Robust3ShotPlanIdx.S1_OMEGA]
            val omAfterDrop = s1Om * (1f - AimConfig.dropFraction.toFloat())
            val s2Om = r[Robust3ShotPlanIdx.S2_OMEGA]
            val heightOk = s2Om <= omAfterDrop || solveNBalls < 2
            lastSolverDiag = SolverDiag(
                feasible  = r[Robust3ShotPlanIdx.FEASIBLE] > 0.5f,
                heightOk  = heightOk,
                T1        = r[Robust3ShotPlanIdx.T1],
                T2        = r[Robust3ShotPlanIdx.T2],
                J         = r[Robust3ShotPlanIdx.J],
                J12       = r[Robust3ShotPlanIdx.J_12],
                J23       = r[Robust3ShotPlanIdx.J_23],
                s1Theta   = r[Robust3ShotPlanIdx.S1_THETA],
                s1Phi     = r[Robust3ShotPlanIdx.S1_PHI],
                s1Omega   = s1Om,
                s2Theta   = r[Robust3ShotPlanIdx.S2_THETA],
                s2Phi     = r[Robust3ShotPlanIdx.S2_PHI],
                s2Omega   = s2Om,
                omAfterDrop = omAfterDrop,
                missDistance = missDistance,
                exception = "none",
            )

            if (r[Robust3ShotPlanIdx.FEASIBLE] > 0.5f) {
                var solverTheta = r[Robust3ShotPlanIdx.S1_THETA].toDouble()
                val relAngle = wrapAngle(solverTheta - fusedPose.rot)
                val thetaInRange = abs(relAngle) <= TurretServoConfig.maxAngle
                if (!thetaInRange) {
                    solverTheta = wrapAngle(
                        relAngle.coerceIn(TurretServoConfig.minAngle, TurretServoConfig.maxAngle)
                                + fusedPose.rot
                    )
                }
                lastTheta = solverTheta.toFloat()
                lastPhi   = r[Robust3ShotPlanIdx.S1_PHI]
                lastOmega = r[Robust3ShotPlanIdx.S1_OMEGA]
                lastT1    = r[Robust3ShotPlanIdx.T1]
                // Only mark solved (eligible to fire) when turret can actually reach the angle.
                // Still set lastTheta/Phi/Omega so the flywheel can prespin.
                solved = thetaInRange

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
            lastSolverDiag = SolverDiag(
                feasible = false, heightOk = false,
                T1 = 0f, T2 = 0f, J = Float.NaN, J12 = Float.NaN, J23 = Float.NaN,
                s1Theta = 0f, s1Phi = 0f, s1Omega = 0f,
                s2Theta = 0f, s2Phi = 0f, s2Omega = 0f,
                omAfterDrop = 0f, missDistance = Float.NaN,
                exception = lastSolveException,
            )
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

        val prespin = nBalls > 0 || inZone || shotRequested
        if (robot.aimFlywheel) {
            shooter.hoodAngle = lastPhi.toDouble()
            shooter.flywheelTarget = lastOmega.toDouble()
        }

        // readyToShoot: forward-simulate from current actuator state
        val actualTheta = (turret.pos + fusedPose.rot).toFloat()
        val actualPhi   = shooter.computedHoodAngle.toFloat()
        val actualOmega = robot.io.flywheelVelocity().toFloat()
        val actualVexit = bridge.vExitFromOmegaH(actualPhi, actualOmega, AimConfig.vMax.toFloat(), omegaHandle)
        missDistance = bridge.shotError(
            tX, tY, tZ, gX, gY, gZ, robotVx, robotVy,
            actualTheta, actualPhi, actualVexit, actualOmega,
            lastT1, physConfig
        )

        lastSolverDiag = lastSolverDiag?.copy(missDistance = missDistance)

        readyToShoot = solved && (actualTheta - lastTheta).absoluteValue < 0.1 && (actualPhi - lastPhi).absoluteValue < 0.05 && (actualOmega-lastOmega).absoluteValue < 8

        if(readyToShoot &&nBalls > 3) {
            burst = true
            burstStart = robot.io.time()
        }
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

        // No real data yet: lay down a dense 2D grid of physics-model anchors.
        // Every anchor satisfies the true SIM relation omega = v / (radius * launchEfficiency).
        // A *diagonal* (3 colinear points) is degenerate for IDW — queries off the diagonal
        // get pulled to the nearest anchor's omega instead of the physical value, which
        // makes close shots overshoot (high-phi/low-v query snaps to high-omega anchor)
        // and far shots undershoot (low-phi/high-v query snaps to low-omega anchor).
        val omegaMax = flywheelMotor.freeSpeed.toFloat()
        val vMaxF = (omegaMax * flywheelRadius * AimConfig.launchEfficiency).toFloat()
        val phiLo = Math.toRadians(ShooterConfig.minAngleDeg).toFloat()
        val phiHi = Math.toRadians(ShooterConfig.maxAngleDeg).toFloat()
        val nPhi = 8
        val nV = 8
        val arr = FloatArray(1 + nPhi * nV * 3)
        arr[0] = (nPhi * nV).toFloat()
        var idx = 1
        for (i in 0 until nPhi) {
            val phi = phiLo + (phiHi - phiLo) * i / (nPhi - 1)
            for (j in 0 until nV) {
                val v = vMaxF * (0.05f + 0.9f * j / (nV - 1))
                val om = v / (flywheelRadius.toFloat() * AimConfig.launchEfficiency.toFloat())
                arr[idx++] = phi
                arr[idx++] = v
                arr[idx++] = om
            }
        }
        return arr
    }
}
