package sigmacorns.logic

import org.joml.Vector2d
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.ballExitRadius
import sigmacorns.constants.flywheelRadius
import sigmacorns.constants.turretPos
import sigmacorns.control.aim.AutoAim
import sigmacorns.control.aim.TurretPlannerBridge
import sigmacorns.control.aim.TurretPlannerBridge.OptimalTIdx
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

    /** Warm-start T* from the previous frame; null → cold start. */
    private var lastTStar: Float? = null

    /** Solver outputs from the last feasible frame. */
    private var lastTheta: Float = 0f
    private var lastPhi:   Float = 0f
    private var lastOmega: Float = 0f

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

        val result: FloatArray = try {
            val tStar = lastTStar
            if (tStar != null) {
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
        } catch (e: Exception) {
            readyToShoot = false
            lastTStar = null
            return
        }

        val feasible = result[OptimalTIdx.FEASIBLE] > 0.5f
        if (!feasible) {
            readyToShoot = false
            lastTStar = null
            return
        }

        lastTStar = result[OptimalTIdx.T_STAR]
        lastTheta = result[OptimalTIdx.THETA]
        lastPhi   = result[OptimalTIdx.PHI]
        lastOmega = result[OptimalTIdx.OMEGA]

        if (aimTurret) {
            turret.fieldRelativeMode = true
            turret.fieldTargetAngle = lastTheta.toDouble()
        }

        if (robot.aimFlywheel) {
            shooter.hoodAngle = lastPhi.toDouble()
            shooter.flywheelTarget = lastOmega.toDouble()
        }

        // Readiness check
        val thetaErr = abs(wrapAngle(turret.pos + fusedPose.rot - lastTheta))
        val phiErr   = abs(shooter.computedHoodAngle - lastPhi)
        val omegaErr = abs(robot.io.flywheelVelocity() - lastOmega)
        readyToShoot = thetaErr < turretTol && phiErr < hoodTol && omegaErr < omegaTol

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
