package sigmacorns.control

import com.bylazar.gamepad.Gamepad
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import sigmacorns.constants.Limelight
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.flywheelMotor
import sigmacorns.constants.flywheelParameters
import sigmacorns.control.mpc.ContourSelectionMode
import sigmacorns.control.mpc.MPCClient
import sigmacorns.control.mpc.MPCRunner
import sigmacorns.control.subsystem.AimingSystem
import sigmacorns.control.subsystem.DriveController
import sigmacorns.control.subsystem.Flywheel
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.constants.FieldLandmarks
import sigmacorns.io.DrakeSimIO
import sigmacorns.io.HardwareIO
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumState
import sigmacorns.sim.ProjectileDynamics
import sigmacorns.sim.ProjectileState
import sigmacorns.sim.viz.AimingVizState
import sigmacorns.sim.viz.ContourVizState
import sigmacorns.sim.viz.GTSAMVizState
import sigmacorns.sim.viz.LandmarkVizState
import sigmacorns.sim.viz.MPCHorizonState
import sigmacorns.sim.viz.PathPoint
import sigmacorns.sim.viz.Point3D
import java.lang.AutoCloseable
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.DurationUnit
import kotlin.use

class Robot(val io: SigmaIO, blue: Boolean): AutoCloseable {
    val aim = AimingSystem(io,blue)
    val flywheel = Flywheel(flywheelMotor, flywheelParameters.inertia, io)
    val logic = SpindexerLogic(io, flywheel)
    val drive = DriveController()

    val dispatcher = PollableDispatcher(io)
    val scope = CoroutineScope(dispatcher)

    val limelight = (io as? HardwareIO)?.limelight
    var mpc: MPCClient? = null
    var runner: MPCRunner? = null

    var aimTurret = true
    var aimFlywheel = true

    fun init(pos: Pose2d, apriltagTracking: Boolean) {
        io.configurePinpoint()
        io.setPosition(pos)
        aim.init(io.position(),apriltagTracking)
    }

    fun startMPC() {
        if(mpc != null) mpc?.close()
        limelight?.pipelineSwitch(Limelight.MPC_PIPELINE)

        mpc = MPCClient(
            drivetrainParameters,
            if(io is DrakeSimIO) Network.SIM_MPC else Network.LIMELIGHT,
            contourSelectionMode = ContourSelectionMode.POSITION,
            preIntegrate = 30.milliseconds,
            sampleLookahead = 0
        )

        println("IP = ${mpc!!.SOLVER_IP}")
        runner = MPCRunner(mpc!!,drive)
        runner!!.start()
    }

    fun stopMPC() {
        runner?.stop()
        runner = null
        mpc?.close()
        mpc = null
    }

    fun startApriltag() {
        limelight?.pipelineSwitch(Limelight.APRILTAG_PIPELINE)
    }

    private var lastTime = io.time()
    fun update() {
        val t = io.time()
        val dt = t - lastTime
        lastTime = t

        dispatcher.update()

        runner?.updateState(
            MecanumState(
                io.velocity(),
                io.position()
            ), 12.0, t)
        runner?.driveWithMPC(io, io.voltage())

        aim.update(dt, aimTurret)
        if(aimFlywheel) aim.getRecommendedFlywheelVelocity()?.let { logic.shotVelocity = it }

        logic.update( dt)

        updateControlViz()
    }

    /**
     * Push MPC, GTSAM, and aiming visualization data to DrakeSimIO for the web visualizer.
     */
    private fun updateControlViz() {
        val drakeIO = io as? DrakeSimIO ?: return

        // --- MPC Visualization ---
        val mpcClient = mpc
        if (mpcClient != null) {
            // Predicted state evolution (list of future positions the MPC predicts)
            val predicted = mpcClient.predictedEvolution
            if (predicted.isNotEmpty()) {
                drakeIO.setMPCPredicted(predicted.map { PathPoint(it.pos.v.x, it.pos.v.y) })
            }

            // Contour targets sent to the solver
            val contours = mpcClient.lastSentContours
            if (contours.isNotEmpty()) {
                drakeIO.setMPCContours(contours.map {
                    ContourVizState(
                        x = it.lineP.x, y = it.lineP.y,
                        theta = it.targetTheta,
                        vx = it.targetVx, vy = it.targetVy
                    )
                })
            }

            // Horizon metadata
            drakeIO.setMPCHorizon(MPCHorizonState(
                horizonSec = MPCClient.horizon.toDouble(DurationUnit.SECONDS),
                sampleIndex = mpcClient.latestSampleI,
                trajectoryComplete = mpcClient.isTrajectoryComplete(),
                knotTimesMs = MPCClient.timeOffsets.map { it.toDouble(DurationUnit.MILLISECONDS) }
            ))
        }

        // --- GTSAM Factor Graph Visualization ---
        val autoAim = aim.autoAim
        val fusedPose = autoAim.fusedPose

        val landmarkViz = FieldLandmarks.landmarks.map { (id, spec) ->
            LandmarkVizState(
                tagId = id,
                x = spec.position.x,
                y = spec.position.y,
                z = spec.position.z,
                yaw = spec.yaw,
                roll = spec.roll,
                pitch = spec.pitch
            )
        }

        val cov = autoAim.solverCovariance
        drakeIO.setGTSAMViz(GTSAMVizState(
            fusedX = fusedPose.v.x,
            fusedY = fusedPose.v.y,
            fusedTheta = fusedPose.rot,
            covXX = cov[0],
            covXY = cov[1],
            covYX = cov[3],
            covYY = cov[4],
            initialized = autoAim.hasTarget,
            landmarks = landmarkViz,
            detectedTags = if (autoAim.trackedTagId >= 0) listOf(autoAim.trackedTagId) else emptyList(),
            hasVision = autoAim.hasVisionTarget,
            usingPrediction = autoAim.usingPrediction,
            odoX = drakeIO.position().v.x,
            odoY = drakeIO.position().v.y,
            odoTheta = drakeIO.position().rot,
            odoCovXX = drakeIO.odometryCovXX,
            odoCovYY = drakeIO.odometryCovYY,
            trueX = drakeIO.truePosition().v.x,
            trueY = drakeIO.truePosition().v.y,
            trueTheta = drakeIO.truePosition().rot
        ))

        // --- Aiming / Move-While-Shoot Visualization ---
        val robotPose = io.position()
        val robotVel = io.velocity()
        val turretAngle = aim.turret.pos
        val turretFieldYaw = robotPose.rot + turretAngle
        val hoodPitch = drakeIO.turretAngle  // hood servo position
        val flywheelOmega = io.flywheelVelocity()
        val goalPos = aim.goalPosition

        // Compute predicted projectile arc
        val projectileArc = computeProjectileArc(
            robotPose, robotVel, turretFieldYaw, hoodPitch, flywheelOmega
        )

        drakeIO.setAimingViz(AimingVizState(
            goalX = goalPos.x,
            goalY = goalPos.y,
            turretFieldYaw = turretFieldYaw,
            hasTarget = autoAim.hasTarget,
            usingPrediction = autoAim.usingPrediction,
            targetDistance = aim.targetDistance,
            flywheelOmega = flywheelOmega,
            flywheelTarget = logic.shotVelocity ?: 0.0,
            flywheelReady = logic.shotVelocity?.let { abs(flywheelOmega - it) < 20.0 } ?: false,
            hoodPitch = hoodPitch,
            projectileArc = projectileArc,
            turretTargetAngle = aim.turret.effectiveTargetAngle,
            turretActualAngle = turretAngle,
            robotVx = robotVel.v.x,
            robotVy = robotVel.v.y
        ))
    }

    /**
     * Compute the predicted projectile trajectory arc for visualization.
     * Simulates projectile motion under gravity from the shooter exit point.
     */
    private fun computeProjectileArc(
        robotPose: Pose2d,
        robotVel: Pose2d,
        turretFieldYaw: Double,
        hoodPitch: Double,
        flywheelOmega: Double
    ): List<Point3D> {
        val exitSpeed = abs(flywheelOmega) * FLYWHEEL_VIZ_RADIUS * FLYWHEEL_VIZ_EFFICIENCY
        if (exitSpeed < 0.5) return emptyList()

        val horizSpeed = exitSpeed * cos(hoodPitch)
        val vertSpeed = exitSpeed * sin(hoodPitch)

        val vx = horizSpeed * cos(turretFieldYaw) + robotVel.v.x
        val vy = horizSpeed * sin(turretFieldYaw) + robotVel.v.y
        val vz = vertSpeed

        // Spawn point (turret location in world frame)
        val spawnX = robotPose.v.x + 0.01 * cos(turretFieldYaw)
        val spawnY = robotPose.v.y + 0.01 * sin(turretFieldYaw)
        val spawnZ = 0.28  // Approximate shooter height

        val dynamics = ProjectileDynamics(9.81)
        val points = mutableListOf<Point3D>()
        var state = ProjectileState(
            org.joml.Vector3d(spawnX, spawnY, spawnZ),
            org.joml.Vector3d(vx, vy, vz),
            true
        )

        val dt = 0.02
        val maxSteps = 60  // ~1.2 seconds of flight
        for (i in 0 until maxSteps) {
            points.add(Point3D(state.position.x, state.position.y, state.position.z))
            if (!state.active) break
            state = dynamics.integrate(dt, 0.005, state)
        }
        return points
    }

    override fun close() {
        aim.close()
        stopMPC()
    }

    companion object {
        private const val FLYWHEEL_VIZ_RADIUS = 0.048
        private const val FLYWHEEL_VIZ_EFFICIENCY = 0.24
    }
}