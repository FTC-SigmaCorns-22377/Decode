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
import sigmacorns.math.closestPointOnConvexPolygon
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
 * Uses `robust3ShotPlan` to plan up to 3 consecutive shots with hard
 * constraints: J_12 <= transferTime and J_23 <= transferTime ensure the
 * flywheel can recover between shots. The burst is only triggered when the
 * solver finds a feasible plan for all loaded balls.
 *
 * Zone logic uses [FieldLandmarks.goalZoneCorners] and [FieldLandmarks.farZoneCorners]
 * to determine if the robot is in a shooting zone. Outside the zone, t_remaining
 * is inflated so the solver optimizes purely for inter-shot robustness.
 */
class NativeAutoAim(
    private val robot: Robot,
    private val blue: Boolean,
) : AutoAim {
    companion object {
        private val Double.f get() = "%.3f".format(this)
        private val Float.f  get() = "%.3f".format(this)
    }

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

    override var primaryShotState: Ballistics.ShotState? = null
        private set
    override var secondaryShotState: Ballistics.ShotState? = null
        private set
    override var tertiaryShotState: Ballistics.ShotState? = null
        private set
    override var isRobustActive: Boolean = false
        private set

    private var logFrame = 0

    // ── Burst sequencing ─────────────────────────────────────────────
    private var burstActive = false
    private var burstStartBallCount = 0
    private var burstShotsFired = 0
    private var burstTotalShots = 0
    private var lastBallExitTime: Duration? = null

    /** True when the last solve was feasible for all balls (burst is safe to start). */
    private var allBallsFeasible = false

    override var plannedShot: PlannedShot? = null
        set(value) {
            field = value
            plannedShotSetTime = if (value != null) robot.io.time() else null
        }
    private var plannedShotSetTime: kotlin.time.Duration? = null

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
            ShotSolverConfig.wOmega.toFloat()
        )
        val c1 = (1.0 / (flywheelRadius * AimConfig.launchEfficiency)).toFloat()
        omegaCoeffs = floatArrayOf(0f, c1, 0f, 0f, 0f, 0f)

        // Mirror zones for alliance (FieldLandmarks uses red-origin coords)
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
        val shouldLog = (logFrame++ % 25 == 0) && false
        val shooter = robot.shooter
        val turret  = robot.turret
        val fusedPose = autoAim.fusedPose
        val now = robot.io.time()
        val nBalls = robot.beamBreak.ballCount

        // ── Burst sequencing: detect ball exits ──────────────────────
        if (burstActive) {
            val fired = burstStartBallCount - nBalls
            if (fired > burstShotsFired) {
                burstShotsFired = fired
                lastBallExitTime = now
            }
            if (burstShotsFired >= burstTotalShots || nBalls == 0) {
                burstActive = false
                burstShotsFired = 0
                lastBallExitTime = null
            }
        }

        // ── t_remaining ──────────────────────────────────────────────
        val tRemaining: Float = if (burstActive && lastBallExitTime != null) {
            val elapsed = (now - lastBallExitTime!!).inWholeMilliseconds / 1000.0
            (AimConfig.transferDelay - elapsed).coerceAtLeast(0.0).toFloat()
        } else {
            AimConfig.transferDelay.toFloat()
        }

        val remainingBalls = if (burstActive)
            (burstTotalShots - burstShotsFired).coerceIn(1, nBalls)
        else nBalls.coerceAtLeast(1)

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

        // ── Time to zone estimate ─────────────────────────────────────
        // Skipped when a planned shot is set — timeUntilPlannedShot carries the
        // exact countdown instead, so the velocity-projection scan is redundant.
        val timeToZone = run {
            if (robot.intakeCoordinator.inShootingZone) return@run 0.0
            if (plannedShot != null) return@run Double.MAX_VALUE

            val robotPos = Vector2d(fusedPose.v.x, fusedPose.v.y)
            val speed = hypot(vel.x, vel.y)

            var bestTime = Double.MAX_VALUE
            for (zone in listOf(goalZone, farZone)) {
                val closest = closestPointOnConvexPolygon(zone, robotPos)
                val dx = closest.x - robotPos.x
                val dy = closest.y - robotPos.y
                val dist = hypot(dx, dy)

                if (dist < 1e-3) {
                    bestTime = 0.0
                    break
                }
                if (speed < 1e-3) continue
                val approachSpeed = (vel.x * dx + vel.y * dy) / dist
                if (approachSpeed <= 1e-3) continue
                bestTime = minOf(bestTime, dist / approachSpeed)
            }
            bestTime
        }

        // Time until the robot reaches the planned shot position, counting down from
        // when the plan was set.  Used for both prespin gating and solverTRemaining.
        val timeUntilPlannedShot: Double? = run {
            val ps = plannedShot ?: return@run null
            val setTime = plannedShotSetTime ?: return@run null
            val elapsed = (now - setTime).inWholeMilliseconds / 1000.0
            (ps.timeToArrival.inWholeMilliseconds / 1000.0 - elapsed).coerceAtLeast(0.0)
        }

        // The solver's t_remaining: prefer the exact planned arrival time so the
        // C++ solver pre-aims for the correct position; fall back to zone estimate.
        val solverTRemaining = when {
            burstActive -> tRemaining
            timeUntilPlannedShot != null -> (tRemaining + timeUntilPlannedShot).toFloat()
            timeToZone > 0 -> tRemaining + timeToZone.toFloat()
            else -> tRemaining
        }

        if (shouldLog) {
            val ps = plannedShot
            if (ps != null) {
                val horizon = solverTRemaining + AimConfig.transferDelay * remainingBalls + AimConfig.predictionHorizon
                println("[NativeAutoAim] plannedShot: eta=${ps.timeToArrival.inWholeMilliseconds}ms" +
                        " remaining=${timeUntilPlannedShot?.let { "%.2fs".format(it) } ?: "null"}" +
                        " solverT=${"%.3f".format(solverTRemaining)}s horizon=${"%.3f".format(horizon)}s" +
                        " balls=$remainingBalls" +
                        " pos=(${ps.state.pos.v.x.f},${ps.state.pos.v.y.f})" +
                        " vel=(${ps.state.vel.v.x.f},${ps.state.vel.v.y.f})")
            } else {
                println("[NativeAutoAim] plannedShot=null timeToZone=${timeToZone.f}s solverT=${solverTRemaining.f}s")
            }
        }

        var solved = false
        isRobustActive = false
        secondaryShotState = null
        tertiaryShotState = null
        allBallsFeasible = false

        // ── Solve ────────────────────────────────────────────────────
        try {
            val ps = plannedShot
            val nSteps: Int
            val trajArray: FloatArray

            if (ps != null && !burstActive) {
                // Linearly interpolate from current state to planned state over
                // [0, solverTRemaining], then hold the planned state for the rest
                // of the horizon so the solver has room to plan the full burst.
                val horizon = (solverTRemaining + AimConfig.transferDelay * remainingBalls + AimConfig.predictionHorizon).toFloat()
                nSteps = 20
                trajArray = FloatArray(nSteps * 6)
                val arriveT = if (solverTRemaining > 1e-6f) solverTRemaining else 1e-6f
                val shotX = ps.state.pos.v.x.toFloat()
                val shotY = ps.state.pos.v.y.toFloat()
                val shotVx = ps.state.vel.v.x.toFloat()
                val shotVy = ps.state.vel.v.y.toFloat()
                val heading = fusedPose.rot.toFloat()
                for (i in 0 until nSteps) {
                    val t = (i.toFloat() / (nSteps - 1)) * horizon
                    val frac = (t / arriveT).coerceIn(0f, 1f)
                    trajArray[i * 6 + 0] = t
                    trajArray[i * 6 + 1] = tX + frac * (shotX - tX)
                    trajArray[i * 6 + 2] = tY + frac * (shotY - tY)
                    trajArray[i * 6 + 3] = heading
                    trajArray[i * 6 + 4] = robotVx + frac * (shotVx - robotVx)
                    trajArray[i * 6 + 5] = robotVy + frac * (shotVy - robotVy)
                }
            } else {
                val horizon = solverTRemaining + AimConfig.transferDelay * remainingBalls + AimConfig.predictionHorizon
                val maxSteps = 60
                val step = maxOf(AimConfig.predictionStep.toFloat(), (horizon / maxSteps).toFloat())
                nSteps = (horizon / step).toInt().coerceIn(2, maxSteps)
                trajArray = FloatArray(nSteps * 6)

                // Integrate trajectory with deceleration near shooting zones.
                // When the predicted position is within 0.5m of a zone, assume the
                // driver brakes — this prevents the solver from picking backward
                // solutions caused by high velocity near the goal.
                var px = tX
                var py = tY
                var vx = robotVx
                var vy = robotVy
                var decelerating = robot.intakeCoordinator.inShootingZone

                for (i in 0 until nSteps) {
                    // Check proximity to zones before recording velocity
                    if (!decelerating) {
                        val pos = Vector2d(px.toDouble(), py.toDouble())
                        for (zone in listOf(goalZone, farZone)) {
                            val closest = closestPointOnConvexPolygon(zone, pos)
                            if (hypot(closest.x - pos.x, closest.y - pos.y) < AimConfig.decelZoneMargin) {
                                decelerating = true
                                break
                            }
                        }
                    }

                    val t = i * step
                    trajArray[i * 6 + 0] = t
                    trajArray[i * 6 + 1] = px
                    trajArray[i * 6 + 2] = py
                    trajArray[i * 6 + 3] = fusedPose.rot.toFloat()
                    trajArray[i * 6 + 4] = vx
                    trajArray[i * 6 + 5] = vy

                    // Advance position, then apply deceleration for the next step
                    if (i < nSteps - 1) {
                        px += vx * step
                        py += vy * step

                        if (decelerating) {
                            val speed = hypot(vx.toDouble(), vy.toDouble()).toFloat()
                            if (speed > 0.01f) {
                                val newSpeed = maxOf(0f, speed - AimConfig.decelRate * step)
                                vx *= newSpeed / speed
                                vy *= newSpeed / speed
                            } else {
                                vx = 0f; vy = 0f
                            }
                        }
                    }
                }
            }

            val r = bridge.robust3ShotPlan(
                trajArray, nSteps, tZ,
                gX, gY, gZ,
                curTheta, curPhi, curOmega,
                remainingBalls,
                solverTRemaining,
                AimConfig.transferDelay.toFloat(),
                AimConfig.dropFraction.toFloat(),
                weights, bounds, physConfig, omegaCoeffs
            )

            if (r[Robust3ShotPlanIdx.FEASIBLE] > 0.5f) {
                // s1 from the solver is computed at the future trajectory point
                // where the shot will actually happen. Use it directly as the
                // actuator target so turret/hood arrive in advance.
                // Clamp field-frame theta to the turret's reachable range.
                // The solver uses [-π,π] theta bounds (full circle) but the
                // physical turret can only reach ±90° from the robot heading.
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
                allBallsFeasible = true

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
        } catch (e: Exception) {
            if (shouldLog) println("[NativeAutoAim] " + "solver threw: $e")
        }

        if (shouldLog) {
            println("[NativeAutoAim] " + "solve: solved=$solved feasible=$allBallsFeasible" +
                    " balls=$nBalls burst=$burstActive" +
                    " shotReq=$shotRequested inZone=${robot.intakeCoordinator.inShootingZone}")
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

        // ── Actuator targeting with look-ahead ───────────────────────
        // During a burst, once a ball is committed to the transfer the hood
        // angle and turret for that ball are locked in (only flywheel speed
        // matters at exit). Start moving hood/turret toward the NEXT shot's
        // parameters immediately so the servos arrive in time.
        val nextShotState = secondaryShotState  // s2 is always the next shot after s1

        val targetTheta: Double
        val targetPhi: Double
        val targetOmega: Double

        if (burstActive && nextShotState != null && tRemaining <= 0.05) {
            // Ball in transit — keep flywheel on s1's omega (it's what the
            // current ball needs), but slew hood toward s2 now. Theta stays
            // on s1 to avoid overcorrection from the future-position angle.
            targetTheta = lastTheta.toDouble()
            targetPhi   = nextShotState.phi
            targetOmega = lastOmega.toDouble()  // hold current shot's flywheel
        } else {
            targetTheta = lastTheta.toDouble()
            targetPhi   = lastPhi.toDouble()
            targetOmega = lastOmega.toDouble()
        }

        if (aimTurret) {
            turret.fieldRelativeMode = true
            turret.fieldTargetAngle = targetTheta
        }

        val prespin = nBalls > 0 &&
                robot.intakeTransfer.state != IntakeTransfer.State.INTAKING &&
                ((timeUntilPlannedShot != null && timeUntilPlannedShot <= AimConfig.spinupLeadTime)
                        || timeToZone <= AimConfig.spinupLeadTime)

        if (robot.aimFlywheel) {
            shooter.hoodAngle = targetPhi
            // Only spin up the flywheel when close enough to a launch zone.
            // When idle, leave flywheelTarget alone so the opmode's idle speed holds.
            if (prespin || burstActive) {
                shooter.flywheelTarget = targetOmega
            }
        }

        // readyToShoot: forward-simulate the shot from the current position
        // with the current actuator state and check if it hits the goal.
        val inTolerance = run {
            val actualTheta = (turret.pos + fusedPose.rot).toFloat()
            val actualPhi   = shooter.computedHoodAngle.toFloat()
            val actualOmega = robot.io.flywheelVelocity().toFloat()
            val actualVexit = (actualOmega * flywheelRadius * AimConfig.launchEfficiency).toFloat()

            val missDistance = bridge.shotError(
                tX, tY, tZ,
                gX, gY, gZ,
                robotVx, robotVy,
                actualTheta, actualPhi, actualVexit, actualOmega,
                lastT1,
                physConfig
            )
            val miss = missDistance < AimConfig.shotTolerance
            val inZone = timeToZone <= 0 || timeUntilPlannedShot?.let { it <= 0.0 } == true
            if (shouldLog) {
                println("[NativeAutoAim] " + "inTolerance: missDistance=${"%.3f".format(missDistance)} miss=$miss inZone=$inZone" +
                        " timeToZone=${timeToZone.f} untilPlanned=${timeUntilPlannedShot?.let { "%.2f".format(it) } ?: "null"}" +
                        " actualOmega=${"%.1f".format(actualOmega)} targetOmega=${"%.1f".format(targetOmega)}" +
                        " prespin=$prespin")
            }
            miss && inZone
        }

        // ── Fire: only in zone, only when solver confirms all balls feasible ──
        // burstActive is managed independently of the turret check so a jerk that
        // throws the turret off only *pauses* the transfer (readyToShoot = false)
        // rather than aborting the burst entirely — the transfer resumes once the
        // turret corrects back within burstTurretTolerance.
        if (shotRequested && (burstActive || inTolerance) && allBallsFeasible) {
            if (!burstActive) {
                burstActive = true
                burstShotsFired = 0
                burstStartBallCount = nBalls
                burstTotalShots = nBalls
                lastBallExitTime = null
                plannedShot = null  // plan consumed — burst is now executing
            }
        } else {
            burstActive = false
        }

        val turretErr = kotlin.math.abs(turret.pos - turret.effectiveTargetAngle)
        readyToShoot = burstActive && turretErr < AimConfig.burstTurretTolerance
    }

    private fun wrapAngle(a: Double): Double {
        var r = a % (2 * PI)
        if (r > PI)  r -= 2 * PI
        if (r < -PI) r += 2 * PI
        return r
    }
}
