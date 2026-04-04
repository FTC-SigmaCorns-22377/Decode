package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.ballExitRadius
import sigmacorns.constants.flywheelRadius
import sigmacorns.constants.turretPos
import sigmacorns.control.aim.TurretPlannerBridge
import sigmacorns.control.aim.TurretPlannerBridge.OptimalTIdx
import sigmacorns.control.aim.TurretPlannerBridge.ZoneTrackerIdx
import sigmacorns.logic.AimConfig
import sigmacorns.logic.ShotSolverConfig
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.ShooterConfig
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

/**
 * TurretPlannerDebugOpMode
 *
 * Drive around manually while the native turret_planner computes aim targets.
 * Everything is logged to telemetry; targets are only applied to subsystems
 * when [applyTargets] is true (toggle with Gamepad 1 A).
 *
 * ─────────── FIELD GEOMETRY ───────────
 * FTC field: 3.66 m × 3.66 m  (±FIELD_HALF in x and y)
 * Robot is constrained to a box ±BOX_X × ±BOX_Y.
 *
 * Two shooting zones — isosceles right triangles:
 *   • Hypotenuse runs along the wall (full wall width = 2·ZONE_HALF_HYP).
 *   • Right-angle vertex points toward the field centre at depth = ZONE_HALF_HYP from wall.
 *   • Zone 1 (LARGE):  top wall  (+y side),  half-hyp = ZONE1_HALF_HYP
 *   • Zone 2 (SMALL):  bottom wall (−y side), half-hyp = ZONE2_HALF_HYP
 *
 * Half-plane representation used by the zone tracker (n·p ≥ d means inside zone):
 *   Zone 1: n = (0,+1), d = FIELD_HALF − ZONE1_HALF_HYP
 *   Zone 2: n = (0,−1), d = FIELD_HALF − ZONE2_HALF_HYP
 *
 * Note: the half-plane only captures depth-into-wall, not the diagonal legs
 * of the triangle. This is the expected simplification of the zone tracker model.
 *
 * ─────────── GAMEPAD 1 ───────────
 *   Left stick          — field-centric translate
 *   Right stick X       — rotate
 *   A (toggle)          — apply targets to hardware  (default: DRY RUN)
 *   B (toggle)          — cycle active zone source:  ZONE1 → ZONE2 → POINT → …
 *   X                   — reset both zone trackers
 */
@TeleOp(name = "Turret Planner Debug", group = "Test")
class TurretPlannerDebugOpMode : SigmaOpMode() {

    // ─────────── geometry constants ───────────────────────────────────────────

    companion object {
        // FTC field ±1.83 m; tighten slightly to avoid wall collisions
        const val FIELD_HALF     = 1.83f   // m
        const val BOX_X          = 1.60f   // ±m, robot operating box
        const val BOX_Y          = 1.60f   // ±m

        // Zone 1: large, top (+y) wall
        const val ZONE1_HALF_HYP = 1.00f   // hyp half-length → depth into field = 1.00 m
        // Zone 2: small, bottom (−y) wall
        const val ZONE2_HALF_HYP = 0.60f   // depth = 0.60 m

        // Zone tracker physics
        const val Q_PROCESS  = 0.05f   // position noise growth (m²/s)
        const val P_LOW      = 0.25f   // urgency threshold: below → idle
        const val P_HIGH     = 0.75f   // urgency threshold: above → full prep
        const val ALPHA_LPF  = 0.35f   // LPF coefficient (0 = no filter, 1 = hold)
        const val OMEGA_IDLE = 150f    // flywheel idle speed (rad/s) while zone-tracking

        // Goal — set to true for blue-alliance goal, false for red
        const val BLUE_ALLIANCE = false
    }

    // ─────────── opmode entry ─────────────────────────────────────────────────

    override fun runOpMode() {

        // Use Kotlin AimingSystem for GTSAM; TurretPlannerBridge called directly.
        // robot.aimTurret and robot.aimFlywheel are both false, so Robot.update()
        // runs vision/GTSAM and sensor fusion but does NOT override turret/shooter targets.
        val robot = Robot(io, blue = BLUE_ALLIANCE, useNativeAim = false)
        robot.aimTurret  = false
        robot.aimFlywheel = false

        // ── native bridge ──────────────────────────────────────────────────────
        val bridge: TurretPlannerBridge
        try {
            bridge = TurretPlannerBridge()
        } catch (e: UnsatisfiedLinkError) {
            telemetry.addLine("FATAL: libturret_planner_jni not found. Run buildTurretPlannerJni first.")
            telemetry.update()
            waitForStart()
            return
        }

        // ── config arrays (constant for the opmode lifetime) ──────────────────
        val physConfig = floatArrayOf(
            AimConfig.g.toFloat(),
            ballExitRadius.toFloat()
        )

        val vMax     = AimConfig.vMax
        val omegaMax = vMax / (flywheelRadius * AimConfig.launchEfficiency)
        val bounds = floatArrayOf(
            (-PI).toFloat(), PI.toFloat(),
            Math.toRadians(ShooterConfig.minAngleDeg).toFloat(),
            Math.toRadians(ShooterConfig.maxAngleDeg).toFloat(),
            vMax.toFloat(),
            omegaMax.toFloat()
        )

        val weights = floatArrayOf(
            ShotSolverConfig.wTheta.toFloat(),
            ShotSolverConfig.wPhi.toFloat(),
            ShotSolverConfig.wOmega.toFloat()
        )

        // omega(phi, vExit) = c1 * vExit,  c1 = 1 / (radius * efficiency)
        val c1 = (1.0 / (flywheelRadius * AimConfig.launchEfficiency)).toFloat()
        val omegaCoeffs = floatArrayOf(0f, c1, 0f, 0f, 0f, 0f)

        // ── zone configs ──────────────────────────────────────────────────────
        // Zone 1: top wall (+y), half-plane n=(0,1), d = FIELD_HALF − ZONE1_HALF_HYP
        val zone1Config = floatArrayOf(
            /* nx */       0f,
            /* ny */       1f,
            /* d  */       FIELD_HALF - ZONE1_HALF_HYP,
            /* Q_process*/ Q_PROCESS,
            /* p_low */    P_LOW,
            /* p_high */   P_HIGH,
            /* alpha_lpf*/ ALPHA_LPF,
            /* omega_idle*/OMEGA_IDLE
        )
        // Zone 2: bottom wall (−y), half-plane n=(0,−1), d = FIELD_HALF − ZONE2_HALF_HYP
        val zone2Config = floatArrayOf(
            /* nx */       0f,
            /* ny */       -1f,
            /* d  */       FIELD_HALF - ZONE2_HALF_HYP,
            /* Q_process*/ Q_PROCESS,
            /* p_low */    P_LOW,
            /* p_high */   P_HIGH,
            /* alpha_lpf*/ ALPHA_LPF,
            /* omega_idle*/OMEGA_IDLE
        )

        val zone1Handle = bridge.createZoneTracker(zone1Config, weights, bounds, physConfig, omegaCoeffs)
        val zone2Handle = bridge.createZoneTracker(zone2Config, weights, bounds, physConfig, omegaCoeffs)

        // ── goal ──────────────────────────────────────────────────────────────
        val goal = FieldLandmarks.goalPosition3d(BLUE_ALLIANCE, AimConfig.goalHeight)

        // ── runtime state ─────────────────────────────────────────────────────
        var applyTargets = false   // A button
        var lastBtnA = false
        var lastBtnB = false
        var lastBtnX = false

        // Which output drives the applied targets
        enum class TargetSource { ZONE1, ZONE2, POINT }
        var targetSource = TargetSource.ZONE1

        // Warm-start T* for point solver
        var pointTStar: Float? = null

        robot.init(Pose2d(), apriltagTracking = false)

        telemetry.addLine("Turret Planner Debug ready.")
        telemetry.addLine("A = toggle apply | B = cycle source | X = reset zones")
        telemetry.update()
        waitForStart()
        if (isStopRequested) return

        ioLoop { _, dt ->

            // ── gamepad ───────────────────────────────────────────────────────
            robot.drive.update(gamepad1, io)

            val pressA = gamepad1.a && !lastBtnA
            val pressB = gamepad1.b && !lastBtnB
            val pressX = gamepad1.x && !lastBtnX
            lastBtnA = gamepad1.a
            lastBtnB = gamepad1.b
            lastBtnX = gamepad1.x

            if (pressA) applyTargets = !applyTargets
            if (pressB) targetSource = when (targetSource) {
                TargetSource.ZONE1 -> TargetSource.ZONE2
                TargetSource.ZONE2 -> TargetSource.POINT
                TargetSource.POINT -> TargetSource.ZONE1
            }
            if (pressX) {
                bridge.resetZoneTracker(zone1Handle)
                bridge.resetZoneTracker(zone2Handle)
                pointTStar = null
            }

            // ── pull state from hardware / GTSAM ──────────────────────────────
            // Robot.update() runs vision + GTSAM but does NOT override targets
            // (aimTurret=false, aimFlywheel=false).
            robot.update()

            val fusedPose = robot.aim.autoAim.fusedPose
            val odoVel    = io.velocity()

            // Robot velocity in field frame (rotate odo-frame vel by heading correction)
            val dRot  = fusedPose.rot - io.position().rot
            val cosDR = Math.cos(dRot)
            val sinDR = Math.sin(dRot)
            val vRobX  = odoVel.v.x
            val vRobY  = odoVel.v.y
            val vFieldX = (vRobX * cosDR - vRobY * sinDR).toFloat()
            val vFieldY = (vRobX * sinDR + vRobY * cosDR).toFloat()
            val omegaRobot = odoVel.rot.toFloat()

            val robotX  = fusedPose.v.x.toFloat()
            val robotY  = fusedPose.v.y.toFloat()
            val heading = fusedPose.rot.toFloat()

            // Turret state in field frame
            val turretTheta = (robot.turret.pos + fusedPose.rot).toFloat()
            val turretPhi   = robot.shooter.computedHoodAngle.toFloat()
            val turretOmega = io.flywheelVelocity().toFloat()

            // Goal
            val gX = goal.x.toFloat()
            val gY = goal.y.toFloat()
            val gZ = goal.z.toFloat()

            // ── Zone 1 update ─────────────────────────────────────────────────
            val z1 = bridge.updateZoneTracker(
                zone1Handle,
                robotX, robotY, heading,
                vFieldX, vFieldY, omegaRobot,
                turretTheta, turretPhi, turretOmega,
                gX, gY, gZ,
                dt.inWholeMilliseconds.toFloat() / 1000f
            )
            // z0 for zone1: n·p − d = (0)(x) + (1)(y) − d = y − d
            val z1_z0 = robotY - (FIELD_HALF - ZONE1_HALF_HYP)

            // ── Zone 2 update ─────────────────────────────────────────────────
            val z2 = bridge.updateZoneTracker(
                zone2Handle,
                robotX, robotY, heading,
                vFieldX, vFieldY, omegaRobot,
                turretTheta, turretPhi, turretOmega,
                gX, gY, gZ,
                dt.inWholeMilliseconds.toFloat() / 1000f
            )
            // z0 for zone2: n·p − d = (0)(x) + (−1)(y) − d = −y − d
            val z2_z0 = -robotY - (FIELD_HALF - ZONE2_HALF_HYP)

            // ── Point solver ──────────────────────────────────────────────────
            val tZ = turretPos.z.toFloat()
            val pointResult: FloatArray = try {
                val tStar = pointTStar
                if (tStar != null) {
                    bridge.optimalTWarm(
                        robotX, robotY, tZ,
                        gX, gY, gZ,
                        vFieldX, vFieldY,
                        tStar,
                        turretTheta, turretPhi, turretOmega,
                        weights, bounds, physConfig, omegaCoeffs
                    )
                } else {
                    bridge.optimalTCold(
                        robotX, robotY, tZ,
                        gX, gY, gZ,
                        vFieldX, vFieldY,
                        turretTheta, turretPhi, turretOmega,
                        weights, bounds, physConfig, omegaCoeffs
                    )
                }
            } catch (e: Exception) {
                floatArrayOf(0f, 0f, turretTheta, turretPhi, 0f, turretOmega, 0f)
            }
            val pointFeasible = pointResult[OptimalTIdx.FEASIBLE] > 0.5f
            if (pointFeasible) {
                pointTStar = pointResult[OptimalTIdx.T_STAR]
            } else {
                pointTStar = null
            }

            // ── Select active target ──────────────────────────────────────────
            val activeTheta: Float
            val activePhi: Float
            val activeOmega: Float
            val activeShouldFire: Boolean
            when (targetSource) {
                TargetSource.ZONE1 -> {
                    activeTheta     = z1[ZoneTrackerIdx.TARGET_THETA]
                    activePhi       = z1[ZoneTrackerIdx.TARGET_PHI]
                    activeOmega     = z1[ZoneTrackerIdx.TARGET_OMEGA]
                    activeShouldFire= z1[ZoneTrackerIdx.SHOULD_FIRE] > 0.5f
                }
                TargetSource.ZONE2 -> {
                    activeTheta     = z2[ZoneTrackerIdx.TARGET_THETA]
                    activePhi       = z2[ZoneTrackerIdx.TARGET_PHI]
                    activeOmega     = z2[ZoneTrackerIdx.TARGET_OMEGA]
                    activeShouldFire= z2[ZoneTrackerIdx.SHOULD_FIRE] > 0.5f
                }
                TargetSource.POINT -> {
                    activeTheta     = pointResult[OptimalTIdx.THETA]
                    activePhi       = pointResult[OptimalTIdx.PHI]
                    activeOmega     = pointResult[OptimalTIdx.OMEGA]
                    activeShouldFire= pointFeasible
                }
            }

            // ── Apply targets to hardware (when not dry-running) ──────────────
            if (applyTargets) {
                robot.turret.fieldRelativeMode = true
                robot.turret.fieldTargetAngle  = activeTheta.toDouble()
                robot.shooter.hoodAngle        = activePhi.toDouble()
                robot.shooter.flywheelTarget   = activeOmega.toDouble()
            }

            // ── Telemetry ─────────────────────────────────────────────────────

            // ── Robot state ───────────────────────────────────────────────────
            telemetry.addLine("══ ROBOT STATE ══")
            telemetry.addData("Fused pose",   "x=%.3f  y=%.3f  θ=%.1f°",
                fusedPose.v.x, fusedPose.v.y, Math.toDegrees(fusedPose.rot))
            telemetry.addData("Odometry",     "x=%.3f  y=%.3f  θ=%.1f°",
                io.position().v.x, io.position().v.y, Math.toDegrees(io.position().rot))
            telemetry.addData("Field vel",    "vx=%.2f  vy=%.2f  ω=%.2f rad/s",
                vFieldX, vFieldY, omegaRobot)
            telemetry.addData("Vision",       if (robot.aim.autoAim.hasVisionTarget) "LOCKED" else "searching")
            telemetry.addData("Goal dist",    "%.2f m",
                hypot(goal.x - fusedPose.v.x, goal.y - fusedPose.v.y))
            telemetry.addData("Box",          "x=%.2f/%.2f  y=%.2f/%.2f",
                -BOX_X, BOX_X, -BOX_Y, BOX_Y)

            // ── Zone 1 (large, top wall) ──────────────────────────────────────
            telemetry.addLine("")
            telemetry.addLine("══ ZONE 1  (top +y,  half-hyp=${ZONE1_HALF_HYP}m,  depth=${ZONE1_HALF_HYP}m) ══")
            telemetry.addData("z₀ (dist to zone)", "%.3f m  (%s)",
                z1_z0, if (z1_z0 >= 0f) "INSIDE" else "outside")
            telemetry.addData("Urgency raw",     "%.3f", z1[ZoneTrackerIdx.URGENCY])
            telemetry.addData("Urgency filtered","%.3f", z1[ZoneTrackerIdx.URGENCY_FILTERED])
            telemetry.addData("Effort",          "%.3f  [0=idle → 1=full prep]", z1[ZoneTrackerIdx.EFFORT])
            telemetry.addData("τ prep",          "%.3f s", z1[ZoneTrackerIdx.TAU])
            telemetry.addData("Target θ",        "%.1f°", Math.toDegrees(z1[ZoneTrackerIdx.TARGET_THETA].toDouble()))
            telemetry.addData("Target φ",        "%.1f°", Math.toDegrees(z1[ZoneTrackerIdx.TARGET_PHI].toDouble()))
            telemetry.addData("Target ω",        "%.0f rad/s  (%.0f rpm)",
                z1[ZoneTrackerIdx.TARGET_OMEGA],
                z1[ZoneTrackerIdx.TARGET_OMEGA] * 60.0 / (2.0 * PI))
            telemetry.addData("Should fire",     if (z1[ZoneTrackerIdx.SHOULD_FIRE] > 0.5f) "FIRE" else "hold")

            // ── Zone 2 (small, bottom wall) ───────────────────────────────────
            telemetry.addLine("")
            telemetry.addLine("══ ZONE 2  (bottom −y,  half-hyp=${ZONE2_HALF_HYP}m,  depth=${ZONE2_HALF_HYP}m) ══")
            telemetry.addData("z₀ (dist to zone)", "%.3f m  (%s)",
                z2_z0, if (z2_z0 >= 0f) "INSIDE" else "outside")
            telemetry.addData("Urgency raw",     "%.3f", z2[ZoneTrackerIdx.URGENCY])
            telemetry.addData("Urgency filtered","%.3f", z2[ZoneTrackerIdx.URGENCY_FILTERED])
            telemetry.addData("Effort",          "%.3f", z2[ZoneTrackerIdx.EFFORT])
            telemetry.addData("τ prep",          "%.3f s", z2[ZoneTrackerIdx.TAU])
            telemetry.addData("Target θ",        "%.1f°", Math.toDegrees(z2[ZoneTrackerIdx.TARGET_THETA].toDouble()))
            telemetry.addData("Target φ",        "%.1f°", Math.toDegrees(z2[ZoneTrackerIdx.TARGET_PHI].toDouble()))
            telemetry.addData("Target ω",        "%.0f rad/s  (%.0f rpm)",
                z2[ZoneTrackerIdx.TARGET_OMEGA],
                z2[ZoneTrackerIdx.TARGET_OMEGA] * 60.0 / (2.0 * PI))
            telemetry.addData("Should fire",     if (z2[ZoneTrackerIdx.SHOULD_FIRE] > 0.5f) "FIRE" else "hold")

            // ── Point solver ──────────────────────────────────────────────────
            telemetry.addLine("")
            telemetry.addLine("══ POINT SOLVER ══")
            telemetry.addData("Feasible",  if (pointFeasible) "YES" else "no")
            telemetry.addData("T*",        "%.3f s", pointResult[OptimalTIdx.T_STAR])
            telemetry.addData("τ (move time)", "%.3f s", pointResult[OptimalTIdx.TAU])
            telemetry.addData("θ",         "%.1f°",  Math.toDegrees(pointResult[OptimalTIdx.THETA].toDouble()))
            telemetry.addData("φ",         "%.1f°",  Math.toDegrees(pointResult[OptimalTIdx.PHI].toDouble()))
            telemetry.addData("v_exit",    "%.2f m/s", pointResult[OptimalTIdx.V_EXIT])
            telemetry.addData("ω",         "%.0f rad/s  (%.0f rpm)",
                pointResult[OptimalTIdx.OMEGA],
                pointResult[OptimalTIdx.OMEGA] * 60.0 / (2.0 * PI))
            telemetry.addData("Warm-start","T* = ${if (pointTStar != null) "%.3f s".format(pointTStar) else "cold (searching)"}")

            // ── Active target & mode ──────────────────────────────────────────
            telemetry.addLine("")
            telemetry.addLine("══ ACTIVE TARGET ══")
            telemetry.addData("Source",    "${targetSource.name}  (B to cycle)")
            telemetry.addData("Mode",      if (applyTargets) "LIVE — writing hardware" else "DRY RUN — log only  (A to arm)")
            telemetry.addData("θ target",  "%.1f°",  Math.toDegrees(activeTheta.toDouble()))
            telemetry.addData("φ target",  "%.1f°",  Math.toDegrees(activePhi.toDouble()))
            telemetry.addData("ω target",  "%.0f rad/s  (%.0f rpm)",
                activeOmega,
                activeOmega * 60.0 / (2.0 * PI))
            telemetry.addData("Should fire", if (activeShouldFire) "FIRE" else "hold")

            // ── Subsystem state & error ───────────────────────────────────────
            telemetry.addLine("")
            telemetry.addLine("══ SUBSYSTEM STATE ══")
            val currentTheta = turretTheta.toDouble()
            val currentPhi   = turretPhi.toDouble()
            val currentOmega = turretOmega.toDouble()
            val errTheta = Math.toDegrees(wrapAngle(currentTheta - activeTheta.toDouble()))
            val errPhi   = Math.toDegrees(currentPhi   - activePhi)
            val errOmega = currentOmega - activeOmega
            telemetry.addData("Turret θ",  "%.1f°  (err %+.1f°)",
                Math.toDegrees(currentTheta), errTheta)
            telemetry.addData("Hood φ",    "%.1f°  (err %+.1f°)",
                Math.toDegrees(currentPhi), errPhi)
            telemetry.addData("Flywheel ω","%.0f rad/s  (err %+.0f)",
                currentOmega, errOmega)
            telemetry.addData("Aligned?", if (
                abs(errTheta) < 3.0 && abs(errPhi) < 3.0 && abs(errOmega) < 50.0
            ) "YES" else "no")

            telemetry.addLine("")
            telemetry.addData("Δt", "%.1f ms", dt.inWholeMilliseconds.toFloat())

            telemetry.update()
            false
        }

        // ── cleanup ───────────────────────────────────────────────────────────
        bridge.destroyZoneTracker(zone1Handle)
        bridge.destroyZoneTracker(zone2Handle)
        robot.close()
    }

    private fun wrapAngle(a: Double): Double {
        var r = a % (2.0 * PI)
        if (r >  PI) r -= 2.0 * PI
        if (r < -PI) r += 2.0 * PI
        return r
    }
}
