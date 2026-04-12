package sigmacorns.logic

import com.qualcomm.robotcore.hardware.Gamepad
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import kotlinx.coroutines.yield
import org.joml.Vector2d
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.robotSize
import sigmacorns.math.Pose2d
import sigmacorns.math.closestPointOnConvexPolygon
import sigmacorns.math.normalizeAngle
import sigmacorns.sim.MecanumState
import sigmacorns.subsystem.IntakeTransfer
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.max
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

class AutomationManager(val robot: Robot) {
    companion object {
        private val Double.f get() = "%.3f".format(this)
    }
    var hasControl: Boolean = false
    var curBehavior: Job? = null

    fun update(gamepad: Gamepad): Boolean {
        val lx = gamepad.left_stick_x
        val ly = gamepad.left_stick_y
        val rx = gamepad.right_stick_x

        if(max(hypot(lx,ly),rx.absoluteValue) > 0.1 || gamepad.left_stick_button || gamepad.right_stick_button) {
            hasControl = false
        }

        if(!hasControl) {
            curBehavior?.cancel()
            curBehavior = null
            return hasControl
        }

        if(curBehavior?.isCompleted == true) {
            curBehavior = null
            hasControl = false
        }

        return hasControl
    }

    private val MIN_FAR_ZONE_Y = -FieldLandmarks.fieldHalfExtend + max(robotSize.x,robotSize.y)


    fun shootFarZone() {
        if(curBehavior != null) return

        curBehavior = robot.scope.launch {
            val p = closestPointOnConvexPolygon(FieldLandmarks.farZoneCorners, robot.io.position().v)
            val curTheta = robot.io.position().rot
            p.y = max(p.y, MIN_FAR_ZONE_Y)

            // bc this is far zone bounds will always be in -PI to PI
            val goalAngle = p.angle(FieldLandmarks.goalPosition(robot.blue))
            val headingRange = (robot.turret.angleLimits.start*0.8 + goalAngle)..(robot.turret.angleLimits.endInclusive*0.8 + goalAngle)

            val errStart = normalizeAngle(headingRange.start-curTheta).absoluteValue
            val errEnd = normalizeAngle(headingRange.endInclusive-curTheta).absoluteValue

            val theta = if(headingRange.contains(curTheta)) {
                curTheta
            } else {
                if(errStart < errEnd) headingRange.start else headingRange.endInclusive
            }

            val target = MecanumState(Pose2d(),Pose2d(p,theta))
            println("[NativeAutoAim] " + "shootFarZone: target=(${p.x.f},${p.y.f}) heading=${Math.toDegrees(theta).f}°")

            val beforeAutoShoot = robot.intakeCoordinator.autoShootEnabled
            val beforeAimFlywheel = robot.aimFlywheel
            val beforeAimTurret = robot.aimTurret
            val beforeFieldCentric = robot.drive.fieldCentric
            robot.intakeCoordinator.autoShootEnabled = true
            robot.aimFlywheel = true
            robot.aimTurret = true
            robot.drive.fieldCentric = false
            val curPos = robot.io.position().v
            val curVel = robot.io.velocity().v
            val dist = hypot(p.x - curPos.x, p.y - curPos.y)
            val speed = hypot(curVel.x, curVel.y).coerceAtLeast(0.5)
            val timeToArrival = (dist / speed).seconds
            println("[NativeAutoAim] " + "shootFarZone: dist=${dist.f}m speed=${speed.f}m/s eta=${timeToArrival.inWholeMilliseconds}ms")

            var holding = false
            var pathingUpdatedEta = false
            lateinit var pathing: Job

            try {
                pathing = launch {
                    pathingUpdatedEta = true
                    robot.ltv.runWaypointToCompletion(target,5.seconds,robot.io)
                    println("[NativeAutoAim] " + "shootFarZone: waypoint reached, now holding")
                    holding = true
                    while (true) {
                        robot.ltv.holdPos(robot.io,target.pos)
                        yield()
                    }
                }
                while (robot.beamBreak.ballCount > 0) {
                    // Compute ETA for NativeAutoAim's trajectory interpolation.
                    // prevWaypointEta() can collapse to 0 if the solver's horizon
                    // shrinks; fall back to distance/velocity when that happens
                    // to prevent NativeAutoAim from aiming at a phantom position.
                    val solverEta = robot.ltv.prevWaypointEta()
                    val pos = robot.io.position().v
                    val d = hypot(target.pos.v.x - pos.x, target.pos.v.y - pos.y)
                    val distEta = (d / hypot(robot.io.velocity().v.x, robot.io.velocity().v.y).coerceAtLeast(0.5)).seconds
                    val eta = when {
                        holding -> 0.seconds
                        !pathingUpdatedEta -> timeToArrival
                        d > 0.1 -> maxOf(solverEta, distEta)
                        else -> solverEta
                    }
                    robot.aim.plannedShot = PlannedShot(target, eta)
                    println("[NativeAutoAim] " + "shootFarZone: balls=${robot.beamBreak.ballCount} holding=$holding eta=${eta.inWholeMilliseconds}ms inZone=${robot.intakeCoordinator.inShootingZone} readyToShoot=${robot.aim.readyToShoot}")
                    yield()
                }
                println("[NativeAutoAim] " + "shootFarZone: all balls fired")
            } finally {
                pathing.cancelAndJoin()
                robot.aim.plannedShot = null
                robot.intakeCoordinator.autoShootEnabled = beforeAutoShoot
                robot.aimFlywheel = beforeAimFlywheel
                robot.aimTurret = beforeAimTurret
                robot.drive.fieldCentric = beforeFieldCentric
                // Zero drive commands — the cancelled pathing coroutine's final
                // iteration writes stale LTV outputs after manual drive has already
                // written to IO, so overwrite them to prevent one frame of stale drive.
                robot.io.driveFL = 0.0
                robot.io.driveBL = 0.0
                robot.io.driveBR = 0.0
                robot.io.driveFR = 0.0
            }
        }

        hasControl = true
    }

    fun intakeGate(timeout: Duration = Duration.INFINITE) {
        if(curBehavior != null) return

        curBehavior = robot.scope.launch {
            val beforeFieldCentric = robot.drive.fieldCentric
            robot.drive.fieldCentric = false
            val p = FieldLandmarks.gateIntakePosition(robot.blue)
            val target = MecanumState(Pose2d(),p)

            try {
                robot.ltv.runWaypointToCompletion(target, 5.seconds, robot.io, posTol = 0.1)
                robot.intakeTransfer.state = IntakeTransfer.State.INTAKING

                withTimeout(timeout) {
                    while (!robot.beamBreak.isFull) {
                        robot.ltv.holdPos(robot.io, p)
                        yield()
                    }
                }
            } finally {
                robot.intakeTransfer.state = IntakeTransfer.State.IDLE
                robot.drive.fieldCentric = beforeFieldCentric
                robot.io.driveFL = 0.0
                robot.io.driveBL = 0.0
                robot.io.driveBR = 0.0
                robot.io.driveFR = 0.0
            }
        }

        hasControl = true
    }
}