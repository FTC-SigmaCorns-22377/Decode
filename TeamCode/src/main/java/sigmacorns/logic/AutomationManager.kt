package sigmacorns.logic

import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import sigmacorns.Robot
import sigmacorns.constants.FieldLandmarks
import sigmacorns.constants.robotSize
import sigmacorns.math.Pose2d
import sigmacorns.math.closestPointOnConvexPolygon
import sigmacorns.math.normalizeAngle
import sigmacorns.sim.MecanumState
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.time.Duration.Companion.seconds

class AutomationManager() {
    var hasControl: Boolean = false
    var curBehavior: Job? = null

    suspend fun update() {
        if(!hasControl) {
            curBehavior?.cancelAndJoin()
            curBehavior = null
            return
        }

        if(curBehavior?.isCompleted == true) {
            curBehavior = null
            hasControl = false
        }
    }

    private val MIN_FAR_ZONE_Y = -FieldLandmarks.fieldHalfExtend + max(robotSize.x,robotSize.y)

    fun shootFarZone(robot: Robot) {
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

            val beforeAutoShoot = robot.intakeCoordinator.autoShootEnabled
            val beforeAimFlywheel = robot.aimFlywheel
            val beforeAimTurret = robot.aimTurret
            robot.intakeCoordinator.autoShootEnabled = true
            robot.aimFlywheel = true
            robot.aimTurret = true

            val pathing = launch {
                robot.ltv.runWaypointToCompletion(target,5.seconds,robot.io)
                while (true) {
                    robot.ltv.holdPos(robot.io,target.pos)
                    yield()
                }
            }

            try {
                while (robot.beamBreak.ballCount>0) yield()
            } finally {
                pathing.cancelAndJoin()
                robot.intakeCoordinator.autoShootEnabled = beforeAutoShoot
                robot.aimFlywheel = beforeAimFlywheel
                robot.aimTurret = beforeAimTurret
            }
        }
    }
}