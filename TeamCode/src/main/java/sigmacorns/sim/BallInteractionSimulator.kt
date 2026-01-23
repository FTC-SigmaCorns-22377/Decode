package sigmacorns.sim

import org.joml.Vector3d
import sigmacorns.math.Pose2d
import kotlin.math.*
import kotlin.time.Duration

/**
 * Represents a ball held in the spindexer (not in Drake physics).
 */
data class SpindexerBall(val color: Balls)

/**
 * Simulates ball interaction with intake, spindexer, and shooter.
 * Infers all state from IO values without depending on SpindexerLogic.
 *
 * Slots are body-relative: slots[i] always refers to the same physical slot
 * on the spindexer, regardless of rotation. The current spindexer angle is used
 * to determine which slot is at the intake or shoot position.
 */
class BallInteractionSimulator(private val model: DrakeRobotModel) {

    // Spindexer slots (3 slots, body-relative: index always refers to same physical slot)
    private val slots: Array<SpindexerBall?> = arrayOfNulls(3)

    // Cached spindexer angle for color sensor queries
    private var currentSpindexerAngle = 0.0

    /**
     * Main update function called each simulation step.
     */
    fun update(
        robotPose: Pose2d,
        robotVelocity: Pose2d,
        intakePower: Double,
        transferPower: Double,
        spindexerAngle: Double,
        turretYaw: Double,
        hoodPitch: Double,
        flywheelOmega: Double,
        dt: Duration
    ) {
        currentSpindexerAngle = spindexerAngle

        // 1. Check for intake capture - find which slot is at the intake position
        val (intakeSlot, intakeError) = getSlotNearAngle(spindexerAngle, INTAKE_ANGLE)
        if (abs(intakePower) > INTAKE_POWER_THRESHOLD &&
            abs(intakeError) < SLOT_ALIGNMENT_TOLERANCE &&
            slots[intakeSlot] == null) {
            checkIntakeCapture(robotPose, intakeSlot)
        }

        // 2. Check for shooting - find which slot is at the shoot position
        val (shootSlot, shootError) = getSlotNearAngle(spindexerAngle, SHOOT_ANGLE)
        if (transferPower > TRANSFER_POWER_THRESHOLD &&
            abs(shootError) < SLOT_ALIGNMENT_TOLERANCE &&
            slots[shootSlot] != null) {
            shootBall(robotPose, robotVelocity, turretYaw, hoodPitch, flywheelOmega, shootSlot)
        }
    }

    /**
     * Finds which slot is closest to a target angle in the robot frame.
     * Returns the slot index and the alignment error (how far the slot is from the target).
     */
    private fun getSlotNearAngle(spindexerAngle: Double, targetAngle: Double): Pair<Int, Double> {
        var bestSlot = 0
        var bestError = Double.MAX_VALUE
        for (i in 0 until 3) {
            val slotRobotAngle = spindexerAngle + i * SPINDEXER_SLOT_ANGLE
            var error = (slotRobotAngle - targetAngle) % (2 * PI)
            if (error > PI) error -= 2 * PI
            if (error < -PI) error += 2 * PI
            if (abs(error) < abs(bestError)) {
                bestError = error
                bestSlot = i
            }
        }
        return Pair(bestSlot, bestError)
    }

    private fun checkIntakeCapture(robotPose: Pose2d, slotIndex: Int) {
        // Find closest ball within rectangular capture zone
        var closestIndex = -1
        var closestDist = Double.MAX_VALUE

        val cos = cos(robotPose.rot)
        val sin = sin(robotPose.rot)

        model.ballPositions.forEachIndexed { i, pos ->
            // Transform ball position to robot-relative frame
            val worldDx = pos.x - robotPose.v.x
            val worldDy = pos.y - robotPose.v.y
            val robotRelativeX = worldDx * cos + worldDy * sin
            val robotRelativeY = -worldDx * sin + worldDy * cos

            // Calculate deviation from intake position
            val xDeviation = abs(robotRelativeX - INTAKE_X_OFFSET)
            val yDeviation = abs(robotRelativeY)

            // Check if ball is within rectangular intake zone and at reasonable height
            if (xDeviation < INTAKE_X_TOLERANCE &&
                yDeviation < INTAKE_Y_TOLERANCE &&
                pos.z < BALL_RADIUS * 3) {
                // Use distance for prioritization when multiple balls are in zone
                val dist = sqrt(xDeviation * xDeviation + yDeviation * yDeviation)
                if (dist < closestDist) {
                    closestDist = dist
                    closestIndex = i
                }
            }
        }

        if (closestIndex >= 0) {
            // Capture ball: store color, remove from Drake
            val color = model.ballColors.getOrElse(closestIndex) { Balls.Green }
            slots[slotIndex] = SpindexerBall(color)
            model.removeBall(closestIndex)
        }
    }

    private fun shootBall(
        robotPose: Pose2d,
        robotVelocity: Pose2d,
        turretYaw: Double,
        hoodPitch: Double,
        flywheelOmega: Double,
        slotIndex: Int
    ) {
        // Calculate exit speed from flywheel
        val exitSpeed = abs(flywheelOmega) * FLYWHEEL_RADIUS * FLYWHEEL_EFFICIENCY
        if (exitSpeed < 0.5) return  // Flywheel not spinning fast enough

        val ball = slots[slotIndex] ?: return
        slots[slotIndex] = null

        // World yaw = robot yaw + turret yaw
        val worldYaw = robotPose.rot + turretYaw

        // Ball velocity components (ball frame -> world frame)
        val horizSpeed = exitSpeed * cos(hoodPitch)
        val vertSpeed = exitSpeed * sin(hoodPitch)

        val ballVelX = horizSpeed * cos(worldYaw)
        val ballVelY = horizSpeed * sin(worldYaw)
        val ballVelZ = vertSpeed

        // Add robot velocity to ball velocity
        val totalVelX = ballVelX + robotVelocity.v.x
        val totalVelY = ballVelY + robotVelocity.v.y
        val totalVelZ = ballVelZ  // Assume robot Z velocity is 0

        // Calculate spawn position (offset from turret to avoid collision)
        // Turret is at (-0.039962, 0, 0.22744) relative to base
        // Flywheel is at (0.05, 0, 0.05) relative to turret
        val turretBase = transformToWorld(robotPose, -0.039962 + 0.05, 0.0, 0.22744 + 0.05)

        // Offset in direction of shot to clear turret
        val spawnX = turretBase.x + SHOT_SPAWN_OFFSET * cos(worldYaw)
        val spawnY = turretBase.y + SHOT_SPAWN_OFFSET * sin(worldYaw)
        val spawnZ = turretBase.z + SHOT_SPAWN_OFFSET * sin(hoodPitch)

        // Spawn ball in Drake with velocity
        model.spawnBallWithVelocity(spawnX, spawnY, spawnZ, totalVelX, totalVelY, totalVelZ, ball.color)
    }

    private fun transformToWorld(pose: Pose2d, localX: Double, localY: Double, localZ: Double): Vector3d {
        val cos = cos(pose.rot)
        val sin = sin(pose.rot)
        return Vector3d(
            pose.v.x + localX * cos - localY * sin,
            pose.v.y + localX * sin + localY * cos,
            localZ
        )
    }

    // Color sensor API - reads whichever slot is closest to the intake position
    fun colorSensorDetectsBall(): Boolean {
        val (slot, _) = getSlotNearAngle(currentSpindexerAngle, INTAKE_ANGLE)
        return slots[slot] != null
    }
    fun colorSensorGetBallColor(): Balls? {
        val (slot, _) = getSlotNearAngle(currentSpindexerAngle, INTAKE_ANGLE)
        return slots[slot]?.color
    }

    // For visualization: get world positions of balls in spindexer
    fun getSpindexerBallPositions(robotPose: Pose2d, spindexerAngle: Double): List<Pair<Vector3d, Balls>> {
        val result = mutableListOf<Pair<Vector3d, Balls>>()
        val spindexerCenter = transformToWorld(robotPose, 0.0, 0.0, 0.08)

        slots.forEachIndexed { i, ball ->
            if (ball != null) {
                // Each slot is 120 degrees apart, slot 0 at angle 0 (relative to robot forward)
                val slotAngle = robotPose.rot + spindexerAngle + i * SPINDEXER_SLOT_ANGLE
                val slotRadius = 0.10  // Ball sits at this radius from spindexer center
                val pos = Vector3d(
                    spindexerCenter.x + slotRadius * cos(slotAngle),
                    spindexerCenter.y + slotRadius * sin(slotAngle),
                    spindexerCenter.z
                )
                result.add(pos to ball.color)
            }
        }
        return result
    }

    // Get the number of balls currently in the spindexer
    fun getBallCount(): Int = slots.count { it != null }

    // Get slot contents for debugging/visualization
    fun getSlots(): Array<SpindexerBall?> = slots.copyOf()

    companion object {
        private const val INTAKE_X_OFFSET = 0.23
        private const val INTAKE_X_TOLERANCE = 0.04 // Forward tolerance
        private const val INTAKE_Y_TOLERANCE = 0.1  // Left/right tolerance
        private const val INTAKE_POWER_THRESHOLD = 0.1
        private const val SPINDEXER_SLOT_ANGLE = 2 * PI / 3
        private const val SLOT_ALIGNMENT_TOLERANCE = 0.15
        private const val TRANSFER_POWER_THRESHOLD = 0.5
        private const val FLYWHEEL_RADIUS = 0.048
        private const val BALL_RADIUS = 0.05
        private const val SHOT_SPAWN_OFFSET = 0.15
        private const val FLYWHEEL_EFFICIENCY = 0.24

        // Fixed robot-frame angles for intake and shoot positions
        private const val INTAKE_ANGLE = 0.0
        private const val SHOOT_ANGLE = PI / 3  // 60 degrees from intake
    }
}
