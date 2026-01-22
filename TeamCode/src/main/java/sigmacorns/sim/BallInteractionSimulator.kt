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
 */
class BallInteractionSimulator(private val model: DrakeRobotModel) {

    // Spindexer slots (3 slots, index 0 faces intake direction)
    private val slots: Array<SpindexerBall?> = arrayOfNulls(3)

    // Track cumulative spindexer rotation for slot shifting
    private var lastSpindexerAngle = 0.0
    private var accumulatedRotation = 0.0
    private var initialized = false

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
        // Initialize tracking on first call
        if (!initialized) {
            lastSpindexerAngle = spindexerAngle
            initialized = true
        }

        // 1. Track spindexer rotation and shift slots when crossing 120 degree boundaries
        updateSlotRotation(spindexerAngle)

        // 2. Check for intake capture
        if (abs(intakePower) > INTAKE_POWER_THRESHOLD && slots[0] == null) {
            if (isInIntakeMode(spindexerAngle)) {
                checkIntakeCapture(robotPose)
            }
        }

        // 3. Check for shooting
        if (transferPower > TRANSFER_POWER_THRESHOLD && slots[0] != null) {
            if (isInShootingMode(spindexerAngle)) {
                shootBall(robotPose, robotVelocity, turretYaw, hoodPitch, flywheelOmega)
            }
        }
    }

    private fun updateSlotRotation(currentAngle: Double) {
        val delta = currentAngle - lastSpindexerAngle
        accumulatedRotation += delta
        lastSpindexerAngle = currentAngle

        // When accumulated rotation crosses 120 degrees (or -120 degrees), shift slots
        while (accumulatedRotation >= SPINDEXER_SLOT_ANGLE) {
            accumulatedRotation -= SPINDEXER_SLOT_ANGLE
            // Clockwise: slots shift 2->1->0->2
            val temp = slots[0]
            slots[0] = slots[2]
            slots[2] = slots[1]
            slots[1] = temp
        }
        while (accumulatedRotation <= -SPINDEXER_SLOT_ANGLE) {
            accumulatedRotation += SPINDEXER_SLOT_ANGLE
            // Counter-clockwise: slots shift 0->1->2->0
            val temp = slots[0]
            slots[0] = slots[1]
            slots[1] = slots[2]
            slots[2] = temp
        }
    }

    private fun isInIntakeMode(angle: Double): Boolean {
        val normalized = ((angle % (2 * PI)) + (2 * PI)) % (2 * PI)
        val cycleAngle = normalized % SPINDEXER_SLOT_ANGLE
        return cycleAngle < SLOT_ALIGNMENT_TOLERANCE ||
               cycleAngle > SPINDEXER_SLOT_ANGLE - SLOT_ALIGNMENT_TOLERANCE
    }

    private fun isInShootingMode(angle: Double): Boolean {
        val normalized = ((angle % (2 * PI)) + (2 * PI)) % (2 * PI)
        val cycleAngle = normalized % SPINDEXER_SLOT_ANGLE
        val shootOffset = MODE_CHANGE_ANGLE  // 60 degrees
        return abs(cycleAngle - shootOffset) < SLOT_ALIGNMENT_TOLERANCE
    }

    private fun checkIntakeCapture(robotPose: Pose2d) {
        // Transform intake position to world frame
        val intakeWorld = transformToWorld(robotPose, INTAKE_X_OFFSET, 0.0, INTAKE_Z_OFFSET)

        // Find closest ball within capture range
        var closestIndex = -1
        var closestDist = INTAKE_CAPTURE_DISTANCE

        model.ballPositions.forEachIndexed { i, pos ->
            val dx = pos.x - intakeWorld.x
            val dy = pos.y - intakeWorld.y
            val dist = sqrt(dx * dx + dy * dy)
            // Check ball is at reasonable height (ground level)
            if (dist < closestDist && pos.z < BALL_RADIUS * 3) {
                closestDist = dist
                closestIndex = i
            }
        }

        if (closestIndex >= 0) {
            // Capture ball: store color, remove from Drake
            val color = model.ballColors.getOrElse(closestIndex) { Balls.Green }
            slots[0] = SpindexerBall(color)
            model.removeBall(closestIndex)
        }
    }

    private fun shootBall(
        robotPose: Pose2d,
        robotVelocity: Pose2d,
        turretYaw: Double,
        hoodPitch: Double,
        flywheelOmega: Double
    ) {
        val ball = slots[0] ?: return
        slots[0] = null

        // Calculate exit speed from flywheel
        val exitSpeed = abs(flywheelOmega) * FLYWHEEL_RADIUS
        if (exitSpeed < 0.5) return  // Flywheel not spinning fast enough

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

    // Color sensor API - always reads slot[0] if ball present
    fun colorSensorDetectsBall(): Boolean = slots[0] != null
    fun colorSensorGetBallColor(): Balls? = slots[0]?.color

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
        private const val INTAKE_X_OFFSET = 0.25
        private const val INTAKE_Z_OFFSET = 0.05
        private const val INTAKE_CAPTURE_DISTANCE = 0.08
        private const val INTAKE_POWER_THRESHOLD = 0.1
        private const val SPINDEXER_SLOT_ANGLE = 2 * PI / 3
        private const val MODE_CHANGE_ANGLE = PI / 3
        private const val SLOT_ALIGNMENT_TOLERANCE = 0.15
        private const val TRANSFER_POWER_THRESHOLD = 0.5
        private const val FLYWHEEL_RADIUS = 0.04
        private const val BALL_RADIUS = 0.05
        private const val SHOT_SPAWN_OFFSET = 0.15
    }
}
