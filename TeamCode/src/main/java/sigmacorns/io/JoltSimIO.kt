package sigmacorns.io

import sigmacorns.constants.drivetrainParameters
import sigmacorns.math.Pose2d
import sigmacorns.sim.JoltNative
import sigmacorns.sim.MecanumDynamics
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

enum class BallColor(val joltId: Int) {
    GREEN(0),
    PURPLE(1);
    companion object {
        fun fromJoltId(id: Int) = if (id == 0) GREEN else PURPLE
    }
}

class JoltSimIO : SigmaIO, AutoCloseable {
    private val handle: Long = JoltNative.nativeCreate()
    private val drivetrain = MecanumDynamics(drivetrainParameters)
    private val robotState = FloatArray(6) // [x, y, theta, vx, vy, omega]

    private var t = 0.seconds

    val heldBalls = mutableListOf<BallColor>()

    override var driveFL: Double = 0.0
    override var driveBL: Double = 0.0
    override var driveFR: Double = 0.0
    override var driveBR: Double = 0.0
    override var flywheel: Double = 0.0
    override var intake: Double = 0.0
    override var turret: Double = 0.0

    override fun position(): Pose2d {
        JoltNative.nativeGetRobotState(handle, robotState)
        return Pose2d(robotState[0].toDouble(), robotState[1].toDouble(), robotState[2].toDouble())
    }

    override fun velocity(): Pose2d {
        JoltNative.nativeGetRobotState(handle, robotState)
        return Pose2d(robotState[3].toDouble(), robotState[4].toDouble(), robotState[5].toDouble())
    }

    override fun flywheelVelocity(): Double = 0.0

    override fun turretPosition(): Double = 0.0

    override fun setTurretPosition(offset: Double) {}

    override fun setPosition(p: Pose2d) {
        JoltNative.nativeSetRobotPose(handle, p.v.x.toFloat(), p.v.y.toFloat(), p.rot.toFloat())
    }

    override fun time(): Duration = t

    override fun configurePinpoint() {}

    override fun voltage(): Double = 12.0

    override fun update() {
        val dt = SIM_UPDATE_TIME.toDouble(DurationUnit.SECONDS).toFloat()

        // Read current robot state from Jolt
        JoltNative.nativeGetRobotState(handle, robotState)
        val fieldVel = Pose2d(robotState[3].toDouble(), robotState[4].toDouble(), robotState[5].toDouble())
        val heading = robotState[2].toDouble()

        // Compute forces from motor model
        val motorPowers = doubleArrayOf(driveFL, driveBL, driveBR, driveFR)
        val forces = drivetrain.computeForces(motorPowers, fieldVel, heading)

        // Apply forces and step physics
        JoltNative.nativeApplyRobotForce(handle,
            forces.v.x.toFloat(), forces.v.y.toFloat(), forces.rot.toFloat())
        JoltNative.nativeStep(handle, dt)

        // Intake: pick up balls if intake is running and we have capacity
        if (intake > 0.3 && heldBalls.size < 3) {
            val overlaps = IntArray(10)
            val count = JoltNative.nativeGetIntakeOverlaps(handle, overlaps, 10)
            if (count > 0) {
                // Pick up the first overlapping ball
                val ballIdx = overlaps[0]
                val colors = IntArray(JoltNative.nativeGetBallCount(handle))
                JoltNative.nativeGetBallColors(handle, colors)
                if (ballIdx < colors.size) {
                    heldBalls.add(BallColor.fromJoltId(colors[ballIdx]))
                    JoltNative.nativeRemoveBall(handle, ballIdx)
                }
            }
        }

        t += SIM_UPDATE_TIME
    }

    // --- Ball API ---

    fun spawnBall(x: Float, y: Float, z: Float, color: BallColor): Int {
        return JoltNative.nativeSpawnBall(handle, x, y, z, 0f, 0f, 0f, color.joltId)
    }

    fun launchBall(color: BallColor, speed: Float = 5f) {
        if (heldBalls.isEmpty()) return
        heldBalls.removeAt(0)

        JoltNative.nativeGetRobotState(handle, robotState)
        val x = robotState[0]
        val y = robotState[1]
        val theta = robotState[2]
        val z = 0.2f // launch height

        val vx = speed * kotlin.math.cos(theta)
        val vy = speed * kotlin.math.sin(theta)
        val vz = speed * 0.5f // upward component

        JoltNative.nativeSpawnBall(handle, x, y, z, vx, vy, vz, color.joltId)
    }

    fun getBallCount(): Int = JoltNative.nativeGetBallCount(handle)

    data class BallState(val x: Float, val y: Float, val z: Float, val color: BallColor)

    fun getBallStates(): List<BallState> {
        val count = JoltNative.nativeGetBallCount(handle)
        if (count == 0) return emptyList()

        val positions = FloatArray(count * 3)
        val colors = IntArray(count)
        JoltNative.nativeGetBallStates(handle, positions)
        JoltNative.nativeGetBallColors(handle, colors)

        return (0 until count).map { i ->
            BallState(positions[i*3], positions[i*3+1], positions[i*3+2], BallColor.fromJoltId(colors[i]))
        }
    }

    fun spawnFieldBalls() {
        val tile = 0.6096f
        val ballOffset = 0.0635f * 2.1f

        // Blue side (x = -2*tile = -1.2192)
        val blueX = -2f * tile
        val rows = floatArrayOf(tile / 2f, -tile / 2f, -tile * 1.5f) // 0.3048, -0.3048, -0.9144
        val greenIndices = intArrayOf(1, 0, 2) // which ball in each row is green

        for ((rowIdx, rowY) in rows.withIndex()) {
            val xOffsets = floatArrayOf(blueX - ballOffset, blueX, blueX + ballOffset)
            for ((ballIdx, bx) in xOffsets.withIndex()) {
                val color = if (ballIdx == greenIndices[rowIdx]) BallColor.GREEN else BallColor.PURPLE
                spawnBall(bx, rowY, 0.0635f, color)
            }
        }

        // Red side (mirrored, x = +2*tile)
        val redX = 2f * tile
        for ((rowIdx, rowY) in rows.withIndex()) {
            val xOffsets = floatArrayOf(redX + ballOffset, redX, redX - ballOffset)
            for ((ballIdx, bx) in xOffsets.withIndex()) {
                val color = if (ballIdx == greenIndices[rowIdx]) BallColor.GREEN else BallColor.PURPLE
                spawnBall(bx, rowY, 0.0635f, color)
            }
        }
    }

    override fun close() {
        JoltNative.nativeDestroy(handle)
    }
}
