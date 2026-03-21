package sigmacorns.io

import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.flywheelMotor
import sigmacorns.constants.intakeMotor
import sigmacorns.sim.FlywheelParameters
import sigmacorns.sim.LinearDcMotor
import sigmacorns.math.Pose2d
import sigmacorns.sim.FlywheelDynamics
import sigmacorns.sim.FlywheelState
import sigmacorns.sim.JoltNative
import sigmacorns.sim.MecanumDynamics
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
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
    private val flywheelDynamics = FlywheelDynamics(SIM_FLYWHEEL_PARAMS)
    private val intakeRollerDynamics = FlywheelDynamics(INTAKE_ROLLER_PARAMS)
    private val robotState = FloatArray(6) // [x, y, theta, vx, vy, omega]

    private var t = 0.seconds
    private var flywheelState = FlywheelState()
    private var intakeRollerState = FlywheelState()
    private var turretAngleRad = 0.0
    private var turretVelocityRad = 0.0
    private var turretOffset = 0.0
    private var hoodAngleRad = Math.toRadians(DEFAULT_BALL_LAUNCH_ANGLE_DEGREES)

    val heldBalls = mutableListOf<BallColor>()

    /** Hood servo input: 0.0 = horizontal, 1.0 = 70 degrees */
    var hood: Double = DEFAULT_BALL_LAUNCH_ANGLE_DEGREES / 70.0

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

    override fun flywheelVelocity(): Double = flywheelState.omega

    override fun turretPosition(): Double = turretAngleRad * turretTicksPerRad + turretOffset

    override fun setTurretPosition(offset: Double) {
        turretOffset = offset
    }

    override fun setPosition(p: Pose2d) {
        JoltNative.nativeSetRobotPose(handle, p.v.x.toFloat(), p.v.y.toFloat(), p.rot.toFloat())
    }

    override fun time(): Duration = t

    override fun configurePinpoint() {}

    override fun voltage(): Double = 12.0

    override fun update() {
        val dtSeconds = SIM_UPDATE_TIME.toDouble(DurationUnit.SECONDS)
        val dt = dtSeconds.toFloat()

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

        // Integrate flywheel dynamics
        flywheelState = flywheelDynamics.integrate(dtSeconds, dtSeconds, doubleArrayOf(flywheel), flywheelState)

        // Integrate intake roller dynamics
        intakeRollerState = intakeRollerDynamics.integrate(dtSeconds, dtSeconds, doubleArrayOf(intake), intakeRollerState)

        // Send roller omega to C++ for contact surface velocity
        JoltNative.nativeSetIntakeRollerOmega(handle, intakeRollerState.omega.toFloat())

        // Integrate turret (servo position model)
        // turret power 0..1 maps to -SERVO_TURRET_RANGE/2..+SERVO_TURRET_RANGE/2
        val turretTarget = (turret - 0.5) * SERVO_TURRET_RANGE
        val turretError = turretTarget - turretAngleRad
        val turretVel = (SERVO_K * turretError).coerceIn(-SERVO_MAX_SPEED, SERVO_MAX_SPEED)
        turretAngleRad += turretVel * dtSeconds
        turretAngleRad = turretAngleRad.coerceIn(-SERVO_TURRET_RANGE / 2.0, SERVO_TURRET_RANGE / 2.0)

        // Integrate hood servo
        val hoodTarget = hood.coerceIn(0.0, 1.0) * HOOD_RANGE
        val hoodError = hoodTarget - hoodAngleRad
        val hoodVel = (SERVO_K * hoodError).coerceIn(-SERVO_MAX_SPEED, SERVO_MAX_SPEED)
        hoodAngleRad += hoodVel * dtSeconds
        hoodAngleRad = hoodAngleRad.coerceIn(0.0, HOOD_RANGE)

        // Physics-based intake: check for pending pickups from C++
        if (heldBalls.size < 3) {
            val pickupBuf = IntArray(10)
            val count = JoltNative.nativeGetPendingPickups(handle, pickupBuf, 10)
            for (i in 0 until count) {
                if (heldBalls.size >= 3) break
                val ballIdx = pickupBuf[i]
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

    // --- Shooter API ---

    /**
     * Shoots a held ball using the flywheel RPM and hood angle.
     *
     * The flywheel (at the front of the shooter) determines exit speed via surface velocity.
     * The adjustable hood (at the back) controls the exit angle/elevation.
     * The turret angle determines the horizontal launch direction relative to the robot.
     */
    fun shootBall() {
        if (heldBalls.isEmpty()) return

        val exitSpeed = abs(flywheelState.omega) * FLYWHEEL_WHEEL_RADIUS * LAUNCH_EFFICIENCY
        if (exitSpeed < 0.1) return // flywheel not spinning fast enough

        val color = heldBalls.removeAt(0)

        JoltNative.nativeGetRobotState(handle, robotState)
        val robotX = robotState[0].toDouble()
        val robotY = robotState[1].toDouble()
        val robotTheta = robotState[2].toDouble()

        // Launch direction: robot heading + turret angle
        val launchHeading = robotTheta + turretAngleRad

        // Hood angle splits exit speed into horizontal and vertical components
        val horizontalSpeed = exitSpeed * cos(hoodAngleRad)
        val verticalSpeed = exitSpeed * sin(hoodAngleRad)

        val vx = horizontalSpeed * cos(launchHeading)
        val vy = horizontalSpeed * sin(launchHeading)
        val vz = verticalSpeed

        // Start at the shooter position on the robot (offset along robot heading)
        val shooterX = robotX + SHOOTER_FORWARD_OFFSET * cos(robotTheta)
        val shooterY = robotY + SHOOTER_FORWARD_OFFSET * sin(robotTheta)
        val shooterZ = ROBOT_HEIGHT + SHOOTER_HEIGHT

        val spawnX = shooterX
        val spawnY = shooterY
        val spawnZ = shooterZ

        JoltNative.nativeSpawnBall(handle,
            spawnX.toFloat(), spawnY.toFloat(), spawnZ.toFloat(),
            vx.toFloat(), vy.toFloat(), vz.toFloat(),
            color.joltId)
    }

    // --- Intake/Hood getters ---

    fun intakeRollerVelocity(): Double = intakeRollerState.omega

    fun intakeAngle(): Double {
        val state = FloatArray(2)
        JoltNative.nativeGetIntakeState(handle, state)
        return state[0].toDouble()
    }

    fun hoodPosition(): Double = hoodAngleRad

    // --- Goal API ---

    data class GoalState(
        val redScore: Int,
        val blueScore: Int,
        val redGateOpen: Float,
        val blueGateOpen: Float,
        val redLeverAngle: Float,
        val blueLeverAngle: Float
    )

    fun getGoalState(): GoalState {
        val out = FloatArray(6)
        JoltNative.nativeGetGoalStates(handle, out)
        return GoalState(
            redScore = out[0].toInt(),
            blueScore = out[1].toInt(),
            redGateOpen = out[2],
            blueGateOpen = out[3],
            redLeverAngle = out[4],
            blueLeverAngle = out[5]
        )
    }

    // --- Ball API ---

    fun spawnBall(x: Float, y: Float, z: Float, color: BallColor): Int {
        return JoltNative.nativeSpawnBall(handle, x, y, z, 0f, 0f, 0f, color.joltId)
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

    companion object {
        val SIM_UPDATE_TIME = 5.milliseconds

        // Robot geometry (must match C++ jolt_world.h)
        const val ROBOT_HEIGHT = 0.15
        const val ROBOT_LENGTH = 0.4

        // Shooter geometry (~1/4 robot volume: 0.2 x 0.2 x 0.15)
        const val SHOOTER_WIDTH = 0.2
        const val SHOOTER_LENGTH = 0.2
        const val SHOOTER_HEIGHT = 0.15
        // Shooter center is 2/3 back from the front of the robot
        const val SHOOTER_FORWARD_OFFSET = ROBOT_LENGTH / 2.0 - (2.0 / 3.0) * ROBOT_LENGTH // ≈ -0.067

        // Flywheel: front of the hooded shooter, gives exit speed
        const val FLYWHEEL_WHEEL_RADIUS = 0.05 // 50mm contact wheel
        const val LAUNCH_EFFICIENCY = 0.3      // energy transfer to ball

        // Turret (DC motor model matching hardware)
        // Uses the same bare motor specs with a turret-specific gear ratio
        private const val TURRET_GEAR_RATIO = (1.0 + 46.0 / 11.0) * 76.0 / 19.0
        val TURRET_MOTOR = LinearDcMotor(
            sigmacorns.constants.bareMotorTopSpeed / TURRET_GEAR_RATIO,
            sigmacorns.constants.bareMotorStallTorque * TURRET_GEAR_RATIO
        )
        const val TURRET_INERTIA = 0.005 // kg·m^2, turret assembly inertia
        const val TURRET_DAMPING = 0.5 // viscous damping coefficient
        val TURRET_ANGLE_LIMIT = Math.PI / 2.0 // ±90° range matching turretRange limits

        // Default hood angle
        const val DEFAULT_BALL_LAUNCH_ANGLE_DEGREES = 45.0

        // Sim-specific flywheel inertia (small wheel, not the full hardware assembly)
        const val SIM_FLYWHEEL_INERTIA = 0.001 // kg·m^2
        val SIM_FLYWHEEL_PARAMS = FlywheelParameters(flywheelMotor, SIM_FLYWHEEL_INERTIA, 0.0001)

        // Intake roller dynamics (reuses FlywheelDynamics with different parameters)
        const val INTAKE_ROLLER_INERTIA = 0.0005 // kg·m^2
        val INTAKE_ROLLER_PARAMS = FlywheelParameters(intakeMotor, INTAKE_ROLLER_INERTIA, 0.0001)

        // Hood servo (controls launch angle)
        val HOOD_RANGE = Math.toRadians(70.0)  // 0 to 70 degrees
    }
}
