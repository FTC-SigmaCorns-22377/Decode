package sigmacorns.io

import sigmacorns.constants.ballExitRadius
import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.flywheelMotor
import sigmacorns.constants.intakeMotor
import sigmacorns.constants.turretPos
import sigmacorns.constants.turretTicksPerRad
import sigmacorns.subsystem.ShooterConfig
import sigmacorns.subsystem.TurretServoConfig
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

class JoltSimIO(
    /** When true, update() sleeps to match wall-clock time to sim time (for visualization). */
    var realTime: Boolean = false,
) : SigmaIO, AutoCloseable {
    private val handle: Long = JoltNative.nativeCreate()
    private val drivetrain = MecanumDynamics(drivetrainParameters)
    private val flywheelDynamics = FlywheelDynamics(SIM_FLYWHEEL_PARAMS)
    private val intakeRollerDynamics = FlywheelDynamics(INTAKE_ROLLER_PARAMS)
    private val robotState = FloatArray(6) // [x, y, theta, vx, vy, omega]

    private var t = 0.seconds
    private val wallClockStart = System.nanoTime()
    private var flywheelState = FlywheelState()
    private var intakeRollerState = FlywheelState()
    private var turretAngleRad = 0.0
    private var turretVelocityRad = 0.0
    private var turretOffset = 0.0
    private var hoodAngleRad = Math.toRadians(ShooterConfig.defaultAngleDeg)

    val heldBalls = mutableListOf<BallColor>()
    private var autoShootTimer = 0.0

    /** Hood servo input: 0.0 = horizontal, 1.0 = 70 degrees */
    var hoodInput: Double = DEFAULT_BALL_LAUNCH_ANGLE_DEGREES / 70.0

    override var driveFL: Double = 0.0
    override var driveBL: Double = 0.0
    override var driveFR: Double = 0.0
    override var driveBR: Double = 0.0
    override var flywheel: Double = 0.0
    override var intake: Double = 0.0
    override var turret: Double = 0.0
    override var turretLeft: Double = 0.5
    override var turretRight: Double = 0.5
    override var hood: Double = 0.5
        set(value) { field = value; hoodInput = value }
    override var blocker: Double = 0.0

    override fun position(): Pose2d {
        JoltNative.nativeGetRobotState(handle, robotState)
        return Pose2d(robotState[0].toDouble(), robotState[1].toDouble(), robotState[2].toDouble())
    }

    override fun velocity(): Pose2d {
        JoltNative.nativeGetRobotState(handle, robotState)
        return Pose2d(robotState[3].toDouble(), robotState[4].toDouble(), robotState[5].toDouble())
    }

    // Sim beam breaks based on held ball count
    override fun beamBreak1(): Boolean = heldBalls.size >= 1
    override fun beamBreak2(): Boolean = heldBalls.size >= 2
    override fun beamBreak3(): Boolean = heldBalls.size >= 3

    override fun flywheelVelocity(): Double = flywheelState.omega

    override fun intake1RPM(): Double = intakeRollerState.omega

    override fun turretPosition(): Double = turretAngleRad + turretOffset
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

        // Turret servo simulation: convert servo position to angle.
        // Turret.kt writes turretLeft where 0.5 = center (servoCenterAngle).
        // angle = (servoPos - 0.5) * servoTotalRange + servoCenterAngle
        val targetTurretAngle = ((turretLeft - 0.5) * TurretServoConfig.servoTotalRange +
            TurretServoConfig.servoCenterAngle)
            .coerceIn(-TURRET_ANGLE_LIMIT, TURRET_ANGLE_LIMIT)

        // Smooth servo response (first-order with time constant)
        val servoAlpha = 1.0 - kotlin.math.exp(-dtSeconds / SERVO_TIME_CONSTANT)
        turretAngleRad += servoAlpha * (targetTurretAngle - turretAngleRad)

        // Integrate hood servo
        // Shooter maps [minAngleDeg, maxAngleDeg] → [0, 1], so invert:
        // angle = minAngle + servoPos * (maxAngle - minAngle)
        val hoodMinRad = Math.toRadians(ShooterConfig.minAngleDeg)
        val hoodMaxRad = Math.toRadians(ShooterConfig.maxAngleDeg)
        val hoodTarget = (hoodMinRad + hoodInput.coerceIn(0.0, 1.0) * (hoodMaxRad - hoodMinRad))
        val hoodError = hoodTarget - hoodAngleRad
        val hoodVel = (SERVO_K * hoodError).coerceIn(-SERVO_MAX_SPEED, SERVO_MAX_SPEED)
        hoodAngleRad += hoodVel * dtSeconds
        hoodAngleRad = hoodAngleRad.coerceIn(hoodMinRad, hoodMaxRad)

        // Physics-based intake: C++ removes balls and returns their colors
        val maxPickup = (3 - heldBalls.size).coerceAtLeast(0)
        if (maxPickup > 0) {
            val colorBuf = IntArray(maxPickup)
            val count = JoltNative.nativeCollectPickups(handle, colorBuf, maxPickup)
            for (i in 0 until count) {
                val color = BallColor.fromJoltId(colorBuf[i])
                heldBalls.add(color)
                println("intake: picked up $color, held=${heldBalls.size}")
            }
        } else {
            // Drain pending pickups so they don't accumulate
            val drain = IntArray(10)
            JoltNative.nativeGetPendingPickups(handle, drain, 10)
        }

        // Auto-shoot: when blocker is disengaged and transfer motor is running,
        // the ball path to the flywheel is open. If flywheel is spinning fast enough,
        // shoot one ball per transfer cycle (~200ms between shots).
        if (blocker > 0.5 && intake > 0.5 && heldBalls.isNotEmpty()) {
            autoShootTimer += dtSeconds
            if (autoShootTimer >= AUTO_SHOOT_INTERVAL) {
                autoShootTimer = 0.0
                shootBall()
            }
        } else {
            autoShootTimer = 0.0
        }

        t += SIM_UPDATE_TIME

        // Pace to wall-clock time for real-time visualization
        if (realTime) {
            val simTimeNanos = t.inWholeNanoseconds
            val wallElapsed = System.nanoTime() - wallClockStart
            val sleepNanos = simTimeNanos - wallElapsed
            if (sleepNanos > 0) {
                Thread.sleep(sleepNanos / 1_000_000, (sleepNanos % 1_000_000).toInt())
            }
        }
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
        if (heldBalls.isEmpty()) {
            println("shootBall: no balls held")
            return
        }

        val exitSpeed = abs(flywheelState.omega) * FLYWHEEL_WHEEL_RADIUS * LAUNCH_EFFICIENCY
        if (exitSpeed < 0.1) {
            println("shootBall: flywheel too slow (${abs(flywheelState.omega).toInt()} rad/s)")
            return
        }

        val color = heldBalls.removeAt(0)
        println("shootBall: shot $color, held=${heldBalls.size}, exitSpeed=${"%.1f".format(exitSpeed)} m/s")

        JoltNative.nativeGetRobotState(handle, robotState)
        val robotX = robotState[0].toDouble()
        val robotY = robotState[1].toDouble()
        val robotTheta = robotState[2].toDouble()

        // Launch direction: robot heading + turret angle
        val launchHeading = robotTheta + turretAngleRad

        // Hood angle splits exit speed into horizontal and vertical components
        val horizontalSpeed = exitSpeed * cos(hoodAngleRad)
        val verticalSpeed = exitSpeed * sin(hoodAngleRad)

        val vx = horizontalSpeed * cos(launchHeading) + robotState[3]
        val vy = horizontalSpeed * sin(launchHeading) + robotState[4]
        val vz = verticalSpeed

        // Start at the shooter position on the robot (offset along robot heading)
        val shooterX = robotX + turretPos.x * cos(robotTheta)
        val shooterY = robotY + turretPos.x * sin(robotTheta)
        val shooterZ = turretPos.z

        // hood angle correction
        val spawnX = shooterX + cos(launchHeading)*(1.0 - sin(hoodAngleRad))*ballExitRadius
        val spawnY = shooterY + sin(launchHeading)*(1.0 - sin(hoodAngleRad))*ballExitRadius
        val spawnZ = shooterZ + cos(hoodAngleRad)*ballExitRadius

        JoltNative.nativeSpawnShotBall(handle,
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

        /** Time between auto-shot attempts when transfer motor is running (seconds). */
        const val AUTO_SHOOT_INTERVAL = 0.2

        // Robot geometry (must match C++ jolt_world.h)
        const val ROBOT_HEIGHT = 0.15
        const val ROBOT_LENGTH = 0.4

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
        /** Servo first-order response time constant (seconds). */
        const val SERVO_TIME_CONSTANT = 0.05
        val TURRET_ANGLE_LIMIT = Math.PI // ±180° range matching turretRange limits

        // Default hood angle
        const val DEFAULT_BALL_LAUNCH_ANGLE_DEGREES = 45.0

        // Flywheel inertia: ½mr² = 0.5 * 0.38709252 kg * (0.046 m)²
        const val SIM_FLYWHEEL_INERTIA = 0.000410 // kg·m^2
        val SIM_FLYWHEEL_PARAMS = FlywheelParameters(flywheelMotor, SIM_FLYWHEEL_INERTIA, 0.0001)

        // Intake roller dynamics (reuses FlywheelDynamics with different parameters)
        const val INTAKE_ROLLER_INERTIA = 0.0005 // kg·m^2
        val INTAKE_ROLLER_PARAMS = FlywheelParameters(intakeMotor, INTAKE_ROLLER_INERTIA, 0.0001)

        // Servo model constants
        const val SERVO_TURRET_RANGE = 2 * Math.PI // rad, full range of servo
        const val SERVO_K = 10.0 // proportional gain (1/s)
        const val SERVO_MAX_SPEED = Math.PI // rad/s, max angular velocity

        // Hood servo (controls launch angle)
        val HOOD_RANGE = Math.toRadians(70.0)  // 0 to 70 degrees
    }
}
