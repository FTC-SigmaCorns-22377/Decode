package sigmacorns.sim

import org.joml.Vector2d
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d
import org.joml.Vector3d
import sigmacorns.io.SigmaIO
import sigmacorns.constants.drivetrainParameters
import sigmacorns.constants.*
import kotlin.math.PI

class DrakeRobotModel(urdfPath: String) {
    private var simPtr: Long = 0

    init {
        simPtr = DrakeNative.createSim(urdfPath)
        // Set mecanum parameters from RobotModelConstants
        DrakeNative.setMecanumParameters(simPtr, drivetrainParameters.toArray())
        // Set motor parameters from RobotModelConstants
        // params: [bare_motor_free_speed, bare_motor_stall_torque,
        //          drive_gear_ratio, spindexer_gear_ratio, turret_gear_ratio,
        //          intake_hood_gear_ratio, flywheel_gear_ratio]
        val motorParams = doubleArrayOf(
            bareMotorTopSpeed,
            bareMotorStallTorque,
            driveGearRatio,
            spindexerGearRatio,
            turretGearRatio,
            intakeHoodGearRatio,
            flywheelGearRatio
        )
        DrakeNative.setMotorParameters(simPtr, motorParams)
    }

    // Mirroring RobotModel properties for compatibility
    var drivetrainState = MecanumState(Pose2d(), Pose2d())
    var baseZ = 0.1
    var flywheelState = FlywheelState()
    var spindexerState = SpindexerState(0.0, listOf(), 0.0)

    // Viz support
    var jointPositions = mutableMapOf<String, Double>()
    var jointVelocities = mutableMapOf<String, Double>()
    var ballPositions = mutableListOf<Vector3d>()
    var ballColors = mutableListOf<Balls>()
    var wheelForces = MutableList(4) { Vector3d() }

    fun advanceSim(t: Double, io: SigmaIO) {
        // Spindexer P-controller: io.spindexer is an encoder tick target (RUN_TO_POSITION mode),
        // not a motor power. Convert to position error and compute motor power via P control.
        val spindexerTickTarget = io.spindexer
        val spindexerTargetRad = spindexerTickTarget / SPINDEXER_TICKS_PER_RAD
        val spindexerCurrentRad = jointPositions["spindexer_joint"] ?: 0.0
        val spindexerError = spindexerTargetRad - spindexerCurrentRad
        val spindexerPower = (SPINDEXER_P_GAIN * spindexerError).coerceIn(-1.0, 1.0)

        // Map IO to input vector
        // Input order: fl, bl, br, fr, intake, spindexer, turret, flywheel, hood
        val inputs = doubleArrayOf(
            io.driveFL, // fl
            io.driveBL, // bl
            io.driveBR, // br
            io.driveFR, // fr
            io.intake,  // intake
            spindexerPower, // spindexer (P-controlled, not raw tick target)
            io.turret, // turret power
            io.shooter, // flywheel
            io.turretAngle // hood (adjustable arc length)
        )

        DrakeNative.step(simPtr, t, inputs)
        updateStateFromNative()
    }

    private fun updateStateFromNative() {
        val state = DrakeNative.getState(simPtr)
        if (state.isEmpty()) return

        // DrakeSim state vector:
        // [x, y, z, yaw, vx, vy, omega, joint_positions..., joint_velocities...]
        val jointNames = listOf(
            "fl_wheel_joint", "bl_wheel_joint", "br_wheel_joint", "fr_wheel_joint",
            "intake_joint", "spindexer_joint", "turret_joint", "flywheel_joint", "hood_joint"
        )

        val wheelForceCount = 4
        val expectedSize = 7 + jointNames.size * 2 + wheelForceCount * 3
        if (state.size >= expectedSize) {
            val x = state[0]
            val y = state[1]
            val z = state[2]
            val yaw = state[3]
            val vx = state[4]
            val vy = state[5]
            val w = state[6]

            drivetrainState.pos = Pose2d(Vector2d(x, y), yaw)
            drivetrainState.vel = Pose2d(Vector2d(vx, vy), w)
            baseZ = z

            val jointPosStart = 7
            val jointVelStart = 7 + jointNames.size
            for (i in jointNames.indices) {
                val qVal = state[jointPosStart + i]
                val vVal = state[jointVelStart + i]
                jointPositions[jointNames[i]] = qVal
                jointVelocities[jointNames[i]] = vVal
            }

            val flywheelVel = state[jointVelStart + 7]
            flywheelState.omega = flywheelVel

            val wheelForceStart = 7 + jointNames.size * 2
            if (wheelForces.size != wheelForceCount) {
                wheelForces = MutableList(wheelForceCount) { Vector3d() }
            }
            for (i in 0 until wheelForceCount) {
                val fx = state[wheelForceStart + i * 3]
                val fy = state[wheelForceStart + i * 3 + 1]
                val fz = state[wheelForceStart + i * 3 + 2]
                wheelForces[i].set(fx, fy, fz)
            }

            val ballStart = wheelForceStart + wheelForceCount * 3
            val remaining = state.size - ballStart
            if (remaining >= 3) {
                val ballCount = remaining / 3
                if (ballPositions.size != ballCount) {
                    ballPositions = MutableList(ballCount) { Vector3d() }
                }
                for (i in 0 until ballCount) {
                    ballPositions[i].x = state[ballStart + i * 3]
                    ballPositions[i].y = state[ballStart + i * 3 + 1]
                    ballPositions[i].z = state[ballStart + i * 3 + 2]
                }
            } else {
                ballPositions.clear()
            }
        }
    }

    fun destroy() {
        DrakeNative.destroySim(simPtr)
    }

    fun spawnBall(x: Double, y: Double, z: Double) {
        DrakeNative.spawnBall(simPtr, x, y, z)
        ballColors.add(Balls.Green)  // Default color
    }

    fun spawnBall(x: Double, y: Double, z: Double, color: Balls) {
        DrakeNative.spawnBall(simPtr, x, y, z)
        ballColors.add(color)
    }

    fun removeBall(index: Int) {
        DrakeNative.removeBall(simPtr, index)
        // Note: ballPositions will be updated on next getState()
        if (index < ballColors.size) {
            ballColors.removeAt(index)
        }
    }

    fun spawnBallWithVelocity(
        x: Double, y: Double, z: Double,
        vx: Double, vy: Double, vz: Double,
        color: Balls
    ) {
        DrakeNative.spawnBallWithVelocity(simPtr, x, y, z, vx, vy, vz)
        ballColors.add(color)
    }

    fun getIntakeContacts(): IntArray {
        return DrakeNative.getIntakeContacts(simPtr)
    }

    fun setPosition(p: Pose2d) {
        DrakeNative.setPosition(simPtr, p.v.x, p.v.y, p.rot)
        drivetrainState.pos = p
        drivetrainState.vel = Pose2d()
    }

    companion object {
        // Spindexer: (1+(46/17)) * (1+(46/11)) * 28 ticks per revolution
        private val SPINDEXER_TICKS_PER_RAD =
            ((1.0 + (46.0 / 17.0)) * (1.0 + (46.0 / 11.0)) * 28.0) / (2 * PI)

        // P gain for simulated RUN_TO_POSITION control (radians -> power)
        private const val SPINDEXER_P_GAIN = 8.0
    }
}
