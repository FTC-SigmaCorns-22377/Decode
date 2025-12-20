package sigmacorns.sim

import org.joml.Vector2d
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d
import org.joml.Vector3d

class DrakeRobotModel(urdfPath: String) {
    private var simPtr: Long = 0

    init {
        simPtr = DrakeNative.createSim(urdfPath)
    }

    // Mirroring RobotModel properties for compatibility
    var drivetrainState = MecanumState(Pose2d(), Pose2d())
    var flywheelState = FlywheelState()
    var spindexerState = SpindexerState(0.0, listOf(), 0.0)

    // Viz support
    var jointPositions = mutableMapOf<String, Double>()
    var jointVelocities = mutableMapOf<String, Double>()
    var ballPositions = List(10) { Vector3d() }

    fun advanceSim(t: Double, io: SimIO) {
        // Map IO to input vector
        // Input order: fl, bl, br, fr, intake, spindexer, turret, flywheel, hood
        val inputs = doubleArrayOf(
            io.driveFL, // fl
            io.driveBL, // bl
            io.driveBR, // br
            io.driveFR, // fr
            io.intake,  // intake
            io.spindexer, // spindexer
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
        // [x, y, yaw, vx, vy, omega, joint_positions..., joint_velocities...]
        val jointNames = listOf(
            "fl_wheel_joint", "bl_wheel_joint", "br_wheel_joint", "fr_wheel_joint",
            "intake_joint", "spindexer_joint", "turret_joint", "flywheel_joint", "hood_joint"
        )

        val expectedSize = 6 + jointNames.size * 2
        if (state.size >= expectedSize) {
            val x = state[0]
            val y = state[1]
            val yaw = state[2]
            val vx = state[3]
            val vy = state[4]
            val w = state[5]

            drivetrainState.pos = Pose2d(Vector2d(x, y), yaw)
            drivetrainState.vel = Pose2d(Vector2d(vx, vy), w)

            val jointPosStart = 6
            val jointVelStart = 6 + jointNames.size
            for (i in jointNames.indices) {
                val qVal = state[jointPosStart + i]
                val vVal = state[jointVelStart + i]
                jointPositions[jointNames[i]] = qVal
                jointVelocities[jointNames[i]] = vVal
            }

            val flywheelVel = state[jointVelStart + 7]
            flywheelState.omega = flywheelVel

            val ballStart = 6 + jointNames.size * 2
            if (state.size >= ballStart + 10 * 3) {
                for (i in 0 until 10) {
                    ballPositions[i].x = state[ballStart + i * 3]
                    ballPositions[i].y = state[ballStart + i * 3 + 1]
                    ballPositions[i].z = state[ballStart + i * 3 + 2]
                }
            }
        }
    }

    fun destroy() {
        DrakeNative.destroySim(simPtr)
    }

    fun spawnBall(x: Double, y: Double, z: Double) {
        DrakeNative.spawnBall(simPtr, x, y, z)
    }
}
