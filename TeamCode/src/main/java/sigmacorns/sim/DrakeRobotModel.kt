package sigmacorns.sim

import org.joml.Vector2d
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d
import org.joml.Vector3d
import sigmacorns.io.SigmaIO

class DrakeRobotModel(urdfPath: String) {
    private var simPtr: Long = 0

    init {
        simPtr = DrakeNative.createSim(urdfPath)
    }

    // Mirroring RobotModel properties for compatibility
    var drivetrainState = MecanumState(Pose2d(), Pose2d())
    var baseZ = 0.048
    var flywheelState = FlywheelState()
    var spindexerState = SpindexerState(0.0, listOf(), 0.0)

    // Viz support
    var jointPositions = mutableMapOf<String, Double>()
    var jointVelocities = mutableMapOf<String, Double>()
    var ballPositions = mutableListOf<Vector3d>()

    fun advanceSim(t: Double, io: SigmaIO) {
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
        // [x, y, z, yaw, vx, vy, omega, joint_positions..., joint_velocities...]
        val jointNames = listOf(
            "fl_wheel_joint", "bl_wheel_joint", "br_wheel_joint", "fr_wheel_joint",
            "intake_joint", "spindexer_joint", "turret_joint", "flywheel_joint", "hood_joint"
        )

        val expectedSize = 7 + jointNames.size * 2
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

            val ballStart = 7 + jointNames.size * 2
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
    }

    fun setPosition(p: Pose2d) {
        DrakeNative.setPosition(simPtr, p.v.x, p.v.y, p.rot)
        drivetrainState.pos = p
        drivetrainState.vel = Pose2d()
    }
}
