package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.joml.Vector2d
import sigmacorns.State
import sigmacorns.constants.Network
import sigmacorns.constants.drivetrainParameters
import sigmacorns.control.DriveController
import sigmacorns.control.PollableDispatcher
import sigmacorns.io.ContourSelectionMode
import sigmacorns.io.HardwareIO
import sigmacorns.io.MPCClient
import sigmacorns.io.TrajoptLoader
import sigmacorns.math.Pose2d
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.LinearDcMotor
import sigmacorns.sim.MecanumState
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.thread
import kotlin.concurrent.withLock
import kotlin.math.min
import kotlin.system.measureNanoTime
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@Autonomous
class RedAutoPathing: SigmaOpMode() {
    override fun runOpMode() {
        val robotDir = TrajoptLoader.robotTrajoptDir()
        val projectFile = TrajoptLoader.findProjectFiles(robotDir).find { it.nameWithoutExtension == "base" }!!

        val intake1 = TrajoptLoader.loadTrajectory(projectFile,"intake_1")!!
        val intake2 = TrajoptLoader.loadTrajectory(projectFile,"intake_2")!!
        val intake3 = TrajoptLoader.loadTrajectory(projectFile,"intake_3")!!

        val dispatcher = PollableDispatcher(io)

        val ll = (io as? HardwareIO)?.limelight
        ll!!.pipelineSwitch(1)

        io.setPosition(intake1.getInitialSample()!!.pos)

        MPCClient(
            drivetrainParameters.let {
                val p = it.copy()
                p.motor = LinearDcMotor(it.motor.freeSpeed, it.motor.stallTorque*0.8)
                p
            },
            Network.LIMELIGHT,
            contourSelectionMode = ContourSelectionMode.POSITION,
            preIntegrate = 30.milliseconds,
            sampleLookahead = 0
        ).use { mpc ->
            val state = State(
                0.0,
                io.position(),
                Pose2d(),
                Pose2d(Vector2d(), 0.0),
                0.0,
                0.0,
                0.seconds
            )

            // Shared state for MPC thread communication
            val mpcLock = ReentrantLock()
            var latestU = doubleArrayOf(0.0, 0.0, 0.0)
            var sharedMecanumState: MecanumState? = null
            var sharedVoltage: Double = 12.0
            var sharedTime: Duration = 0.seconds
            val mpcRunning = AtomicBoolean(true)

            waitForStart()

            // Start MPC thread
            val mpcThread = thread(name = "MPC-Thread") {
                while (mpcRunning.get()) {
                    val (mecanumState, voltage, time) = mpcLock.withLock {
                        Triple(sharedMecanumState, sharedVoltage, sharedTime)
                    }

                    if (mecanumState != null) {
                        val n = measureNanoTime {
                            // Receive any pending responses and get control output
                            val u = mpc.getU(time)
                            mpcLock.withLock {
                                latestU = u
                            }

                            // Send new request with current state
                            mpc.update(mecanumState, voltage, time)
                        }
                        println("MPC thread update took ${n.toDouble()/1_000_000} ms")
                    }

                    // Small sleep to prevent busy-waiting
                    Thread.sleep(1)
                }
            }

            val driveController = DriveController()

            val schedule = CoroutineScope(dispatcher).launch {
                mpc.runTrajectory(intake1,1.milliseconds)()
                delay(1000)
                mpc.runTrajectory(intake2,1.milliseconds)()
                delay(1000)
                mpc.runTrajectory(intake3,1.milliseconds)()
                delay(3000)
            }

            while (opModeIsActive() && !schedule.isCompleted) {
                val t = io.time()
                state.update(io)

                dispatcher.update()

                // Update shared state for MPC thread
                val u: List<Double>
                val n = measureNanoTime {
                    mpcLock.withLock {
                        sharedMecanumState = state.mecanumState
                        sharedVoltage = min(11.0,io.voltage())
                        sharedTime = t
                        u = latestU.copyOf().map {
                            it*11.0/io.voltage()
                        }
                    }
                }

                println("MPC state sync took ${n.toDouble()/1_000_000} ms")

                // u is now [drive, strafe, turn]
                val drive = u[0]
                val strafe = u[1]
                val turn = u[2]

                // Convert drive, strafe, turn to motor powers using DriveController
                val robotPower = Pose2d(drive, strafe, turn)
                driveController.drive(robotPower, io)

                val n2 = measureNanoTime { io.update() }

                println("MPC IO update took ${n2.toDouble()/1_000_000} ms")
            }

            // Stop MPC thread
            mpcRunning.set(false)
            mpcThread.join(50)
            if (mpcThread.isAlive) {
                mpcThread.interrupt()
            }
        }
    }
}