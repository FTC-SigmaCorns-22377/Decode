package sigmacorns.control.mpc

import sigmacorns.control.subsystem.DriveController
import sigmacorns.io.SigmaIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.MecanumState
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.thread
import kotlin.concurrent.withLock
import kotlin.system.measureNanoTime
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

/**
 * Commands that can be sent to the MPC thread for safe execution.
 */
private sealed class MPCCommand {
    data class SetTarget(val traj: TrajoptTrajectory) : MPCCommand()
}

/**
 * Manages a background thread for MPC computation, providing thread-safe
 * state exchange between the main opmode loop and the MPC solver.
 *
 * Usage:
 * ```
 * val runner = MPCRunner(mpc)
 * runner.start()
 *
 * // In main loop:
 * runner.updateState(state.mecanumState, voltage, time)
 * val control = runner.getControl(voltage)
 * runner.driveWithControl(control, io)
 *
 * // When done:
 * runner.stop()
 * ```
 */
class MPCRunner(
    val mpc: MPCClient,
    val driveController: DriveController = DriveController(),
) : AutoCloseable {

    private val mpcLock = ReentrantLock()
    private var latestU = doubleArrayOf(0.0, 0.0, 0.0)
    private var sharedMecanumState: MecanumState? = null
    private var sharedVoltage = 12.0
    private var sharedTime: Duration = 0.seconds
    val mpcRunning = AtomicBoolean(false)
    private var mpcThread: Thread? = null

    // Command queue for thread-safe MPC operations
    private val commandQueue = ConcurrentLinkedQueue<MPCCommand>()

    // Cached state values (updated by MPC thread, read by main thread with lock)
    private var cachedLatestSampleI = 0
    private var cachedIsTrajectoryComplete: TrajoptTrajectory? = null

    /**
     * Start the background MPC thread.
     * The thread continuously reads shared state, sends it to the solver,
     * and stores the latest control output.
     */
    fun start() {
        if (mpcRunning.getAndSet(true)) return // already running

        mpcThread = thread(name = "MPC-Thread") {
            while (mpcRunning.get()) {
                val (mecanumState, voltage, time) = mpcLock.withLock {
                    Triple(sharedMecanumState, sharedVoltage, sharedTime)
                }

                // Process all pending commands first
                while (true) {
                    val cmd = commandQueue.poll() ?: break
                    when (cmd) {
                        is MPCCommand.SetTarget -> mpc.setTarget(cmd.traj)
                    }
                }

                if (mecanumState != null) {
                    val n = measureNanoTime {
                        val u = mpc.getU(time)
                        mpc.update(mecanumState, voltage, time)

                        // Update cached state and control under lock
                        mpcLock.withLock {
                            latestU = u
                            if(!mpc.isTrajectoryComplete()) cachedLatestSampleI = mpc.latestSampleI
                            cachedIsTrajectoryComplete = mpc.trajSnapshot.trajectory.takeIf { mpc.isTrajectoryComplete() }
                        }
                    }
                    println("MPC thread update took ${n.toDouble() / 1_000_000} ms")
                }

                Thread.sleep(1)
            }
        }
    }

    /**
     * Stop the background MPC thread and wait for it to finish.
     */
    fun stop() {
        mpcRunning.set(false)
        mpcThread?.join(50)
        if (mpcThread?.isAlive == true) {
            mpcThread?.interrupt()
        }
        mpcThread = null
    }

    /**
     * Update the shared state that the MPC thread reads.
     * Call this from the main loop each iteration.
     */
    fun updateState(mecanumState: MecanumState, voltage: Double, time: Duration) {
        mpcLock.withLock {
            sharedMecanumState = mecanumState
            sharedVoltage = voltage
            sharedTime = time
        }
    }

    /**
     * Get the latest MPC control output, voltage-compensated.
     * Call this from the main loop each iteration after [updateState].
     *
     * @return list of [drive, strafe, turn] values
     */
    fun getControl(voltage: Double): List<Double> {
        return mpcLock.withLock {
            latestU.copyOf().map { it * 12.0 / voltage }
        }
    }

    /**
     * Convenience: get control and apply it to the drivetrain in one call.
     */
    fun driveWithMPC(io: SigmaIO, voltage: Double) {
        val u = getControl(voltage)
        driveController.drive(Pose2d(u[0], u[1], u[2]), io)
    }

    /**
     * Set the target trajectory for the MPC to follow.
     * Thread-safe: posts command to MPC thread for execution.
     * Non-blocking: returns immediately.
     */
    fun setTarget(traj: TrajoptTrajectory) {
        commandQueue.add(MPCCommand.SetTarget(traj))
        mpcLock.withLock {
            cachedLatestSampleI = 0
        }
    }

    /**
     * Check if the current trajectory is complete.
     * Thread-safe: reads cached value updated by MPC thread.
     *
     * @return true if at the last sample of the trajectory or no trajectory set
     */
    fun isTrajectoryComplete(traj: TrajoptTrajectory): Boolean {
        return mpcLock.withLock {
            cachedIsTrajectoryComplete == traj
        }
    }

    /**
     * Get the current trajectory sample index.
     * This tracks which sample point the MPC is closest to.
     * Thread-safe: reads cached value updated by MPC thread.
     */
    val latestSampleI: Int
        get() = mpcLock.withLock {
            cachedLatestSampleI
        }

    override fun close() {
        stop()
    }
}
