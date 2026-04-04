package sigmacorns.control.ltv

import org.joml.Vector3d
import sigmacorns.State
import sigmacorns.control.trajopt.TrajoptTrajectory
import sigmacorns.io.RerunLogging
import sigmacorns.io.SigmaIO

class LTVRerunLogger(
    private val rr: RerunLogging,
) : AutoCloseable {

    /** Log the reference trajectory path as a line strip (call once after loading). */
    fun logTrajectoryPath(traj: TrajoptTrajectory) {
        val points = traj.samples.map { Vector3d(it.x, it.y, 0.0) }
        rr.logLineStrip("ltv/path", points)
    }

    /** Log the full robot state (pose, velocity, etc). */
    fun logState(state: State) {
        rr.logState(state)
    }

    /** Log motor inputs from the IO interface. */
    fun logInputs(io: SigmaIO) {
        rr.logInputs(io)
    }

    /** Log LTV control outputs (wheel duty cycles in SigmaIO order). */
    fun logControls(u: DoubleArray) {
        rr.logScalar("ltv/u/FL", u[0])
        rr.logScalar("ltv/u/BL", u[1])
        rr.logScalar("ltv/u/BR", u[2])
        rr.logScalar("ltv/u/FR", u[3])
    }

    /** Log tracking error between current state and reference trajectory. */
    fun logTrackingError(state: State, traj: TrajoptTrajectory, elapsedSeconds: Double) {
        val ref = traj.sampleAt(elapsedSeconds) ?: return
        val dx = state.driveTrainPosition.v.x - ref.x
        val dy = state.driveTrainPosition.v.y - ref.y
        val dtheta = state.driveTrainPosition.rot - ref.heading
        rr.logScalar("ltv/error/x", dx)
        rr.logScalar("ltv/error/y", dy)
        rr.logScalar("ltv/error/theta", dtheta)
        rr.logScalar("ltv/error/pos", kotlin.math.hypot(dx, dy))
    }

    override fun close() {
        rr.close()
    }
}
