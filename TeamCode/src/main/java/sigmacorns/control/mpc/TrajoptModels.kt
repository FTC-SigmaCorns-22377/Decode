package sigmacorns.control.mpc

import sigmacorns.math.Pose2d

/**
 * Data classes for parsing mecanum_trajopt JSON project files.
 */
data class TrajoptProject(
    val version: Int,
    val name: String,
    val trajectories: List<TrajoptTrajectoryEntry>,
    val robotParams: TrajoptRobotParams? = null,
)

data class TrajoptRobotParams(
    val mass: Double,
    val inertia: Double,
    val wheel_radius: Double,
    val lx: Double,
    val ly: Double,
    val w_max: Double,
    val t_max: Double,
)

data class TrajoptTrajectoryEntry(
    val id: String,
    val name: String,
    val trajectory: TrajoptTrajectoryData?,
    val followsTrajectoryId: String? = null,
    val eventMarkers: List<TrajoptEventMarker>? = null,
)

data class TrajoptTrajectoryData(
    val times: List<Double>,
    val states: List<List<Double>>,
    val controls: List<List<Double>>,
    val totalTime: Double,
    val waypoint_times: List<Double>? = null,
)

data class TrajoptEventMarker(
    val waypointIndex: Int,
    val percentage: Double,
    val name: String,
    val timestamp: Double,
)

/**
 * Represents a trajectory sample at a specific time.
 * State vector: [vx, vy, omega, px, py, theta]
 */
data class TrajoptSample(
    val timestamp: Double,
    val vx: Double,
    val vy: Double,
    val omega: Double,
    val x: Double,
    val y: Double,
    val heading: Double,
) {
    val pos: Pose2d get() = Pose2d(x, y, heading)
    val vel: Pose2d get() = Pose2d(vx, vy, omega)

    fun toStateArray(): DoubleArray = doubleArrayOf(vx, vy, omega, x, y, heading)
}

/**
 * Represents a solved trajectory from the mecanum_trajopt optimizer.
 */
class TrajoptTrajectory(
    val name: String,
    val samples: List<TrajoptSample>,
    val totalTime: Double,
    val waypointTimes: List<Double> = emptyList(),
    val eventMarkers: List<TrajoptEventMarker> = emptyList(),
) {
    fun getInitialSample(): TrajoptSample? = samples.firstOrNull()
    fun getFinalSample(): TrajoptSample? = samples.lastOrNull()

    /**
     * Sample the trajectory at an arbitrary time using linear interpolation.
     */
    fun sampleAt(time: Double): TrajoptSample? {
        if (samples.isEmpty()) return null
        if (time <= 0.0) return samples.first()
        if (time >= totalTime) return samples.last()

        // Find the two samples to interpolate between
        var lowIdx = 0
        var highIdx = samples.size - 1

        while (lowIdx < highIdx - 1) {
            val mid = (lowIdx + highIdx) / 2
            if (samples[mid].timestamp <= time) {
                lowIdx = mid
            } else {
                highIdx = mid
            }
        }

        val s0 = samples[lowIdx]
        val s1 = samples[highIdx]
        val t = (time - s0.timestamp) / (s1.timestamp - s0.timestamp)

        return TrajoptSample(
            timestamp = time,
            vx = s0.vx + t * (s1.vx - s0.vx),
            vy = s0.vy + t * (s1.vy - s0.vy),
            omega = s0.omega + t * (s1.omega - s0.omega),
            x = s0.x + t * (s1.x - s0.x),
            y = s0.y + t * (s1.y - s0.y),
            heading = s0.heading + t * (s1.heading - s0.heading),
        )
    }
}
