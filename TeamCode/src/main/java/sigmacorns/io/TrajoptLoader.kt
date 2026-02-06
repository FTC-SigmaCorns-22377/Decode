package sigmacorns.io

import com.google.gson.Gson
import org.joml.Vector2d
import sigmacorns.math.Pose2d
import java.io.File
import java.io.FileReader

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

/**
 * Loader for mecanum_trajopt trajectory project files.
 *
 * Trajectory files are stored as JSON in /sdcard/FIRST/trajopt/ on the robot
 * or in TeamCode/trajopt/ during development.
 */
object TrajoptLoader {
    private val gson = Gson()

    /**
     * Load a trajectory project from a JSON file.
     */
    fun loadProject(file: File): TrajoptProject {
        return FileReader(file).use { reader ->
            gson.fromJson(reader, TrajoptProject::class.java)
        }
    }

    /**
     * Load a trajectory project from a file path.
     */
    fun loadProject(path: String): TrajoptProject = loadProject(File(path))

    /**
     * Load a specific trajectory by name from a project file.
     */
    fun loadTrajectory(file: File, trajectoryName: String): TrajoptTrajectory? {
        val project = loadProject(file)
        return project.trajectories
            .find { it.name == trajectoryName }
            ?.let { parseTrajectory(it) }
    }

    /**
     * Load the first trajectory from a project file.
     */
    fun loadFirstTrajectory(file: File): TrajoptTrajectory? {
        val project = loadProject(file)
        return project.trajectories.firstOrNull()?.let { parseTrajectory(it) }
    }

    /**
     * Load all trajectories from a project file.
     */
    fun loadAllTrajectories(file: File): List<TrajoptTrajectory> {
        val project = loadProject(file)
        return project.trajectories.mapNotNull { parseTrajectory(it) }
    }

    /**
     * Load a trajectory from a project file by index.
     */
    fun loadTrajectoryByIndex(file: File, index: Int): TrajoptTrajectory? {
        val project = loadProject(file)
        return project.trajectories.getOrNull(index)?.let { parseTrajectory(it) }
    }

    /**
     * Parse a trajectory entry into a TrajoptTrajectory.
     */
    internal fun parseTrajectory(entry: TrajoptTrajectoryEntry): TrajoptTrajectory? {
        val data = entry.trajectory ?: return null

        val samples = data.times.mapIndexed { i, time ->
            val state = data.states.getOrNull(i) ?: return@mapIndexed null
            if (state.size < 6) return@mapIndexed null

            TrajoptSample(
                timestamp = time,
                vx = state[0],
                vy = state[1],
                omega = state[2],
                x = state[3],
                y = state[4],
                heading = state[5],
            )
        }.filterNotNull()

        return TrajoptTrajectory(
            name = entry.name,
            samples = samples,
            totalTime = data.totalTime,
            waypointTimes = data.waypoint_times!!,
            eventMarkers = entry.eventMarkers ?: emptyList(),
        )
    }

    /**
     * Load all trajectories ordered by followsTrajectoryId chain (roots first, then children).
     */
    fun loadAllTrajectoriesOrdered(file: File, root: String?): List<TrajoptTrajectory> {
        val project = loadProject(file)
        val childrenOf = project.trajectories.groupBy { it.followsTrajectoryId }
        val ordered = mutableListOf<TrajoptTrajectoryEntry>()
        fun walk(entry: TrajoptTrajectoryEntry) {
            ordered.add(entry)
            childrenOf[entry.id]?.forEach { walk(it) }
        }

        if(root != null) {
            walk(project.trajectories.find { it.name == root  }!!)
        } else {
            childrenOf[null]?.forEach { walk(it) }
        }

        return ordered.mapNotNull { parseTrajectory(it) }
    }

    /**
     * Find trajectory files in a directory.
     */
    fun findProjectFiles(directory: File): List<File> {
        return directory.listFiles { file ->
            file.isFile && file.extension == "json"
        }?.toList() ?: emptyList()
    }

    /**
     * Default trajopt directory on the robot.
     */
    fun robotTrajoptDir(): File = File("/sdcard/FIRST/trajopt")
}
