package sigmacorns.control.mpc

import com.google.gson.Gson
import java.io.File
import java.io.FileReader

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
