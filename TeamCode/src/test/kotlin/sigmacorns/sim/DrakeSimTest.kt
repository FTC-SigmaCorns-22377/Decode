package sigmacorns.sim

import org.junit.jupiter.api.Test
import sigmacorns.sim.viz.SimServer
import sigmacorns.sim.viz.SimState
import sigmacorns.sim.viz.BaseState
import sigmacorns.sim.viz.TelemetryState
import sigmacorns.io.SimIO
import sigmacorns.math.Pose2d
import java.io.File
import kotlin.math.sin

class DrakeSimTest {

    @Test
    fun runVizSim() {
        println("CWD: ${System.getProperty("user.dir")}")
        // Only run if we are in an environment that can support it (or manually triggered)
        // For CI/CD, we might want to skip this or run headless.
        // Assuming user runs this locally.
        
        // Ensure natives are found. 
        // Need to set java.library.path or ensure Drake is in path. 
        // This test might fail if Drake isn't setup. 
        
        val urdfCandidates = listOfNotNull(
            System.getenv("ROBOT_URDF"),
            "TeamCode/src/main/assets/robot.urdf",
            "src/main/assets/robot.urdf"
        ).map { File(it) }

        val urdfFile = urdfCandidates.firstOrNull { it.exists() }
        if (urdfFile == null) {
            println("URDF not found, skipping test.")
            return
        }
        
        // We need to ensure the shared library can be loaded.
        // In Gradle `externalNativeBuild` creates the libs in build/intermediates...
        // For unit tests, we need to point java.library.path to where the .so is.
        // Or we can manually load it.
        // For now, let's assume the user has configured the environment or the test setup copies libs.
        // Actually, since I added `externalNativeBuild`, I need to make sure the lib is built and available.
        // `native-lib.so` will be in `TeamCode/build/intermediates/cmake/debug/obj/x86_64/` (if running on x86_64 linux).
        
        // Let's try to load it dynamically if standard load fails, or just print instruction.
        try {
            System.loadLibrary("native-lib")
        } catch (e: UnsatisfiedLinkError) {
            println("Native lib not found in java.library.path. Trying to locate in build dir...")
            val envLib = System.getenv("NATIVE_LIB_PATH")?.let { File(it) }
            val libName = "libnative-lib.so"
            val candidates = listOfNotNull(
                envLib,
                File("TeamCode/build/native-desktop/lib/$libName"),
                File("build/native-desktop/lib/$libName"),
                File("TeamCode/build")
            )
            val lib = candidates.firstOrNull { it.isFile && it.name == libName }
                ?: candidates.asSequence()
                    .filter { it.isDirectory }
                    .flatMap { it.walk() }
                    .firstOrNull { it.name == libName }

            if (lib != null) {
                System.load(lib.absolutePath)
            } else {
                println("Could not find $libName. Build the project first.")
            }
        }

        val robot = DrakeRobotModel(urdfFile.absolutePath)
        val io = SimIO()
        val server = SimServer(8080)
        server.start()
        
        println("Simulation running. Open http://localhost:8080")
        
        // Spawn a test ball
        robot.spawnBall(1.0, 1.0, 0.5)
        
        // Simulation Loop
        val dt = 0.01
        var t = 0.0
        val duration = 600.0 // Run for 600 seconds
        
        try {
            while (t < duration) {
                // Generate some inputs
                io.driveFL = 1.0 // Drive forward
                io.driveFR = 1.0
                io.driveBL = 1.0
                io.driveBR = 1.0
                
                io.turret = sin(t) // Oscillate turret power
                io.turretAngle = 0.4 // Set hood position
                io.shooter = 1.0 // Spin flywheel
                
                robot.advanceSim(dt, io)
                t += dt
                
                // Get State
                val state = robot.drivetrainState
                val fw = robot.flywheelState
                
                // Map to Viz State
                // We need full joint map. DrakeRobotModel currently only parses limited state.
                // To get full visualization, we might need to expose more from DrakeRobotModel or just use what we have.
                // The visualizer expects joint names matching URDF.
                
                val joints = robot.jointPositions
                
                // Ideally update DrakeRobotModel to expose raw joint map.
                // For now, let's just push the Base Pose which is the most important.
                
                val vizState = SimState(
                    t = t,
                    base = BaseState(
                        x = state.pos.v.x,
                        y = state.pos.v.y,
                        z = 0.0, // Assuming planar for now or 0
                        roll = 0.0,
                        pitch = 0.0,
                        yaw = state.pos.rot
                    ),
                    joints = joints,
                    telemetry = TelemetryState(
                        fl = io.driveFL,
                        fr = io.driveFR,
                        bl = io.driveBL,
                        br = io.driveBR,
                        flywheel = fw.omega,
                        turret = io.turret
                    )
                )
                
                server.broadcast(vizState)
                
                Thread.sleep((dt * 1000).toLong())
            }
        } catch (e: Throwable) {
            e.printStackTrace()
        } finally {
            robot.destroy()
            server.stop()
        }
    }
}
