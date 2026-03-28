package sigmacorns.test.jolt

import com.qualcomm.robotcore.hardware.Gamepad
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.Robot
import sigmacorns.io.BallColor
import sigmacorns.io.JoltSimIO
import sigmacorns.math.Pose2d
import sigmacorns.sim.SimGamepad
import sigmacorns.subsystem.DriveController

class JoltVisualizerTest {
    private lateinit var sim: JoltSimIO

    @BeforeEach
    fun setUp() {
        sim = JoltSimIO()
    }

    @AfterEach
    fun tearDown() {
        sim.close()
    }

    @Test
    fun testShooterWithVisualizer() {
        // Place 3 balls just ahead of the robot for easy intake
        sim.spawnBall(0.2f, 0.0f, 0.0635f, BallColor.GREEN)
        sim.spawnBall(0.25f, 0.05f, 0.0635f, BallColor.PURPLE)
        sim.spawnBall(0.25f, -0.05f, 0.0635f, BallColor.GREEN)

        val server = sigmacorns.sim.viz.SimVizServer(sim, 8081)
        server.start()

        println("Shooter visualizer running at http://localhost:8081")

        // Phase 1: Drive forward with intake to pick up balls (1 second)
        sim.driveFL = 0.3
        sim.driveBL = 0.3
        sim.driveBR = 0.3
        sim.driveFR = 0.3
        sim.intake = 1.0
        repeat(200) { i ->
            sim.update()
            if (i % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }
        sim.driveFL = 0.0
        sim.driveBL = 0.0
        sim.driveBR = 0.0
        sim.driveFR = 0.0
        sim.intake = 0.0

        println("Picked up ${sim.heldBalls.size} balls")

        // Phase 2: Spin up flywheel (2 seconds)
        sim.flywheel = 1.0
        repeat(400) { i ->
            sim.update()
            if (i % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }

        // Phase 3: Rotate turret left while shooting (servo=0.65 = ~54° left)
        sim.turret = 0.65
        val ballsToShoot = sim.heldBalls.size

        for (shot in 0 until ballsToShoot) {
            // Rotate turret for 500ms
            repeat(100) { i ->
                sim.update()
                if (i % 4 == 0) server.broadcastState()
                Thread.sleep(5)
            }

            println("Shot $shot: turret=${Math.toDegrees(sim.turretPosition())}°, " +
                    "flywheel=${(sim.flywheelVelocity() * 60 / (2 * Math.PI)).toInt()} RPM")
            sim.shootBall()

            // Let ball fly for 1 second
            repeat(200) { i ->
                sim.update()
                if (i % 4 == 0) server.broadcastState()
                Thread.sleep(5)
            }
        }

        // Phase 4: Drive forward while turret sweeps
        sim.driveFL = 0.3
        sim.driveBL = 0.3
        sim.driveBR = 0.3
        sim.driveFR = 0.3
        sim.turret = 0.25 // servo=0.25 targets -π/2 (sweep right)

        repeat(600) { i ->
            sim.update()
            if (i % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }

        // Assertions on final state
        val pos = sim.position()
        assertTrue(pos.v.x > 0.1, "Robot should have moved forward")
        assertEquals(0, sim.heldBalls.size, "All balls should have been shot")

        server.stop()
    }

    @Test
    fun testGoalScoringWithVisualizer() {
        val server = sigmacorns.sim.viz.SimVizServer(sim, 8082)
        server.start()

        println("Goal scoring visualizer running at http://localhost:8082")

        val halfField = 3.6576f / 2f
        val crampWidth = 0.16f
        val goalLeg = 0.6858f
        val crampStartH = 0.49f
        val goalTotalHeight = 1.3716f

        // Phase 1: Drop balls from above into the red goal scoring zone
        println("--- Phase 1: Dropping balls into red goal ---")
        for (i in 0 until 5) {
            val offset = 0.03f + i * 0.05f
            sim.spawnBall(
                halfField - crampWidth - offset,
                halfField - offset,
                goalTotalHeight - 0.05f + i * 0.1f,
                if (i % 2 == 0) BallColor.GREEN else BallColor.PURPLE
            )
        }

        repeat(600) { i ->
            sim.update()
            if (i % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }

        val gs1 = sim.getGoalState()
        println("Red score after drop: ${gs1.redScore}")

        // Phase 2: Drop balls onto the ramp — they roll down and stay at the gate
        println("--- Phase 2: Balls dropping onto ramp and rolling ---")
        val rampStartJoltX = -halfField + goalLeg
        val rampZ = halfField - crampWidth / 2f

        for (i in 0 until 3) {
            sim.spawnBall(
                rampZ,
                -rampStartJoltX - i * 0.12f,
                crampStartH + 0.3f + i * 0.15f,
                if (i % 2 == 0) BallColor.GREEN else BallColor.PURPLE
            )
        }

        repeat(1600) { i ->
            sim.update()
            if (i % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }

        val balls = sim.getBallStates()
        println("Balls remaining in ramp: ${balls.size}")
        for ((idx, b) in balls.withIndex()) {
            println("  Ball $idx: (${b.x}, ${b.y}, ${b.z})")
        }

        // Phase 3: Drop balls into the blue goal
        println("--- Phase 3: Dropping balls into blue goal ---")
        for (i in 0 until 3) {
            val offset = 0.03f + i * 0.05f
            sim.spawnBall(
                -(halfField - crampWidth - offset),
                halfField - offset,
                goalTotalHeight - 0.05f + i * 0.1f,
                BallColor.PURPLE
            )
        }

        repeat(600) { i ->
            sim.update()
            if (i % 4 == 0) server.broadcastState()
            Thread.sleep(5)
        }

        val gs2 = sim.getGoalState()
        println("Final scores — Red: ${gs2.redScore}, Blue: ${gs2.blueScore}")

        server.stop()
    }

    @Test
    fun testWithWasdDrive() {
        sim.spawnFieldBalls()

        val server = sigmacorns.sim.viz.SimVizServer(sim)
        server.start()

        val driveController = DriveController()

        println("Visualizer running at http://localhost:8080")
        println("Use WASD keys in the browser to drive. Press Ctrl+C to stop.")

        var frameCount = 0
        var prevF = false
        while (true) {
            val wasd = server.wasdState
            val vx = if (wasd.w) 1.0 else if (wasd.s) -1.0 else 0.0
            val vy = if (wasd.a) -1.0 else if (wasd.d) 1.0 else 0.0
            val omega = if (wasd.q) -1.0 else if (wasd.e) 1.0 else 0.0
            driveController.drive(Pose2d(vx, vy, omega), sim)
            sim.turret = 0.5

            // R = intake + flywheel, F = shoot one ball per press
            sim.intake = if (wasd.r) 1.0 else 0.0
            sim.flywheel = if (wasd.r) 1.0 else 0.0
            if (wasd.f && !prevF) sim.shootBall()
            prevF = wasd.f

            sim.update()
            frameCount++
            if (frameCount % 4 == 0) {
                server.broadcastState()
            }
            Thread.sleep(5)
        }
    }

    @Test
    fun testWithVisualizer() {
        sim.spawnFieldBalls()

        val server = sigmacorns.sim.viz.SimVizServer(sim)
        server.start()

        val gamepad = Gamepad()
        val simGamepad = SimGamepad(gamepad)
        val driveController = DriveController()

        println("Visualizer running at http://localhost:8080")
        println("Use a connected gamepad to drive. Press Ctrl+C to stop.")

        var frameCount = 0
        while (true) {
            simGamepad.tick()
            driveController.update(gamepad, sim)

            // Right trigger = intake, left bumper = flywheel, right bumper = shoot
            sim.intake = gamepad.right_trigger.toDouble()
            sim.flywheel = if (gamepad.left_bumper) 1.0 else 0.0
            if (gamepad.right_bumper) sim.shootBall()

            sim.update()
            frameCount++
            if (frameCount % 4 == 0) {
                server.broadcastState()
            }
            Thread.sleep(5)
        }
    }

    /**
     * Runs the full Robot.kt control stack on the Jolt sim.
     * Driver controls via gamepad (matching TeleopBase gm1 layout).
     * Operator controls via browser keyboard (matching TeleopBase gm2 layout).
     */
    @Test
    fun testWithRobot() {
        sim.spawnFieldBalls()

        val server = sigmacorns.sim.viz.SimVizServer(sim)
        server.start()

        val shotDataPath = javaClass.getResource("/shot_tuning_data.json")?.path
            ?: error("shot_tuning_data.json not found in test resources")
        val robot = Robot(sim, blue = true, shotDataPath = shotDataPath)
        robot.init(Pose2d(0.0, 0.0, 0.0), apriltagTracking = false)
        robot.aimFlywheel = false
        // Sim X axis is negated relative to real field — flip goal X for aiming
        robot.aim.goalPosition = org.joml.Vector2d(-robot.aim.goalPosition.x, robot.aim.goalPosition.y)
        robot.aim.targeting.goalPosition = robot.aim.goalPosition
        val gamepad = Gamepad()
        val simGamepad = SimGamepad(gamepad)

        println("Robot.kt sim running at http://localhost:8080")
        println("=== DRIVER (Gamepad) ===")
        println("  Sticks        = drive (dpad up/down = speed mode)")
        println("  Left trigger   = intake")
        println("  Right trigger  = shoot")
        println("  A              = emergency stop")
        println("  B              = force intake")
        println("  Left bumper    = flywheel spinup toggle")
        println("=== OPERATOR (Keyboard in browser) ===")
        println("  Q / E          = turret yaw left / right")
        println("  W / S          = manual flywheel power up / down")
        println("  1              = auto-aim toggle")
        println("  2              = flywheel spinup toggle")
        println("  3              = reset position")
        println("  R              = intake + flywheel (toggle)")
        println("  F              = shoot")

        // Edge detection state
        var wasShooting = false
        var wasLeftBumper = false
        var prevF = false
        var prevN1 = false
        var prevN2 = false
        var prevN3 = false

        // Operator state
        var manualFlywheelPower = 0.0

        var frameCount = 0
        val dtSeconds = JoltSimIO.SIM_UPDATE_TIME.toDouble(kotlin.time.DurationUnit.SECONDS)

        while (true) {
            simGamepad.tick()
            val kb = server.wasdState

            // === DRIVER (Gamepad - matching TeleopBase gm1) ===

            // Drivetrain (sticks + dpad speed mode handled by DriveController)
            robot.drive.update(gamepad, sim)

            // Intake - left trigger (matching TeleopBase)
            sim.intake = gamepad.left_trigger.toDouble()

            // Force intake - B button
            if (gamepad.b) sim.intake = 1.0

            // Shoot - right trigger with edge detection (matching TeleopBase)
            val isShooting = gamepad.right_trigger > 0.5
            if (isShooting && !wasShooting) sim.shootBall()
            wasShooting = isShooting

            // Flywheel spinup toggle - left bumper
            if (gamepad.left_bumper && !wasLeftBumper) {
                robot.aimFlywheel = !robot.aimFlywheel
                println("Flywheel aim: ${if (robot.aimFlywheel) "ON" else "OFF"}")
            }
            wasLeftBumper = gamepad.left_bumper

            // Emergency stop - A button (matching TeleopBase)
            if (gamepad.a) {
                sim.driveFL = 0.0; sim.driveBL = 0.0
                sim.driveFR = 0.0; sim.driveBR = 0.0
                sim.intake = 0.0; sim.flywheel = 0.0
                robot.aimFlywheel = false
                manualFlywheelPower = 0.0
            }

            // === OPERATOR (Keyboard - matching TeleopBase gm2) ===

            // Turret yaw - Q/E (matching gm2 left_stick_x)
            val yawInput = if (kb.q) 1.0 else if (kb.e) -1.0 else 0.0
            if (kotlin.math.abs(yawInput) > 0.1) {
                robot.aim.positionOverride = (robot.aim.positionOverride ?: 0.0) +
                    yawInput * Math.toRadians(60.0) * dtSeconds
            } else if (!kb.q && !kb.e) {
                // Clear override when no input (let auto-aim take over if enabled)
            }

            // Manual flywheel power - W/S (matching gm2 right_stick_y)
            if (kb.w) manualFlywheelPower = (manualFlywheelPower + 2.0 * dtSeconds).coerceAtMost(1.0)
            if (kb.s) manualFlywheelPower = (manualFlywheelPower - 2.0 * dtSeconds).coerceAtLeast(0.0)

            // Auto-aim toggle - 1 key (matching gm2 back)
            if (kb.n1 && !prevN1) {
                robot.aim.autoAim.enabled = !robot.aim.autoAim.enabled
                if (!robot.aim.autoAim.enabled) {
                    robot.aim.turret.fieldRelativeMode = true
                }
                println("Auto-aim: ${if (robot.aim.autoAim.enabled) "ON" else "OFF"}")
            }
            prevN1 = kb.n1

            // Flywheel spinup toggle - 2 key (matching gm2 left_bumper)
            if (kb.n2 && !prevN2) {
                robot.aimFlywheel = !robot.aimFlywheel
                println("Flywheel aim: ${if (robot.aimFlywheel) "ON" else "OFF"}")
            }
            prevN2 = kb.n2

            // Reset position - 3 key (matching gm2 Y)
            if (kb.n3 && !prevN3) {
                sim.setPosition(Pose2d(0.0, 0.0, 0.0))
                println("Position reset")
            }
            prevN3 = kb.n3

            // Keyboard intake + flywheel combo - R (toggle)
            if (kb.r) {
                sim.intake = 1.0
                robot.aimFlywheel = true
            }

            // Keyboard shoot - F
            if (kb.f && !prevF) sim.shootBall()
            prevF = kb.f

            // Manual flywheel power override (W/S keys)
            if (manualFlywheelPower > 0.01) {
                robot.aimFlywheel = false
                sim.flywheel = manualFlywheelPower
            }

            // Run Robot's update loop (turret PID, aiming, dispatcher)
            robot.update()

            sim.update()
            frameCount++
            if (frameCount % 4 == 0) {
                server.broadcastState()
            }
            Thread.sleep(5)
        }
    }

    /**
     * WASD-driven sim using the full Robot.kt control stack.
     * WASD+QE for driving, number keys and R/F for subsystems.
     */
    @Test
    fun testWithRobotWasd() {
        sim.spawnFieldBalls()

        val server = sigmacorns.sim.viz.SimVizServer(sim)
        server.start()

        val shotDataPath = javaClass.getResource("/shot_tuning_data.json")?.path
            ?: error("shot_tuning_data.json not found in test resources")
        val robot = Robot(sim, blue = true, shotDataPath = shotDataPath)
        robot.init(Pose2d(0.0, 0.0, 0.0), apriltagTracking = false)
        robot.aimFlywheel = false
        robot.aim.goalPosition = org.joml.Vector2d(-robot.aim.goalPosition.x, robot.aim.goalPosition.y)
        robot.aim.targeting.goalPosition = robot.aim.goalPosition

        println("Robot.kt WASD sim running at http://localhost:8080")
        println("=== DRIVER (Keyboard) ===")
        println("  WASD           = drive forward/left/back/right")
        println("  Q / E          = rotate left / right")
        println("=== SUBSYSTEMS (Keyboard) ===")
        println("  O / P          = turret yaw left / right")
        println("  R              = intake + flywheel (toggle)")
        println("  F              = shoot")
        println("  1              = auto-aim toggle")
        println("  2              = flywheel spinup toggle")
        println("  3              = reset position")

        var prevF = false
        var prevN1 = false
        var prevN2 = false
        var prevN3 = false
        val dtSeconds = JoltSimIO.SIM_UPDATE_TIME.toDouble(kotlin.time.DurationUnit.SECONDS)

        var frameCount = 0
        while (true) {
            val kb = server.wasdState
            val vx = if (kb.w) 1.0 else if (kb.s) -1.0 else 0.0
            val vy = if (kb.a) -1.0 else if (kb.d) 1.0 else 0.0
            val omega = if (kb.q) -0.3 else if (kb.e) 0.3 else 0.0

            robot.drive.drive(Pose2d(vx, vy, omega), sim)

            // Turret yaw - O/P (directly set robot-relative target)
            val yawInput = if (kb.o) 1.0 else if (kb.p) -1.0 else 0.0
            if (kotlin.math.abs(yawInput) > 0.1) {
                robot.aim.turret.fieldRelativeMode = false
                robot.aim.turret.targetAngle += yawInput * Math.toRadians(60.0) * dtSeconds
            }

            // Intake + flywheel combo - R (toggle)
            if (kb.r) {
                sim.intake = 1.0
                robot.aimFlywheel = true
            } else {
                sim.intake = 0.0
            }

            // Shoot - F
            if (kb.f && !prevF) sim.shootBall()
            prevF = kb.f

            // Auto-aim toggle - 1
            if (kb.n1 && !prevN1) {
                robot.aim.autoAim.enabled = !robot.aim.autoAim.enabled
                if (!robot.aim.autoAim.enabled) {
                    robot.aim.turret.fieldRelativeMode = true
                }
                println("Auto-aim: ${if (robot.aim.autoAim.enabled) "ON" else "OFF"}")
            }
            prevN1 = kb.n1

            // Flywheel spinup toggle - 2
            if (kb.n2 && !prevN2) {
                robot.aimFlywheel = !robot.aimFlywheel
                println("Flywheel aim: ${if (robot.aimFlywheel) "ON" else "OFF"}")
            }
            prevN2 = kb.n2

            // Reset position - 3
            if (kb.n3 && !prevN3) {
                sim.setPosition(Pose2d(0.0, 0.0, 0.0))
                println("Position reset")
            }
            prevN3 = kb.n3

            robot.update()

            sim.update()
            frameCount++
            if (frameCount % 4 == 0) {
                server.broadcastState()
            }
            Thread.sleep(5)
        }
    }
}
