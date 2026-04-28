package sigmacorns.test

import com.qualcomm.robotcore.hardware.Gamepad
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import sigmacorns.io.BallColor
import sigmacorns.io.JoltSimIO
import sigmacorns.sim.SimGamepad
import sigmacorns.subsystem.Drivetrain
import kotlin.math.abs

class JoltSimTest {
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
    fun testBasicPhysics() {
        // Drive forward (positive FL,BL,BR,FR powers)
        sim.driveFL = 0.5
        sim.driveBL = 0.5
        sim.driveBR = 0.5
        sim.driveFR = 0.5

        // Run for 500ms
        repeat(100) { sim.update() }

        val pos = sim.position()
        // Robot should have moved in some direction
        val distSq = pos.v.x * pos.v.x + pos.v.y * pos.v.y
        assertTrue(distSq > 0.001, "Robot should have moved, pos=$pos")
    }

    @Test
    fun testDrivetrain() {
        // Forward: all motors positive should move in +x (forward, same direction as intake)
        sim.driveFL = 0.5
        sim.driveBL = 0.5
        sim.driveBR = 0.5
        sim.driveFR = 0.5
        repeat(100) { sim.update() }

        var pos = sim.position()
        println("Forward drive: pos=(${pos.v.x}, ${pos.v.y}), heading=${pos.rot}")
        assertTrue(pos.v.x > 0.01, "Forward drive should move in +x, got x=${pos.v.x}")
        assertTrue(abs(pos.v.y) < abs(pos.v.x) * 0.1, "Forward drive should not strafe, y=${pos.v.y} vs x=${pos.v.x}")

        // Reset for strafe test
        sim.close()
        sim = JoltSimIO()

        // Strafe left: FL=-1, BL=+1, BR=-1, FR=+1 should move in +y
        sim.driveFL = -0.5
        sim.driveBL = 0.5
        sim.driveBR = -0.5
        sim.driveFR = 0.5
        repeat(100) { sim.update() }

        pos = sim.position()
        println("Strafe left: pos=(${pos.v.x}, ${pos.v.y}), heading=${pos.rot}")
        assertTrue(pos.v.y > 0.01, "Strafe left should move in +y, got y=${pos.v.y}")
        assertTrue(abs(pos.v.x) < abs(pos.v.y) * 0.1, "Strafe left should not move forward, x=${pos.v.x} vs y=${pos.v.y}")

        // Reset for turn test
        sim.close()
        sim = JoltSimIO()

        // Turn left (CCW): FL=-1, BL=-1, BR=+1, FR=+1 should give positive omega
        sim.driveFL = -0.5
        sim.driveBL = -0.5
        sim.driveBR = 0.5
        sim.driveFR = 0.5
        repeat(100) { sim.update() }

        pos = sim.position()
        println("Turn left: pos=(${pos.v.x}, ${pos.v.y}), heading=${pos.rot}")
        assertTrue(pos.rot > 0.01, "Turn left should give positive heading, got=${pos.rot}")
    }

    @Test
    fun testBallSpawning() {
        sim.spawnBall(0.5f, 0.5f, 0.0635f, BallColor.GREEN)
        sim.spawnBall(-0.5f, 0.5f, 0.0635f, BallColor.PURPLE)

        assertEquals(2, sim.getBallCount())

        val balls = sim.getBallStates()
        assertEquals(2, balls.size)
        assertEquals(BallColor.GREEN, balls[0].color)
        assertEquals(BallColor.PURPLE, balls[1].color)
    }

    @Test
    fun testFieldBallSpawning() {
        sim.spawnFieldBalls()
        assertEquals(18, sim.getBallCount(), "Should spawn 18 field balls")
    }

    @Test
    fun testIntakePickup() {
        // Place a ball right at the intake sensor area
        sim.spawnBall(0.0f, 0.2f, 0.0635f, BallColor.GREEN)
        assertEquals(1, sim.getBallCount())

        // Turn on intake and step
        sim.intake = 1.0
        repeat(10) { sim.update() }

        // Ball should be picked up
        assertTrue(sim.heldBalls.size > 0 || sim.getBallCount() == 0,
            "Ball should be picked up by intake or still in field")
    }

    @Test
    fun testWallCollision() {
        // Place robot near the +X wall
        sim.setPosition(sigmacorns.math.Pose2d(1.7, 0.0, 0.0))

        // Drive toward the wall (all wheels forward = robot moves in +Z in Jolt = +Y in sim...
        // Actually depends on heading. At heading=0, forward is +Z in Jolt = +Y in sim
        // Let's drive with heading facing the wall
        sim.setPosition(sigmacorns.math.Pose2d(1.6, 0.0, 0.0))

        // Drive hard toward +X (strafe with heading=0)
        sim.driveFL = 1.0
        sim.driveBL = -1.0
        sim.driveBR = -1.0
        sim.driveFR = 1.0

        // Run for 2 seconds
        repeat(400) { sim.update() }

        val pos = sim.position()
        val halfField = 3.6576 / 2.0
        assertTrue(pos.v.x < halfField + 0.01,
            "Robot should not pass through wall, x=${pos.v.x}, limit=$halfField")
    }

    @Test
    fun testMaxHeldBalls() {
        // Spawn 5 balls near intake
        for (i in 0 until 5) {
            sim.spawnBall(0.0f, 0.15f + i * 0.01f, 0.0635f, BallColor.GREEN)
        }

        sim.intake = 1.0
        repeat(200) { sim.update() }

        assertTrue(sim.heldBalls.size <= 3, "Should hold at most 3 balls, held=${sim.heldBalls.size}")
    }

    @Test
    fun testFlywheelSpinup() {
        // Flywheel should spin up when given power
        sim.flywheel = 1.0
        repeat(200) { sim.update() } // 1 second

        val rpm = sim.flywheelVelocity() * 60.0 / (2.0 * Math.PI)
        println("Flywheel after 1s at full power: ${rpm.toInt()} RPM (${sim.flywheelVelocity()} rad/s)")
        assertTrue(sim.flywheelVelocity() > 100.0, "Flywheel should spin up, got ${sim.flywheelVelocity()} rad/s")

        // Should slow down when power removed
        sim.flywheel = 0.0
        val peakVel = sim.flywheelVelocity()
        repeat(400) { sim.update() } // 2 more seconds
        assertTrue(sim.flywheelVelocity() < peakVel, "Flywheel should slow down, peak=$peakVel, now=${sim.flywheelVelocity()}")
    }

    @Test
    fun testTurretRotation() {
        // Servo turret: 0..1 maps to -π..+π target position
        // Setting turretRight=1.0 targets +π (full right)
        sim.turretRight = 1.0
        repeat(100) { sim.update() } // 500ms

        val angle = sim.turretPosition()
        println("Turret after 500ms at servo=1.0: ${Math.toDegrees(angle)}°")
        assertTrue(angle > 0.1, "Turret should have rotated positively, got $angle rad")

        // Turret should clamp to ±π (SERVO_TURRET_RANGE/2)
        repeat(400) { sim.update() } // 2 more seconds
        val clampedAngle = sim.turretPosition()
        assertTrue(clampedAngle <= Math.PI + 0.001,
            "Turret should clamp to π, got $clampedAngle rad")
    }

    @Test
    fun testShooterLaunch() {
        // Give the robot a ball
        sim.heldBalls.add(BallColor.GREEN)
        assertEquals(1, sim.heldBalls.size)

        // Spin up flywheel
        sim.flywheel = 1.0
        repeat(400) { sim.update() } // 2 seconds to spin up
        assertTrue(sim.flywheelVelocity() > 200.0, "Flywheel should be spun up")

        // Shoot the ball (default hood angle = 45°, turret at 0 = forward)
        val ballCountBefore = sim.getBallCount()
        sim.shootBall()
        assertEquals(0, sim.heldBalls.size, "Ball should be removed from held")
        assertEquals(ballCountBefore + 1, sim.getBallCount(), "Ball should be spawned in world")

        // Let the ball fly for a bit
        repeat(20) { sim.update() } // 100ms
        val balls = sim.getBallStates()
        assertTrue(balls.isNotEmpty(), "Ball should exist in world")

        val ball = balls.last()
        println("Shot ball position: (${ball.x}, ${ball.y}, ${ball.z})")
        // Ball should have moved forward (+x in sim) since turret=0 and heading=0
        assertTrue(ball.x > 0.3, "Ball should have moved forward, x=${ball.x}")
        // Ball should have gone upward (z > ground level)
        assertTrue(ball.z > 0.1, "Ball should be in the air, z=${ball.z}")
    }

    @Test
    fun testShooterTurretDirection() {
        // Rotate turret 90° left, shoot, verify ball goes in +y direction
        // servo=0.75 targets +π/2 (90° left)
        sim.turretRight = 0.75
        repeat(200) { sim.update() } // let turret settle
        sim.turretRight = 0.75 // hold position

        val turretAngle = sim.turretPosition()
        println("Turret angle: ${Math.toDegrees(turretAngle)}°")
        assertTrue(turretAngle > 0.5, "Turret should have rotated")

        // Spin up and shoot
        sim.heldBalls.add(BallColor.PURPLE)
        sim.flywheel = 1.0
        repeat(400) { sim.update() }
        sim.shootBall()

        // Let ball fly
        repeat(40) { sim.update() }
        val balls = sim.getBallStates()
        assertTrue(balls.isNotEmpty(), "Ball should exist")

        val ball = balls.last()
        println("Turret-rotated shot: (${ball.x}, ${ball.y}, ${ball.z})")
        // With turret rotated left (~90°), ball should move mostly in +y
        assertTrue(ball.y > 0.2, "Ball should move in +y (left) with turret rotated, y=${ball.y}")
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
    fun testWithVisualizer() {
        sim.spawnFieldBalls()

        val server = sigmacorns.sim.viz.SimVizServer(sim)
        server.start()

        val gamepad = Gamepad()
        val simGamepad = SimGamepad(gamepad)
        val drivetrain = Drivetrain()

        // Start in ROBOT-CENTRIC mode (like the real robot)
        drivetrain.fieldCentric = false

        var lastRB = false
        var lastLB = false
        var lastX = false

        println("Visualizer running at http://localhost:8080")
        println("Controls (GAMEPAD 1):")
        println("  Left Stick       - Drive forward/strafe")
        println("  Right Stick X   - Rotate")
        println("  D-pad Up/Down  - Full/Precision speed")
        println("  X              - Toggle field-centric")
        println("  Left Trigger   - Intake")
        println("  Right Trigger  - Shoot")
        println("  B              - Reverse intake (spit)")
        println("  Right Bumper   - Shoot far zone")
        println("  Left Bumper   - Shoot goal zone")
        println("Press Ctrl+C to stop.")

        var frameCount = 0
        while (true) {
            simGamepad.tick()

            // Drive from gamepad
            drivetrain.update(gamepad, sim)

            // Speed mode toggle (dpad up/down)
            if (gamepad.dpad_up) {
                // Full speed - handled in Drivetrain
            } else if (gamepad.dpad_down) {
                // Precision mode - handled in Drivetrain
            }

            // Toggle field-centric (X button)
            if (gamepad.x && !lastX) {
                drivetrain.fieldCentric = !drivetrain.fieldCentric
                println("Field-centric: ${drivetrain.fieldCentric}")
            }
            lastX = gamepad.x

            // Intake (left trigger)
            sim.intake = gamepad.left_trigger.toDouble()

            // Reverse intake (B button)
            if (gamepad.b) {
                sim.intake = -1.0
            }

            // Shoot (right trigger = shoot with flywheel)
            val rightTriggerActive = gamepad.right_trigger > 0.1f
            if (rightTriggerActive && sim.heldBalls.isNotEmpty()) {
                sim.flywheel = 1.0
                if (!lastRB && gamepad.right_bumper) {
                    // Shoot far zone (just trigger, don't shoot here)
                    lastRB = true
                } else if (!lastLB && gamepad.left_bumper) {
                    // Shoot goal zone (just trigger, don't shoot here)
                    lastLB = true
                } else {
                    sim.shootBall()
                }
            }
            lastRB = gamepad.right_bumper
            lastLB = gamepad.left_bumper

            // Flywheel spin-up only (left bumper held without right trigger)
            if (gamepad.left_bumper && !rightTriggerActive) {
                sim.flywheel = 1.0
            } else if (!rightTriggerActive) {
                sim.flywheel = 0.0
            }

            sim.update()
            frameCount++
            if (frameCount % 4 == 0) {
                server.broadcastState()
            }
            Thread.sleep(5)
        }
    }
}
