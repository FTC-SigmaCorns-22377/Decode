package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import sigmacorns.control.AutoAim
import sigmacorns.control.MotorRangeMapper
import sigmacorns.control.PollableDispatcher
import sigmacorns.control.SpindexerLogic
import sigmacorns.control.Turret
import sigmacorns.io.HardwareIO
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.sim.Balls
import kotlin.math.PI
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "Simple Auto", group = "Auto")
class SimpleAuto : SigmaOpMode() {

    private lateinit var spindexerLogic: SpindexerLogic
    private lateinit var dispatcher: PollableDispatcher
    private lateinit var autoAim: AutoAim
    private lateinit var turret: Turret

    private val ticksPerRad = (1.0 + (46.0 / 11.0)) * 28.0 / (2 * PI) * 76 / 19
    private val turretRange = MotorRangeMapper(
        limits = -PI / 2.0..PI / 2.0,
        limitsTick = -PI / 2.0 * ticksPerRad..PI / 2.0 * ticksPerRad,
        slowdownDist = 0.3
    )

    override fun runOpMode() {
        val voltageSensor = hardwareMap.voltageSensor.iterator().next()

        spindexerLogic = SpindexerLogic(io)
        dispatcher = PollableDispatcher(io)
        turret = Turret(turretRange, io)

        // Initialize auto-aim with limelight from HardwareIO
        val hardwareIO = io as? HardwareIO
        autoAim = AutoAim(hardwareIO?.limelight, targetAprilTagIds = setOf(20))
        autoAim.configure()
        autoAim.enabled = true

        // Preload spindexer state with balls
        spindexerLogic.spindexerState[0] = Balls.Green
        spindexerLogic.spindexerState[1] = Balls.Green
        spindexerLogic.spindexerState[2] = Balls.Green

        val scope = CoroutineScope(dispatcher)

        // Define the auto sequence
        scope.launch {
            println("Auto: Starting sequence")

            // Wait for auto-aim to acquire target (up to 2 seconds)
            var aimAttempts = 0
            while (!autoAim.hasTarget && aimAttempts < 40) {
                delay(50)
                aimAttempts++
            }

            if (autoAim.hasTarget) {
                println("Auto: Target acquired at ${autoAim.targetDistance}m")
            } else {
                println("Auto: No target found, shooting anyway")
            }

            // Shoot until empty
            while (spindexerLogic.spindexerState.any { it != null }) {
                spindexerLogic.shootingRequested = true
                spindexerLogic.shoot()
                delay(100)
            }

            println("Auto: Finished shooting")
            spindexerLogic.shootingRequested = false

            // Move sideways (Strafe Right)
            val strafePower = 0.5
            io.driveFL = strafePower
            io.driveFR = -strafePower
            io.driveBL = -strafePower
            io.driveBR = strafePower

            delay(2000)

            // Stop
            io.driveFL = 0.0
            io.driveFR = 0.0
            io.driveBL = 0.0
            io.driveBR = 0.0

            println("Auto: Sequence complete")
        }

        waitForStart()

        ioLoop { state, dt ->
            val voltage = voltageSensor.voltage
            val dVoltage = 12.0 / voltage

            // Update dispatcher (runs the coroutine)
            dispatcher.update()

            // Update auto-aim with current robot pose and turret angle
            val robotPose = io.position()
            val turretAngle = turret.pos
            autoAim.update(robotPose, turretAngle)

            // Update turret heading for field-relative aiming
            turret.robotHeading = robotPose.rot
            turret.robotAngularVelocity = io.velocity().rot

            // Auto-aim turret control
            if (autoAim.enabled && autoAim.hasTarget) {
                autoAim.getTargetTurretAngle()?.let { robotAngle ->
                    turret.targetAngle = robotAngle
                }
            }

            // Update turret PID
            turret.update(dt)

            // Update spindexer logic
            spindexerLogic.update(io.spindexerPosition(), dt, dVoltage)

            telemetry.addData("State", spindexerLogic.currentState)
            telemetry.addData("Balls", spindexerLogic.spindexerState.count { it != null })
            telemetry.addData("AutoAim", if (autoAim.hasTarget) "LOCKED" else "SEARCHING")
            telemetry.addData("Target Dist", "%.2f m", autoAim.targetDistance)
            telemetry.addData("Turret Angle", "%.1f deg", Math.toDegrees(turret.pos))
            telemetry.update()

            false
        }
    }
}
