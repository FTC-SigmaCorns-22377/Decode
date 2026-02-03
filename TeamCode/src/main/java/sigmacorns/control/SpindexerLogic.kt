package sigmacorns.control

import com.bylazar.configurables.annotations.Configurable
import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.delay
import sigmacorns.sim.Balls
import sigmacorns.io.SigmaIO
import sigmacorns.opmode.tune.ShooterFlywheelPIDConfig
import kotlin.math.sign
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds


@Configurable
object ShotPowers {
    @JvmField var shortShotPower = 0.6
    @JvmField var midShotPower = 0.75
    @JvmField var longShotPower = 0.94
    @JvmField var shortDistanceLimit = 1.0
    @JvmField var midDistanceLimit = 2.3
}

class SpindexerLogic(val io: SigmaIO) {
    // Spindexer state tracking
    var spindexerRotation: Double = 0.0
    var spindexerState: MutableList<Balls?> = mutableListOf(null, null, null)

    // Constants
    private val PI = kotlin.math.PI
    internal val ROTATE_ANGLE = (2 * PI) / 3  // rotation for cycling to next ball
    internal val MODE_CHANGE_ANGLE = PI / 3   // rotation for switching intake to shoot mode


    // PID controller for flywheel
    val flywheelPID = PIDController(
        ShooterFlywheelPIDConfig.kP,
        ShooterFlywheelPIDConfig.kD,
        ShooterFlywheelPIDConfig.kI,
        0.0
    )

    val spindexer = Spindexer(MotorRangeMapper(
        0.0..2.0*PI,
        0.0..(((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0,
        Double.POSITIVE_INFINITY
    ),io)

    // Error thresholds
    internal val POSITION_ERROR_THRESHOLD = 0.05  // radians - position threshold for spindexer
    internal val VELOCITY_ERROR_THRESHOLD = 20.0   // rad/s - velocity threshold for flywheel

    // Timeout for safety (fallback if threshold never reached)
    internal val MAX_WAIT_TIME = 3000.milliseconds

    // Transfer servo timing constants (continuous servo: -1 to 1, 0 = stopped)
    private val TRANSFER_UP_POWER = 1.0       // Power to move transfer up (towards shooter)
    private val TRANSFER_DOWN_POWER = -1.0    // Power to move transfer down (reset position)
    private val TRANSFER_STOP_POWER = 0.0     // Stopped
    private val TRANSFER_UP_DURATION = 300.milliseconds    // Time to transfer ball up
    private val TRANSFER_DOWN_DURATION = 500.milliseconds  // Time to reset transfer down

    //extra variables
    internal var offsetActive: Boolean = false
    private var transferNeedsReset: Boolean = true  // Track if transfer needs to be reset
    private var nudgeDirection: Double = 1.0  // Track the direction of the last nudge (1.0 for forward/CW, -1.0 for backward/CCW)
    private var sortCycle: Int = 0 //Tracks the number of iterations the sorting has been through
    /** Current calculated target velocity for flywheel */
    var targetFlywheelVelocity: Double = 0.0

    /** Target power fraction for the shot */
    var targetShotPower: Double = 0.0
    var motif: List<Balls?> = listOf(Balls.Green, Balls.Green, Balls.Purple)

    /** Whether continuous shooting is requested */
    var shootingRequested: Boolean = false

    /** The motif the sorted shooter will target. */
    internal var targetMotif: List<Balls?> = listOf(Balls.Green, Balls.Green, Balls.Purple)


    enum class State {
        IDLE,
        INTAKING,
        MOVING,
        FULL,
        SHOOTING,
        MOVING_SHOOT,
        SORTED_SHOOTING
    }

    enum class Event {
        START_INTAKING,
        STOP_INTAKING,
        SHOOT,
        SORTED_SHOOT,
        BALL_DETECTED
    }

    val fsm = FSM<State, Event>(
        initialState = State.IDLE,
        behaviors = { state -> stateBehavior(state) },
        io = io
    ).apply {
        // Event: start intaking (user input)
        onEvent(Event.START_INTAKING) {
            if (curState == State.IDLE || curState == State.FULL) {
                curState = State.INTAKING
            }

            if(curState == State.SHOOTING) {
                if(transferNeedsReset) resetTransfer()
                curState = State.INTAKING
            }
        }

        // Event: stop intaking (user input)
        onEvent(Event.STOP_INTAKING) {
            if (curState == State.INTAKING) {
                curState = State.IDLE
            }
        }

        // Event: shoot (user input)
        onEvent(Event.SHOOT) {
            when (curState) {
                State.IDLE, State.FULL -> {
                    curState = State.MOVING_SHOOT
                }
                State.INTAKING, State.MOVING -> {
                    // Cancel intaking when user requests shooting
                    io.intake = 0.0
                    curState = State.MOVING_SHOOT
                }
                else -> {}
            }
        }
        onEvent(Event.BALL_DETECTED) {
            if(curState == State.INTAKING) {
                spindexerState[0] = Balls.Green
            }
        }
        // Event: sorted shoot (new)
        onEvent(Event.SORTED_SHOOT) {
            when (curState) {
                State.IDLE, State.FULL -> {
                    curState = State.SORTED_SHOOTING
                }
                State.INTAKING, State.MOVING -> {
                    io.intake = 0.0
                    curState = State.SORTED_SHOOTING
                }
                else -> {}
            }
        }
    }

    private fun stateBehavior(state: State): suspend () -> State = when (state) {
        State.IDLE -> idleBehavior()
        State.INTAKING -> intakingBehavior()
        State.MOVING -> movingBehavior()
        State.FULL -> fullBehavior()
        State.SHOOTING -> shootingBehavior()
        State.MOVING_SHOOT -> movingShootBehavior()
        State.SORTED_SHOOTING -> sortedShootingBehavior()  // ← ADD THIS LINE
    }

    private fun idleBehavior(): suspend () -> State = suspend {
        // Idle: wait for events, no automatic transitions
        io.intake = 0.0
        io.shooter = 0.0
        io.transfer = TRANSFER_STOP_POWER

        // Reset transfer on first idle if needed
        if (transferNeedsReset) {
            resetTransfer()
        }

        // Wait indefinitely until cancelled by an event-triggered state change
        delay(Long.MAX_VALUE)
        State.IDLE
    }

    /** Reset the transfer servo by running it down for a set duration */
    internal suspend fun resetTransfer() {
        io.transfer = TRANSFER_DOWN_POWER
        delay(TRANSFER_DOWN_DURATION.inWholeMilliseconds)
        io.transfer = TRANSFER_STOP_POWER
        transferNeedsReset = false
    }

    /** Activate transfer servo by running it up for a set duration to transfer a ball */
    internal suspend fun activateTransfer() {
        io.transfer = TRANSFER_UP_POWER
        delay(TRANSFER_UP_DURATION.inWholeMilliseconds)
        io.transfer = TRANSFER_STOP_POWER
        transferNeedsReset = true  // Mark that we need to reset next time
    }

    private fun intakingBehavior(): suspend () -> State = suspend {
        // Run intake motor
        println("Entered intaking state")
        io.intake = -1.0
        if (offsetActive == true) {
            println("changed offset angle")
            spindexerRotation += MODE_CHANGE_ANGLE
            offsetActive = false

            // Wait until spindexer reaches target position
            val startTime = io.time()
            while (true) {
                val error = kotlin.math.abs(spindexer.curRotation - spindexerRotation)

                if (error < POSITION_ERROR_THRESHOLD) {
                    break
                }
                if (io.time() - startTime > MAX_WAIT_TIME) {
                    println("Warning: Spindexer offset timeout")
                    break
                }
                delay(10)
            }
        }

        // Check for ball detection (polling)
        while (spindexerState[0] == null) {
            // Poll color sensor for automatic ball detection
            val ballDetected = io.colorSensorDetectsBall()
            if (ballDetected) {
                spindexerState[0] = io.colorSensorGetBallColor() ?: Balls.Green
                break
            }
            delay(10)
        }

        // Ball detected -> transition to MOVING
        State.MOVING
    }

    private fun movingBehavior(): suspend () -> State = suspend {
        // Stop intake while moving
        io.intake = 0.0

        // Rotate spindexer to next position based on nudge direction
        val rotationAngle = nudgeDirection * ROTATE_ANGLE
        spindexerRotation += rotationAngle

        // Shift balls in spindexer based on rotation direction
        if (nudgeDirection > 0) {
            val temp = spindexerState[2]
            spindexerState[2] = spindexerState[1]
            spindexerState[1] = spindexerState[0]
            spindexerState[0] = temp
        } else {
            val temp = spindexerState[0]
            spindexerState[0] = spindexerState[1]
            spindexerState[1] = spindexerState[2]
            spindexerState[2] = temp
        }

        // Wait for spindexer to reach target position
        val startTime = io.time()
        while (true) {
            val error = kotlin.math.abs(spindexer.curRotation - spindexerRotation)

            if (error < POSITION_ERROR_THRESHOLD) {
                break
            }
            if (io.time() - startTime > MAX_WAIT_TIME) {
                println("Warning: Spindexer movement timeout")
                break
            }
            delay(10)
        }

        // Check if spindexer is full
        if (spindexerState.all { it != null }) {
            State.FULL
        } else {
            State.IDLE
        }
    }

    private fun fullBehavior(): suspend () -> State = suspend {
        // Full: wait for shoot event, no automatic transitions
        io.intake = 0.0
        delay(Long.MAX_VALUE)
        State.FULL
    }
    var foundAnyBall = false
    var requiredColor = motif[0]
    private suspend fun sortBehavior() {
        if (sortCycle == 0) {
            requiredColor = motif[0]
        }
        if (sortCycle == 1) {
            requiredColor = motif[1]
        }
        if (sortCycle == 2) {
            requiredColor = motif[2]
        }

        // Check if there are any balls left at all
        foundAnyBall = false
//        for (checkRotation in 0..2) {
//            if (io.colorSensorDetectsBall()) {
//                foundAnyBall = true
//                break
//            }
//            if (checkRotation < 2) {
//                movingBehavior()()
//            }
//
//
//            // If no balls found anywhere, we're done
//            if (!foundAnyBall) {
//                println("Sorted shoot: No balls remaining")
//                break
//            }

            // If we need a specific color, rotate until we find it
            if (requiredColor != null) {
                var rotations = 0
                var foundColor = false

                while (rotations < 3) {
                    if (io.colorSensorDetectsBall()) {
                        val currentColor = io.colorSensorGetBallColor()
                        println("SORT: Found Ball: $currentColor Required Color: $requiredColor")
                        if (currentColor == requiredColor) {
                            foundColor = true
                            break
                        }
                    }

                    // Try next slot
                    if (rotations < 2) {
                        movingBehavior()()
                        rotations++
                    } else {
                        break
                    }
                }

                // If we didn't find the required color, skip this motif slot
                if (!foundColor) {
                    println("Sorted shoot: Color $requiredColor not found, skipping")

                    //continue
                }
            }
    }

    private fun shootingBehavior(): suspend () -> State = suspend {
        // Flywheel control is handled in update() based on state


        //add shooting logic here

        // Wait for flywheel to spin up
        val startTime = io.time()
        while (true) {
            val currentVelocity = io.flywheelVelocity()
            // Target is calculated in update()
            val error = kotlin.math.abs(currentVelocity - (targetShotPower * ShooterFlywheelPIDConfig.maxVelocity))

            if (error < VELOCITY_ERROR_THRESHOLD) {
                break
            }
            if (io.time() - startTime > MAX_WAIT_TIME) {
                println("Warning: Flywheel spinup timeout")
                break
            }
            delay(10)
        }

        // Activate transfer to shoot ball (continuous servo - runs for set duration)
        transferNeedsReset = true
        activateTransfer()

        // Mark current ball as shot
        spindexerState[0] = null

        // Check if spindexer is empty
        if (spindexerState.all { it == null } && !shootingRequested) {
            io.shooter = 0.0
            State.IDLE
        } else {
            State.MOVING_SHOOT
        }
    }

    private fun movingShootBehavior(): suspend () -> State = suspend {
        if (!offsetActive) {
            spindexerRotation += MODE_CHANGE_ANGLE
            offsetActive = true

            // Wait until spindexer reaches target position
            val startTime = io.time()
            while (true) {
                val error = kotlin.math.abs(spindexer.curRotation - spindexerRotation)

                if (error < POSITION_ERROR_THRESHOLD) {
                    break
                }
                if (io.time() - startTime > MAX_WAIT_TIME) {
                    println("Warning: Spindexer offset timeout")
                    break
                }
                delay(10)
            }
        }

        resetTransfer()

        // Continue shooting if requested, otherwise go to IDLE
        if (shootingRequested) {
            sortBehavior()
            sortCycle =+ 1
            State.SHOOTING
        } else {
            sortCycle = 0
            State.IDLE
        }
    }

    // Public API for sending events
    fun startIntaking() = fsm.sendEvent(Event.START_INTAKING)
    fun stopIntaking() = fsm.sendEvent(Event.STOP_INTAKING)
    fun shoot() = fsm.sendEvent(Event.SHOOT)
    fun rotate() = fsm.sendEvent(Event.BALL_DETECTED)
    fun sortedShoot() = fsm.sendEvent(Event.SORTED_SHOOT)

    fun nudge(ccw: Boolean) {
        if (currentState == State.INTAKING || currentState == State.IDLE || currentState == State.FULL) {
//            spindexerState[0] = Balls.Purple
            nudgeDirection = if(ccw) 1.0 else -1.0

            // Transition to MOVING state to execute the nudge with directionality
            fsm.curState = State.MOVING
        }
    }

    // Current state accessor
    val currentState: State get() = fsm.curState

    private fun sortedShootingBehavior(): suspend () -> State = suspend {
        sortedShoot(targetMotif)
        State.IDLE
    }

    /**
     * Public API to start sorted shooting.
     */
    fun startSortedShoot(motif: List<Balls?>) {
        targetMotif = motif
        fsm.sendEvent(Event.SORTED_SHOOT)
    }

    fun update(motorTick: Double, deltaT: Duration, dVoltage: Double = 1.0) {
        fsm.update()

        // Update spindexer control
        spindexer.target = spindexerRotation
        spindexer.update(deltaT)

        // Update Flywheel PID if in shooting state
        if (fsm.curState == State.SHOOTING || fsm.curState == State.MOVING_SHOOT) {
            // Update PID gains
            flywheelPID.kp = ShooterFlywheelPIDConfig.kP
            flywheelPID.kd = ShooterFlywheelPIDConfig.kD
            flywheelPID.ki = ShooterFlywheelPIDConfig.kI

            // Calculate target
            val maxVel = ShooterFlywheelPIDConfig.maxVelocity
            targetFlywheelVelocity = targetShotPower * maxVel
            flywheelPID.setpoint = targetFlywheelVelocity

            // Calculate output
            val currentVel = io.flywheelVelocity()
            val pidOut = flywheelPID.update(currentVel, deltaT)
            val feedforward = targetFlywheelVelocity / maxVel

            // Apply power with voltage compensation
            // Note: feedforward is 0-1, pidOut is theoretically unbounded but usually small
            // We want (ff + pid) scaled by voltage
            val totalPower = (feedforward + pidOut).coerceIn(-1.0, 1.0)
            io.shooter = totalPower * dVoltage
        }
    }
}
