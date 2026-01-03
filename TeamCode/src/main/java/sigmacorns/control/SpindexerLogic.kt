package sigmacorns.control

import com.bylazar.configurables.annotations.Configurable
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.delay
import sigmacorns.control.ShotPowers.shotPower
import sigmacorns.sim.Balls
import sigmacorns.io.SigmaIO
import sigmacorns.opmode.test.SpindexerPIDConfig
import sigmacorns.opmode.test.ShooterFlywheelPIDConfig
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds


@Configurable
object ShotPowers {
    @JvmField
    var shotPower = 0.8
}

class SpindexerLogic(val io: SigmaIO) {
    // Spindexer state tracking
    var spindexerRotation: Double = 0.0
    var spindexerState: MutableList<Balls?> = mutableListOf(null, null, null)

    // Constants
    private val PI = kotlin.math.PI
    private val ROTATE_ANGLE = (2 * PI) / 3  // rotation for cycling to next ball
    private val MODE_CHANGE_ANGLE = PI / 3   // rotation for switching intake to shoot mode

    // PID controller for spindexer motor
    val spindexerPID = PIDController(0.25, 10.0, 0.0, 0.0)
    
    // PID controller for flywheel
    val flywheelPID = PIDController(
        ShooterFlywheelPIDConfig.kP,
        ShooterFlywheelPIDConfig.kD,
        ShooterFlywheelPIDConfig.kI,
        0.0
    )
    
    private val slewRateLimiter = SlewRateLimiter(maxRate = SpindexerPIDConfig.slewRate)

    /** Whether slew rate limiting is enabled */
    var slewRateLimitingEnabled: Boolean = true

    /** The actual target after limiting */
    var effectiveTargetRotation: Double = 0.0
        private set

    // Error thresholds
    private val POSITION_ERROR_THRESHOLD = 0.05  // radians - position threshold for spindexer
    private val VELOCITY_ERROR_THRESHOLD = 20.0   // rad/s - velocity threshold for flywheel

    // Timeout for safety (fallback if threshold never reached)
    private val MAX_WAIT_TIME = 3000.milliseconds

    // Transfer servo timing constants (continuous servo: -1 to 1, 0 = stopped)
    private val TRANSFER_UP_POWER = 1.0       // Power to move transfer up (towards shooter)
    private val TRANSFER_DOWN_POWER = -1.0    // Power to move transfer down (reset position)
    private val TRANSFER_STOP_POWER = 0.0     // Stopped
    private val TRANSFER_UP_DURATION = 300.milliseconds    // Time to transfer ball up
    private val TRANSFER_DOWN_DURATION = 500.milliseconds  // Time to reset transfer down

    //extra variables
    private var offsetActive: Boolean = false
    private var transferNeedsReset: Boolean = true  // Track if transfer needs to be reset

    /** Voltage compensation factor */
    var dVoltage: Double = 1.0
    
    /** Current calculated target velocity for flywheel */
    var targetFlywheelVelocity: Double = 0.0

    /** Whether continuous shooting is requested */
    var shootingRequested: Boolean = false

    enum class State {
        IDLE,
        INTAKING,
        MOVING,
        FULL,
        SHOOTING,
        MOVING_SHOOT
    }

    enum class Event {
        START_INTAKING,
        STOP_INTAKING,
        SHOOT,
        BALL_DETECTED
    }

    val fsm = FSM<State, Event>(
        initialState = State.IDLE,
        behaviors = { state -> stateBehavior(state) },
        io = io
    ).apply {
        // Event: start intaking (user input)
        onEvent(Event.START_INTAKING) {
            if (curState == State.IDLE) {
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
                    curState = State.SHOOTING
                }
                else -> {}
            }
        }
        onEvent(Event.BALL_DETECTED) {
            if(curState == State.INTAKING) {
                spindexerState[0] = Balls.Green
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

        State.IDLE
    }

    /** Reset the transfer servo by running it down for a set duration */
    private suspend fun resetTransfer() {
        io.transfer = TRANSFER_DOWN_POWER
        delay(TRANSFER_DOWN_DURATION.inWholeMilliseconds)
        io.transfer = TRANSFER_STOP_POWER
        transferNeedsReset = false
    }

    /** Activate transfer servo by running it up for a set duration to transfer a ball */
    private suspend fun activateTransfer() {
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
                val curRotation = io.spindexerPosition() / ((1 + (46.0 / 11.0)) * 28) * 2 * PI
                val error = kotlin.math.abs(curRotation - spindexerRotation)

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
            delay(10)
        }

        // Ball detected -> transition to MOVING
        State.MOVING
    }

    private fun movingBehavior(): suspend () -> State = suspend {
        // Stop intake while moving
        io.intake = 0.0
        println("Entering Move State")

        // Rotate spindexer to next position
        spindexerRotation += ROTATE_ANGLE

        // Shift balls in spindexer
        val temp = spindexerState[2]
        spindexerState[2] = spindexerState[1]
        spindexerState[1] = spindexerState[0]
        spindexerState[0] = temp

        // Wait for spindexer to reach target position
        val startTime = io.time()
        while (true) {
            val curRotation = io.spindexerPosition() / ((1 + (46.0 / 11.0)) * 28) * 2 * PI
            val error = kotlin.math.abs(curRotation - spindexerRotation)

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
        State.FULL
    }

    private fun shootingBehavior(): suspend () -> State = suspend {
        // Flywheel control is handled in update() based on state

        if (offsetActive == false) {
            spindexerRotation += MODE_CHANGE_ANGLE
            offsetActive = true

            // Wait until spindexer reaches target position
            val startTime = io.time()
            while (true) {
                val curRotation = io.spindexerPosition() / ((1 + (46.0 / 11.0)) * 28) * 2 * PI
                val error = kotlin.math.abs(curRotation - spindexerRotation)

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

        // Wait for flywheel to spin up
        val startTime = io.time()
        while (true) {
            val currentVelocity = io.flywheelVelocity()
            // Target is calculated in update()
            val error = kotlin.math.abs(currentVelocity - (ShotPowers.shotPower * ShooterFlywheelPIDConfig.maxVelocity))

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
        activateTransfer()

        transferNeedsReset = true

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
        awaitAll(
            fsm.scope.async { resetTransfer() },
            fsm.scope.async {
                // Flywheel control is handled in update() based on state
                
                // Rotate to next ball
                spindexerRotation += ROTATE_ANGLE

                // Shift balls
                val temp = spindexerState[2]
                spindexerState[2] = spindexerState[1]
                spindexerState[1] = spindexerState[0]
                spindexerState[0] = temp

                // Wait for spindexer to reach target position
                val startTime = io.time()
                while (true) {
                    val curRotation = io.spindexerPosition() / ((1 + (46.0 / 11.0)) * 28) * 2 * PI
                    val error = kotlin.math.abs(curRotation - spindexerRotation)

                    if (error < POSITION_ERROR_THRESHOLD) {
                        break
                    }
                    if (io.time() - startTime > MAX_WAIT_TIME) {
                        println("Warning: Spindexer movement timeout")
                        break
                    }
                    delay(10)
                }
            }
        )

        // Continue shooting if requested, otherwise go to IDLE
        if (shootingRequested) {
            State.SHOOTING
        } else {
            State.IDLE
        }
    }

    // Public API for sending events
    fun startIntaking() = fsm.sendEvent(Event.START_INTAKING)
    fun stopIntaking() = fsm.sendEvent(Event.STOP_INTAKING)
    fun shoot() = fsm.sendEvent(Event.SHOOT)
    fun rotate() = fsm.sendEvent(Event.BALL_DETECTED)
    
    /** Manually rotate the spindexer by a specific angle (radians) */
    fun nudge(angle: Double) {
        spindexerRotation += angle
    }

    // Current state accessor
    val currentState: State get() = fsm.curState

    fun update(motorTick: Double, deltaT: Duration, dVoltage: Double = 1.0) {
        this.dVoltage = dVoltage
        fsm.update()

        val curRotation = motorTick/((1+(46.0/11.0)) * 28)*2*PI

        val slewLimitedTarget = if (slewRateLimitingEnabled) {
            slewRateLimiter.maxRate = SpindexerPIDConfig.slewRate
            slewRateLimiter.calculate(spindexerRotation, deltaT)
        } else {
            spindexerRotation
        }

        effectiveTargetRotation = slewLimitedTarget

        // Update PID for spindexer motor position
        val count = spindexerState.count { it != null }

        spindexerPID.kp = SpindexerPIDConfig.getKp(count)
        spindexerPID.kd = SpindexerPIDConfig.getKd(count)
        spindexerPID.ki = SpindexerPIDConfig.getKi(count)

        spindexerPID.setpoint = effectiveTargetRotation
        val power = spindexerPID.update(curRotation, deltaT).coerceIn(-1.0, 1.0)
        io.spindexer = power.coerceIn(-0.5,0.5)
        
        // Update Flywheel PID if in shooting state
        if (fsm.curState == State.SHOOTING || fsm.curState == State.MOVING_SHOOT) {
            // Update PID gains
            flywheelPID.kp = ShooterFlywheelPIDConfig.kP
            flywheelPID.kd = ShooterFlywheelPIDConfig.kD
            flywheelPID.ki = ShooterFlywheelPIDConfig.kI
            
            // Calculate target
            val maxVel = ShooterFlywheelPIDConfig.maxVelocity
            targetFlywheelVelocity = ShotPowers.shotPower * maxVel
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
