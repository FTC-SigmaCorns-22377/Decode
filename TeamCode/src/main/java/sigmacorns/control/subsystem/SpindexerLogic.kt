package sigmacorns.control.subsystem

import com.bylazar.configurables.annotations.Configurable
import kotlinx.coroutines.delay
import kotlinx.coroutines.yield
import sigmacorns.constants.flywheelMotor
import sigmacorns.control.FSM
import sigmacorns.control.MotorRangeMapper
import sigmacorns.control.PIDController
import sigmacorns.sim.Balls
import sigmacorns.io.SigmaIO
import sigmacorns.opmode.tune.ShooterFlywheelPIDConfig
import kotlin.math.abs
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

class SpindexerLogic(val io: SigmaIO, var flywheel: Flywheel? = null) {
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

    val spindexer = Spindexer(
        MotorRangeMapper(
            0.0..2.0 * PI,
            0.0..(((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0,
            Double.POSITIVE_INFINITY
        ),io)

    // Error thresholds
    internal val POSITION_ERROR_THRESHOLD = 0.10  // radians - position threshold for spindexer
    internal val VELOCITY_ERROR_THRESHOLD = 10.0   // rad/s - velocity threshold for flywheel

    // Timeout for safety (fallback if threshold never reached)
    internal val MAX_WAIT_TIME = 10000.milliseconds

    // Transfer servo position constants db(discrete servo: 0.0 = retracted, 1.0 = extended)
    private val TRANSFER_UP_POSITION = 0.135       // Extended position (towards shooter)
    private val TRANSFER_DOWN_POSITION = 0.0     // Retracted position (reset)
    private val TRANSFER_UP_TIME = 450.milliseconds
    private val TRANSFER_RESET_TIME = 200.milliseconds

    //extra variables
    internal var offsetActive: Boolean = false
    private var transferNeedsReset: Boolean = true  // Track if transfer needs to be reset
    private var nudgeDirection: Double = 1.0  // Track the direction of the last nudge (1.0 for forward/CW, -1.0 for backward/CCW)
    private var sortCycle: Int = 0 //Tracks the number of iterations the sorting has been through

    var shotPower: Double = 0.0
    var spinupPower: Double = 0.6 * flywheelMotor.freeSpeed
    var shotVelocity: Double? = null
    var preSpinActive: Boolean = false

    private var flywheelTargetVelocity: Double = 0.0
    var motif: List<Balls?> = listOf(Balls.Green, Balls.Purple, Balls.Purple)

    /** Whether continuous shooting is requested */
    var shootingRequested: Boolean = false
    var spinupRequested: Boolean = false

    var autoSort: Boolean = true

    // Brake tilt servos
    var requestLift: Boolean = false
    private val TILT_ACTIVE_POSITION = 0.5

    public enum class State {
        IDLE,
        INTAKING,
        MOVING,
        FULL,
        SHOOTING,
        MOVING_SHOOT,
        SORTED_SHOOTING,
        ZERO
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
            shootingRequested = true
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
        State.IDLE -> this::idleBehavior
        State.INTAKING -> this::intakingBehavior
        State.MOVING -> this::movingBehavior
        State.FULL -> this::fullBehavior
        State.SHOOTING -> this::shootingBehavior
        State.MOVING_SHOOT -> this::movingShootBehavior
        State.SORTED_SHOOTING -> this::sortedShootingBehavior
        State.ZERO -> this::zeroBehavior
    }

    private suspend fun zeroBehavior(): State {
        if(offsetActive) spindexerRotation -= MODE_CHANGE_ANGLE


        delay(100)

        return State.ZERO
    }

    private suspend fun idleBehavior(): State {
        // Idle: wait for events, no automatic transitions
        io.intake = 0.0
        io.transfer = TRANSFER_DOWN_POSITION

        if(!offsetActive) {
            pollColor()
        }

        // Reset transfer on first idle if needed
        if (transferNeedsReset) {
            resetTransfer()
        }

        // Poll so spinup changes are picked up
        while (true) {
            if (spinupRequested) {
                flywheelTargetVelocity = spinupPower
                flywheel?.hold = true
            } else {
                flywheelTargetVelocity = 0.0
                flywheel?.hold = true
            }
            delay(10)
        }
        return State.IDLE
    }

    /** Reset the transfer servo to the retracted position */
    internal suspend fun resetTransfer() {
        io.transfer = TRANSFER_DOWN_POSITION
        if(transferNeedsReset) delay(TRANSFER_RESET_TIME)
        transferNeedsReset = false
    }

    /** Extend transfer servo to shoot a ball */
    internal suspend fun activateTransfer() {
        flywheel?.hold = true
        io.transfer = TRANSFER_UP_POSITION
        delay(TRANSFER_UP_TIME)
        transferNeedsReset = true  // Mark that we need to reset next time
    }

    private fun pollColor(): Boolean {
        val ballDetected = io.colorSensorDetectsBall()
        if (ballDetected) {
            spindexerState[0] = io.colorSensorGetBallColor() ?: Balls.Green
        }
        return ballDetected
    }

    private suspend fun intakingBehavior(): State {
        // Run intake motor
        if (offsetActive) {
            spindexerRotation += MODE_CHANGE_ANGLE
            offsetActive = false

            // Wait until spindexer reaches target position
            val startTime = io.time()
            while (true) {
                val error = abs(spindexer.curRotation - spindexerRotation)

                if (error < POSITION_ERROR_THRESHOLD) {
                    break
                }
                if (io.time() - startTime > MAX_WAIT_TIME) {
                    println("Warning: Spindexer offset timeout")
                    break
                }
                yield()
            }
        }

        //if flywheel spinup is requested, spin up flywheel
        if (spinupRequested == true) {
            flywheelTargetVelocity = spinupPower
            flywheel?.hold = true
        } else {
            flywheelTargetVelocity = 0.0
            flywheel?.hold = true
        }

        io.intake = -1.0

        // Check for ball detection (polling)
        while (!pollColor()) yield()

        // Ball detected -> transition to MOVING
        nudgeDirection = 1.0
        return State.MOVING
    }

    private suspend fun movingBehavior(): State {
        // slow intake while moving
        io.intake = if (fsm.curState == State.MOVING) -1.0 else 0.0

        // Rotate spindexer to next position based on nudge direction
        val rotationAngle = nudgeDirection * ROTATE_ANGLE
        spindexerRotation += rotationAngle

        // Shift balls in spindexer based on rotation direction
        if (nudgeDirection > 0) {
            val temp = spindexerState[2]
            spindexerState[2] = spindexerState[1]
            spindexerState[1] = spindexerState[0]
            spindexerState[0] = temp
        } else if(nudgeDirection < 0) {
            val temp = spindexerState[0]
            spindexerState[0] = spindexerState[1]
            spindexerState[1] = spindexerState[2]
            spindexerState[2] = temp
        }

        // Wait for spindexer to reach target position
        val startTime = io.time()
        while (true) {
            val error = abs(spindexer.curRotation - spindexerRotation)

            //when close enough start intaking
            if (error < Math.toRadians(18.0) && fsm.curState == State.MOVING) {
                io.intake = -1.0
            }

            if (error < POSITION_ERROR_THRESHOLD) {
                break
            }
            if (io.time() - startTime > MAX_WAIT_TIME) {
                println("Warning: Spindexer movement timeout")
                break
            }
            yield()
        }

        // Check if spindexer is full
        return if (spindexerState.all { it != null }) State.FULL else State.INTAKING
    }

    private suspend fun fullBehavior(): State {
        // Full: wait for shoot event, no automatic transitions
        io.intake = 0.0

        //if flywheel spinup is requested, spin up flywheel
        if (spinupRequested == true) {
            flywheelTargetVelocity = spinupPower
            flywheel?.hold = true
        } else {
            flywheelTargetVelocity = 0.0
            flywheel?.hold = true
        }

        delay(Long.MAX_VALUE)
        return State.FULL
    }
    var foundAnyBall = false
    var requiredColor = motif[0]
    private suspend fun sortBehavior() {
        requiredColor = motif[sortCycle % 3]

        foundAnyBall = spindexerState.contains(requiredColor)

        val nudgePairs = mapOf(1 to 0.0, 0 to 1.0, 2 to -1.0)
        nudgeDirection = nudgePairs.firstNotNullOfOrNull { p ->
            p.value.takeIf { spindexerState[p.key]==requiredColor }
        } ?: nudgePairs.firstNotNullOfOrNull { p ->
            p.value.takeIf {spindexerState[p.key] != null}
        } ?: 1.0

        println("SPINDEXER: sort found=$foundAnyBall required=$requiredColor nudgeDirection=$nudgeDirection")
        movingBehavior()
    }

    private suspend fun shootingBehavior(): State {
        // Wait for flywheel to spin up
        spinupRequested = false
        val startTime = io.time()
        while (true) {
            flywheelTargetVelocity = shotVelocity ?: (shotPower* flywheelMotor.freeSpeed)
            val currentVelocity = io.flywheelVelocity()
            val error = abs(currentVelocity - flywheelTargetVelocity)

            if (error < VELOCITY_ERROR_THRESHOLD) break
            if (io.time() - startTime > MAX_WAIT_TIME) {
                println("Warning: Flywheel spinup timeout")
                break
            }
            yield()
        }

        // Extend transfer servo to shoot ball
        transferNeedsReset = true
        activateTransfer()

        // Mark current ball as shot
        spindexerState[1] = null
        sortCycle += 1 //changed this from =+, if there are issues, go back and change it

        // Check if spindexer is empty
        return if (spindexerState.all { it == null } && !shootingRequested) {
            io.shooter = 0.0
            State.IDLE
        } else {
            State.MOVING_SHOOT
        }
    }

    private suspend fun movingShootBehavior(): State {
        if (shootingRequested) {
            flywheelTargetVelocity = shotVelocity ?: (shotPower* flywheelMotor.freeSpeed)
            flywheel?.hold = false
        }

        resetTransfer()
        if (!offsetActive) {
            spindexerRotation += MODE_CHANGE_ANGLE
            offsetActive = true
        }

        // Continue shooting if requested, otherwise go to IDLE
        return if (shootingRequested) {
            shootingRequested = false
            if(autoSort) sortBehavior() else {
                nudgeDirection = 1.0
                movingBehavior()
            }
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
    fun sortedShoot() = fsm.sendEvent(Event.SORTED_SHOOT)

    fun nudge(ccw: Boolean) {
        if (currentState == State.INTAKING || currentState == State.IDLE || currentState == State.FULL) {
            nudgeDirection = if(ccw) 1.0 else -1.0

            // Transition to MOVING state to execute the nudge with directionality
            fsm.curState = State.MOVING
        }
    }

    // Current state accessor
    val currentState: State get() = fsm.curState

    private suspend fun sortedShootingBehavior(): State {
        sortedShoot()
        return State.IDLE
    }

    /**
     * Public API to start sorted shooting.
     */
    fun startSortedShoot(motif: List<Balls?>) {
        this.motif = motif
        fsm.sendEvent(Event.SORTED_SHOOT)
    }

    fun update(dt: Duration) {
        fsm.update()

        // Update spindexer control
        spindexer.target = spindexerRotation
        spindexer.update(dt)

        // Brake tilt servos
        if (requestLift) {
            io.tilt1 = TILT_ACTIVE_POSITION
            io.tilt2 = TILT_ACTIVE_POSITION
        }

        flywheel?.also {
            if (preSpinActive) {
                it.target = shotVelocity ?: (shotPower * flywheelMotor.freeSpeed)
                it.hold = false
            } else {
                it.target = flywheelTargetVelocity
            }
            it.update(io.flywheelVelocity(),dt)
        } ?: run {
            // Use PID + feedforward
            flywheelPID.kp = ShooterFlywheelPIDConfig.kP
            flywheelPID.kd = ShooterFlywheelPIDConfig.kD
            flywheelPID.ki = ShooterFlywheelPIDConfig.kI

            val maxVel = ShooterFlywheelPIDConfig.maxVelocity
            flywheelPID.setpoint = flywheelTargetVelocity

            val currentVel = io.flywheelVelocity()
            val pidOut = flywheelPID.update(currentVel, dt)
            val feedforward = flywheelTargetVelocity / maxVel

            val totalPower = (feedforward + pidOut).coerceIn(-1.0, 1.0)
            io.shooter = totalPower * 12.0/io.voltage()
        }
    }
}
