package sigmacorns.control

import kotlinx.coroutines.delay
import sigmacorns.sim.Balls
import sigmacorns.io.SigmaIO
import kotlin.time.Duration.Companion.milliseconds

class SpindexerLogic(val io: SigmaIO) {
    // Spindexer state tracking
    var spindexerRotation: Double = 0.0
    var spindexerState: MutableList<Balls?> = mutableListOf(null, null, null)

    // Constants
    private val PI = kotlin.math.PI
    private val ROTATE_ANGLE = (2 * PI) / 3  // rotation for cycling to next ball
    private val MODE_CHANGE_ANGLE = PI / 3   // rotation for switching intake to shoot mode

    // PID controller for spindexer motor
    val spindexerPID = PIDController(1.0, 0.0, 0.0, 0.0)

    // Timings
    private val SPINUP_TIME = 400.milliseconds
    private val MOVE_TIME = 300.milliseconds  // time to rotate spindexer

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
        SHOOT
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
                    io.shooter = 0.8
                    curState = State.SHOOTING
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
    }

    private fun idleBehavior(): suspend () -> State = suspend {
        // Idle: wait for events, no automatic transitions
        io.intake = 0.0
        io.shooter = 0.0
        io.transfer = 0.0
        State.IDLE
    }

    private fun intakingBehavior(): suspend () -> State = suspend {
        // Run intake motor
        io.intake = 0.7

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

        // Rotate spindexer to next position
        spindexerRotation += ROTATE_ANGLE

        // Shift balls in spindexer
        val temp = spindexerState[2]
        spindexerState[2] = spindexerState[1]
        spindexerState[1] = spindexerState[0]
        spindexerState[0] = temp

        // Wait for movement to complete
        delay(MOVE_TIME.inWholeMilliseconds)

        // Check if spindexer is full
        if (spindexerState.all { it != null }) {
            State.FULL
        } else {
            State.INTAKING
        }
    }

    private fun fullBehavior(): suspend () -> State = suspend {
        // Full: wait for shoot event, no automatic transitions
        io.intake = 0.0
        State.FULL
    }

    private fun shootingBehavior(): suspend () -> State = suspend {
        // Start flywheel
        io.shooter = 0.8

        // Wait for spinup
        delay(SPINUP_TIME.inWholeMilliseconds)

        // Activate transfer to shoot ball
        io.transfer = 0.7

        // Wait for ball to be shot
        delay(200)

        // Mark current ball as shot
        spindexerState[0] = null
        io.transfer = 0.0

        // Check if spindexer is empty
        if (spindexerState.all { it == null }) {
            io.shooter = 0.0
            State.IDLE
        } else {
            State.MOVING_SHOOT
        }
    }

    private fun movingShootBehavior(): suspend () -> State = suspend {
        // Rotate to next ball while keeping flywheel spinning
        io.shooter = 0.8
        spindexerRotation += ROTATE_ANGLE

        // Shift balls
        val temp = spindexerState[2]
        spindexerState[2] = spindexerState[1]
        spindexerState[1] = spindexerState[0]
        spindexerState[0] = temp

        // Wait for movement
        delay(MOVE_TIME.inWholeMilliseconds)

        // Continue shooting
        State.SHOOTING
    }

    // Public API for sending events
    fun startIntaking() = fsm.sendEvent(Event.START_INTAKING)
    fun stopIntaking() = fsm.sendEvent(Event.STOP_INTAKING)
    fun shoot() = fsm.sendEvent(Event.SHOOT)

    // Current state accessor
    val currentState: State get() = fsm.curState

    fun update(deltaT: kotlin.time.Duration) {
        fsm.update()

        // Update PID for spindexer motor position
        io.spindexer = spindexerPID.update(spindexerRotation, deltaT)
    }
}
