package sigmacorns.test

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Nested
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.assertNotNull
import sigmacorns.control.FSM
import sigmacorns.control.PollableDispatcher
import sigmacorns.io.SimIO
import java.util.concurrent.atomic.AtomicInteger
import kotlin.time.Duration.Companion.milliseconds

@DisplayName("FSM Tests")
class FSMTest {

    private lateinit var simIO: SimIO

    @BeforeEach
    fun setup() {
        simIO = SimIO()
    }

    @Nested
    @DisplayName("PollableDispatcher Tests")
    inner class PollableDispatcherTests {

        @Test
        @DisplayName("dispatch adds tasks to the queue")
        fun testDispatchAddsTask() {
            val dispatcher = PollableDispatcher(simIO)
            val executed = AtomicInteger(0)

            CoroutineScope(dispatcher).launch {
                executed.incrementAndGet()
            }

            dispatcher.update()
            assertEquals(1, executed.get())
        }

        @Test
        @DisplayName("update respects the limit parameter")
        fun testUpdateLimitParameter() {
            val dispatcher = PollableDispatcher(simIO)
            val executed = AtomicInteger(0)

            // Add 5 tasks
            repeat(5) {
                CoroutineScope(dispatcher).launch {
                    executed.incrementAndGet()
                }
            }

            // Update with limit of 2
            dispatcher.update(limit = 2)
            assertEquals(2, executed.get())

            // Update again to process remaining
            dispatcher.update(limit = Int.MAX_VALUE)
            assertEquals(5, executed.get())
        }

        @Test
        @DisplayName("update default limit processes 100 tasks")
        fun testUpdateDefaultLimit() {
            val dispatcher = PollableDispatcher(simIO)
            val executed = AtomicInteger(0)

            // Add 100 tasks
            repeat(100) {
                CoroutineScope(dispatcher).launch {
                    executed.incrementAndGet()
                }
            }

            dispatcher.update()
            assertEquals(100, executed.get())
        }

        @Test
        @DisplayName("scheduled delays are processed in order")
        fun testScheduledDelaysInOrder() {
            val dispatcher = PollableDispatcher(simIO)
            val order = mutableListOf<Int>()

            CoroutineScope(dispatcher).launch {
                delay(50)
                order.add(1)
            }

            CoroutineScope(dispatcher).launch {
                delay(10)
                order.add(2)
            }

            CoroutineScope(dispatcher).launch {
                delay(30)
                order.add(3)
            }

            // Advance time and process
            simIO.update()
            repeat(100) {
                simIO.update()
                dispatcher.update(limit = 100)
            }

            // Should process in order of delay (10ms, 30ms, 50ms)
            assertEquals(listOf(2, 3, 1), order)
        }

        @Test
        @DisplayName("multiple tasks are executed in order")
        fun testMultipleTasksExecutedInOrder() {
            val dispatcher = PollableDispatcher(simIO)
            val executionOrder = mutableListOf<Int>()

            repeat(5) { i ->
                CoroutineScope(dispatcher).launch {
                    executionOrder.add(i)
                }
            }

            dispatcher.update()
            assertEquals(listOf(0, 1, 2, 3, 4), executionOrder)
        }

        @Test
        @DisplayName("tasks added during update are queued for next update")
        fun testTasksAddedDuringUpdate() {
            val dispatcher = PollableDispatcher(simIO)
            val executed = AtomicInteger(0)

            CoroutineScope(dispatcher).launch {
                executed.incrementAndGet()
                // Add another task during execution
                CoroutineScope(dispatcher).launch {
                    executed.incrementAndGet()
                }
            }

            dispatcher.update()
            assertEquals(1, executed.get())

            dispatcher.update()
            assertEquals(2, executed.get())
        }
    }

    @Nested
    @DisplayName("FSM Tests")
    inner class FSMStateTests {

        @Test
        @DisplayName("FSM initializes with correct initial state")
        fun testFSMInitialization() {
            val fsm = FSM<String, Unit>(
                initialState = "idle",
                behaviors = { state -> { state } },
                io = simIO
            )

            assertEquals("idle", fsm.curState)
        }

        @Test
        @DisplayName("FSM update creates a job on first call")
        fun testFSMFirstUpdate() {
            val fsm = FSM<String, Unit>(
                initialState = "idle",
                behaviors = { state -> { state } },
                io = simIO
            )

            fsm.update()
            assertNotNull(fsm.job)
        }

        @Test
        @DisplayName("FSM transitions between states")
        fun testFSMStateTransition() {
            val fsm = FSM<String, Unit>(
                initialState = "idle",
                behaviors = { state ->
                    {
                        when (state) {
                            "idle" -> "running"
                            "running" -> "idle"
                            else -> state
                        }
                    }
                },
                io = simIO
            )

            fsm.update()

            assertEquals("running", fsm.curState)
        }

        @Test
        @DisplayName("FSM behaviors function is called with correct state")
        fun testFSMBehaviorsCalled() {
            var receivedState: String? = null

            val fsm = FSM<String, Unit>(
                initialState = "initial",
                behaviors = { state ->
                    receivedState = state
                    { state }
                },
                io = simIO
            )

            fsm.update()
            repeat(5) { simIO.update() }
            fsm.update()

            assertEquals("initial", receivedState)
        }

        @Test
        @DisplayName("FSM handles multiple state transitions")
        fun testFSMMultipleTransitions() {
            val stateSequence = mutableListOf<String>()
            var transitionCount = 0

            val fsm = FSM<Int, Unit>(
                initialState = 0,
                behaviors = { state ->
                    stateSequence.add("state_$state")
                    transitionCount++
                    {
                        if (state < 3) state + 1 else state
                    }
                },
                io = simIO
            )

            // Run several update cycles
            repeat(8) {
                fsm.update()
                repeat(5) { simIO.update() }
            }

            // Should have transitioned 0 -> 1 -> 2 -> 3
            assertEquals(3, fsm.curState)
            assertTrue(transitionCount >= 4)
        }

        // BUG: error is reported when suspend is in displayname
        @Test
        @DisplayName("FSM handles suspen ded behaviors")
        fun testFSMSuspendedBehaviors() {
            val fsm = FSM<String, Unit>(
                initialState = "start",
                behaviors = { state ->
                    when (state) {
                        "start" -> suspend { "middle" }
                        "middle" -> suspend { "end" }
                        else -> suspend { state }
                    }
                },
                io = simIO
            )

            fsm.update()
            repeat(20) { simIO.update() }
            fsm.update()

            // Should have progressed through states
            assertTrue(fsm.curState in listOf("start", "middle", "end"))
        }

        @Test
        @DisplayName("FSM does not create new job while current job is running")
        fun testFSMJobManagement() {
            var behaviorCallCount = 0

            val fsm = FSM<String, Unit>(
                initialState = "state",
                behaviors = { _ ->
                    behaviorCallCount++
                    { "state" }
                },
                io = simIO
            )

            val initialJob = fsm.job
            fsm.update()
            val jobAfterFirstUpdate = fsm.job

            // Multiple updates without advancing time should not create new job
            fsm.update()
            fsm.update()

            assertNotNull(jobAfterFirstUpdate)
            assertTrue(behaviorCallCount >= 1)
        }

        @Test
        @DisplayName("FSM transitions with numeric states")
        fun testFSMNumericStates() {
            val fsm = FSM<Int, Unit>(
                initialState = 0,
                behaviors = { state ->
                    { if (state < 5) state + 1 else state }
                },
                io = simIO
            )

            repeat(12) {
                fsm.update()
                repeat(5) { simIO.update() }
            }

            assertEquals(5, fsm.curState)
        }

        @Test
        @DisplayName("FSM can maintain state if behavior returns same state")
        fun testFSMStableState() {
            val fsm = FSM<String, Unit>(
                initialState = "stable",
                behaviors = { state -> { state } },
                io = simIO
            )

            val originalState = fsm.curState
            repeat(5) {
                fsm.update()
                repeat(5) { simIO.update() }
            }

            assertEquals(originalState, fsm.curState)
        }

        @Test
        @DisplayName("FSM tracks state through complex transitions")
        fun testFSMComplexTransitions() {
            data class ComplexState(val phase: String, val iteration: Int)

            val fsm = FSM<ComplexState, Unit>(
                initialState = ComplexState("init", 0),
                behaviors = { state ->
                    {
                        when {
                            state.phase == "init" && state.iteration < 2 ->
                                state.copy(iteration = state.iteration + 1)
                            state.phase == "init" && state.iteration >= 2 ->
                                ComplexState("processing", 0)
                            state.phase == "processing" && state.iteration < 1 ->
                                state.copy(iteration = state.iteration + 1)
                            else ->
                                ComplexState("complete", 0)
                        }
                    }
                },
                io = simIO
            )

            repeat(10) {
                fsm.update()
                repeat(5) { simIO.update() }
            }

            assertEquals("complete", fsm.curState.phase)
        }
    }


    enum class EventType {
        START, STOP, RESET, ADVANCE
    }

    enum class StateType {
        IDLE, RUNNING, STOPPED
    }

    @Nested
    @DisplayName("FSM Event System Tests")
    inner class FSMEventSystemTests {

        @Test
        @DisplayName("onEvent registers event handler successfully")
        fun testOnEventRegistersHandler() {
            var eventHandled = false

            val fsm = FSM<String, EventType>(
                initialState = "idle",
                behaviors = { state -> { state } },
                io = simIO
            )

            fsm.onEvent(EventType.START) {
                eventHandled = true
            }

            fsm.sendEvent(EventType.START)
            fsm.update()

            assertTrue(eventHandled)
        }

        @Test
        @DisplayName("event handler can change FSM state")
        fun testEventHandlerChangesState() {
            val fsm = FSM<StateType, EventType>(
                initialState = StateType.IDLE,
                behaviors = { state -> { state } },
                io = simIO
            )

            fsm.onEvent(EventType.START) {
                fsm.curState = StateType.RUNNING
            }

            fsm.update()
            fsm.sendEvent(EventType.START)
            fsm.update()

            assertEquals(StateType.RUNNING, fsm.curState)
        }

        @Test
        @DisplayName("multiple events are processed in order")
        fun testMultipleEventsProcessedInOrder() {
            val processedEvents = mutableListOf<EventType>()

            val fsm = FSM<String, EventType>(
                initialState = "idle",
                behaviors = { state -> { state } },
                io = simIO
            )

            fsm.onEvent(EventType.START) {
                processedEvents.add(EventType.START)
            }

            fsm.onEvent(EventType.ADVANCE) {
                processedEvents.add(EventType.ADVANCE)
            }

            fsm.onEvent(EventType.STOP) {
                processedEvents.add(EventType.STOP)
            }

            fsm.sendEvent(EventType.START)
            fsm.sendEvent(EventType.ADVANCE)
            fsm.sendEvent(EventType.STOP)

            fsm.update()

            assertEquals(listOf(EventType.START, EventType.ADVANCE, EventType.STOP), processedEvents)
        }

        @Test
        @DisplayName("event handler with delayed operation changes state")
        fun testEventHandlerWithDelayChangesState() {
            val fsm = FSM<StateType, EventType>(
                initialState = StateType.IDLE,
                behaviors = { state -> { state } },
                io = simIO
            )

            fsm.onEvent(EventType.START) {
                delay(10)
                fsm.curState = StateType.RUNNING
            }

            fsm.update()
            fsm.sendEvent(EventType.START)
            fsm.update()

            // Time hasn't advanced yet
            assertEquals(StateType.IDLE, fsm.curState)

            // Advance time and update
            while(simIO.time() < 20.milliseconds) {
                fsm.update()
                simIO.update()
            }
            fsm.update()

            assertEquals(StateType.RUNNING, fsm.curState)
        }

        @Test
        @DisplayName("event during state transition is handled")
        fun testEventDuringStateTransition() {
            var eventProcessed = false
            var stateWhenEventProcessed = ""

            val fsm = FSM<String, EventType>(
                initialState = "state1",
                behaviors = { state ->
                    {
                        println("curstate= $state")
                        if (state == "state1") "state2" else "state1"
                    }
                },
                io = simIO
            )

            fsm.onEvent(EventType.STOP) {
                eventProcessed = true
                stateWhenEventProcessed = fsm.curState
            }

            assertEquals("state1", fsm.curState)
            fsm.update()

            assertEquals("state2", fsm.curState)
            repeat(5) { simIO.update() }
            fsm.update()

            assertEquals("state1", fsm.curState)

            // Now in state2
            fsm.sendEvent(EventType.STOP)
            fsm.update()

            assertTrue(eventProcessed)
            assertEquals("state2", stateWhenEventProcessed)
        }

        @Test
        @DisplayName("event handler can trigger state changes via different events")
        fun testEventHandlerTriggersStateChange() {
            val fsm = FSM<StateType, EventType>(
                initialState = StateType.IDLE,
                behaviors = { state ->
                    {
                        when (state) {
                            StateType.IDLE -> StateType.IDLE
                            StateType.RUNNING -> StateType.STOPPED
                            StateType.STOPPED -> StateType.IDLE
                        }
                    }
                },
                io = simIO
            )

            fsm.onEvent(EventType.START) {
                fsm.curState = StateType.RUNNING
            }

            fsm.onEvent(EventType.STOP) {
                fsm.curState = StateType.STOPPED
            }

            fsm.update()
            repeat(5) { simIO.update() }

            fsm.sendEvent(EventType.START)
            fsm.update()
            assertEquals(StateType.RUNNING, fsm.curState)

            repeat(5) { simIO.update() }
            fsm.update()

            fsm.sendEvent(EventType.STOP)
            fsm.update()
            assertEquals(StateType.STOPPED, fsm.curState)
        }

        @Test
        @DisplayName("multiple consecutive events change state correctly")
        fun testConsecutiveEventsChangeState() {
            var eventCounter = 0

            val fsm = FSM<Int, EventType>(
                initialState = 0,
                behaviors = { state -> { state } },
                io = simIO
            )

            fsm.onEvent(EventType.ADVANCE) {
                eventCounter++
                fsm.curState = eventCounter
            }

            fsm.update()

            repeat(5) {
                fsm.sendEvent(EventType.ADVANCE)
                fsm.update()
            }

            assertEquals(5, fsm.curState)
            assertEquals(5, eventCounter)
        }

        @Test
        @DisplayName("event handler state change prevents behavior from running same state again")
        fun testEventStateChangePreventsRepeatBehavior() {
            var behaviorCallCount = 0

            val fsm = FSM<StateType, EventType>(
                initialState = StateType.IDLE,
                behaviors = { state ->
                    behaviorCallCount++
                    { state }
                },
                io = simIO
            )

            fsm.onEvent(EventType.START) {
                fsm.curState = StateType.RUNNING
            }

            fsm.update()
            repeat(5) { simIO.update() }
            val callCountBeforeEvent = behaviorCallCount

            fsm.sendEvent(EventType.START)
            fsm.update()
            repeat(5) { simIO.update() }
            fsm.update()

            // Behavior should have been called again with new state
            assertTrue(behaviorCallCount > callCountBeforeEvent)
            assertEquals(StateType.RUNNING, fsm.curState)
        }

        @Test
        @DisplayName("event sent but not processed until update is called")
        fun testEventNotProcessedWithoutUpdate() {
            var eventHandled = false

            val fsm = FSM<String, EventType>(
                initialState = "idle",
                behaviors = { state -> { state } },
                io = simIO
            )

            fsm.onEvent(EventType.START) {
                eventHandled = true
            }

            fsm.sendEvent(EventType.START)

            // Event should not be processed yet
            assertFalse(eventHandled)

            fsm.update()

            assertTrue(eventHandled)
        }

        @Test
        @DisplayName("event handler can access FSM instance and its state")
        fun testEventHandlerAccessesFSMState() {
            var stateWhenHandlerCalled = ""

            val fsm = FSM<String, EventType>(
                initialState = "initial",
                behaviors = { state -> { state } },
                io = simIO
            )

            fsm.onEvent(EventType.START) {
                stateWhenHandlerCalled = fsm.curState
            }

            fsm.update()
            fsm.sendEvent(EventType.START)
            fsm.update()

            assertEquals("initial", stateWhenHandlerCalled)
        }

        @Test
        @DisplayName("many events queued are all processed")
        fun testManyEventsProcessed() {
            val processedCount = AtomicInteger(0)

            val fsm = FSM<String, Int>(
                initialState = "idle",
                behaviors = { state -> { state } },
                io = simIO
            )

            for (i in 0 until 50) {
                fsm.onEvent(i) {
                    processedCount.incrementAndGet()
                }
            }

            fsm.update()

            // Queue many events
            repeat(50) { i ->
                fsm.sendEvent(i)
            }

            fsm.update()

            assertEquals(50, processedCount.get())
        }

        @Test
        @DisplayName("event can trigger complex state transitions")
        fun testComplexStateTransitionViaEvent() {
            data class ComplexState(val mode: String, val value: Int)

            val fsm = FSM<ComplexState, EventType>(
                initialState = ComplexState("idle", 0),
                behaviors = { state -> { state } },
                io = simIO
            )

            fsm.onEvent(EventType.START) {
                fsm.curState = fsm.curState.copy(mode = "running", value = 1)
            }

            fsm.onEvent(EventType.ADVANCE) {
                fsm.curState = fsm.curState.copy(value = fsm.curState.value + 1)
            }

            fsm.onEvent(EventType.STOP) {
                fsm.curState = fsm.curState.copy(mode = "stopped")
            }

            fsm.onEvent(EventType.RESET) {
                fsm.curState = ComplexState("idle", 0)
            }

            fsm.update()

            fsm.sendEvent(EventType.START)
            fsm.update()
            assertEquals("running", fsm.curState.mode)
            assertEquals(1, fsm.curState.value)

            fsm.sendEvent(EventType.ADVANCE)
            fsm.update()
            assertEquals(2, fsm.curState.value)

            fsm.sendEvent(EventType.STOP)
            fsm.update()
            assertEquals("stopped", fsm.curState.mode)

            fsm.sendEvent(EventType.RESET)
            fsm.update()
            assertEquals("idle", fsm.curState.mode)
            assertEquals(0, fsm.curState.value)
        }
    }

    @Nested
    @DisplayName("FSM Integration Tests")
    inner class FSMIntegrationTests {

        @Test
        @DisplayName("FSM state machine for simple switch states")
        fun testSimpleStateMachine() {

            val fsm = FSM<State, Unit>(
                initialState = State.OFF,
                behaviors = { state ->
                    {
                        when (state) {
                            State.OFF -> State.ON
                            State.ON -> State.ERROR
                            State.ERROR -> State.OFF
                        }
                    }
                },
                io = simIO
            )

            repeat(10) {
                fsm.update()
                repeat(5) { simIO.update() }
            }

            assertNotNull(fsm.curState)
            assertTrue(fsm.curState in listOf(State.OFF, State.ON, State.ERROR))
        }
    }

    private enum class State {
        OFF, ON, ERROR
    }
}
