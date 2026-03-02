package sigmacorns.control

import kotlinx.coroutines.CancellableContinuation
import kotlinx.coroutines.CoroutineDispatcher
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Delay
import kotlinx.coroutines.InternalCoroutinesApi
import kotlinx.coroutines.Job
import kotlinx.coroutines.Runnable
import kotlinx.coroutines.launch
import sigmacorns.io.SigmaIO
import java.util.PriorityQueue
import kotlin.coroutines.CoroutineContext
import kotlin.math.min

// a coroutine dispatcher that can run on the main thread with an explicit update method
@OptIn(InternalCoroutinesApi::class)
class PollableDispatcher(val io: SigmaIO): CoroutineDispatcher(), Delay {
    private val q = ArrayDeque<Runnable>()
    private val timers = PriorityQueue<Pair<Long, Runnable>>(compareBy { it.first })

    override fun dispatch(context: CoroutineContext, block: Runnable) {
        q.add(block)
    }

    override fun scheduleResumeAfterDelay(
        timeMillis: Long,
        continuation: CancellableContinuation<Unit>
    ) {
        timers.add(
            io.time().inWholeMilliseconds + timeMillis to Runnable { continuation.resume(Unit) { cause, _, _ -> } }
        )
    }

    fun update(limit: Int = Int.MAX_VALUE) {
        // move due timers into execution queue
        val now = io.time().inWholeMilliseconds
        while (true) {
            val head = timers.peek() ?: break
            val due = if (head.first <= now) timers.poll()!!.second else break
            q.add(due)
        }

        // run execution queue, not running the ones that are added in the middle of the update
        val limit = min(limit, q.size)
        var ran = 0
        while (ran < limit) {
            val r = q.removeFirstOrNull() ?: break
            r.run()
            ran++
        }
    }
}

class FSM<S,E>(
    initialState: S,
    val behaviors: (S) -> (suspend () -> S),
    io: SigmaIO
) {
    var curState: S = initialState

    private val dispatcher = PollableDispatcher(io)
    val scope = CoroutineScope(dispatcher)

    var job: Job = scope.launch { curState = behaviors(curState)() }

    private val sentEvents = ArrayDeque<E>()
    private val eventHandlers = mutableMapOf<E, suspend () -> Unit>()


    fun onEvent(event: E, handler: suspend () -> Unit) {
        eventHandlers[event] = handler
    }

    fun sendEvent(e: E) {
        sentEvents.add(e)
    }

    private var oldState = curState

    fun update() {
        // process events
        val limit = sentEvents.size
        var i = 0
        while(i < limit) {
            val e = sentEvents.removeFirstOrNull() ?: break
            val handler = eventHandlers[e]
            if (handler != null) {
                scope.launch {
                    val oldState = curState
                    handler()

                    // if state was changed by event, cancel the state behavior from running
                    if(curState != oldState && !job.isCompleted) job.cancel()
                }
            }
            i++
        }

        dispatcher.update(100)

        if(job.isCompleted || oldState != curState) {
            // if state was forcibly changed, cancel the old state behavior
            if(!job.isCompleted) job.cancel()

            // start new state behavior
            job = scope.launch { curState = behaviors(curState)() }
        }

        oldState = curState
    }
}