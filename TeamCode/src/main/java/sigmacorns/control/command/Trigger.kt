package sigmacorns.control.command

class Trigger(private val condition: () -> Boolean) {
    private var lastState = false
    private val actions = mutableListOf<(pressed: Boolean, lastPressed: Boolean) -> Unit>()

    init {
        CommandScheduler.registerTrigger(this)
    }

    internal fun poll() {
        val current = condition()
        for (action in actions) action(current, lastState)
        lastState = current
    }

    fun whenPressed(command: Command): Trigger = also {
        actions.add { pressed, last ->
            if (pressed && !last) CommandScheduler.schedule(command)
        }
    }

    fun whenReleased(command: Command): Trigger = also {
        actions.add { pressed, last ->
            if (!pressed && last) CommandScheduler.schedule(command)
        }
    }

    fun whilePressed(command: Command): Trigger = also {
        actions.add { pressed, last ->
            if (pressed && !last) CommandScheduler.schedule(command)
            else if (!pressed && last) CommandScheduler.cancel(command)
        }
    }

    fun whileReleased(command: Command): Trigger = also {
        actions.add { pressed, last ->
            if (!pressed && !last) { /* keep running */ }
            else if (!pressed && last) CommandScheduler.schedule(command)
            else if (pressed && !last) CommandScheduler.cancel(command)
        }
    }

    fun toggleWhenPressed(command: Command): Trigger = also {
        actions.add { pressed, last ->
            if (pressed && !last) {
                if (CommandScheduler.isScheduled(command)) CommandScheduler.cancel(command)
                else CommandScheduler.schedule(command)
            }
        }
    }

    fun and(other: Trigger): Trigger = Trigger { condition() && other.condition() }
    fun or(other: Trigger): Trigger = Trigger { condition() || other.condition() }
    fun negate(): Trigger = Trigger { !condition() }
    operator fun not(): Trigger = negate()
}
