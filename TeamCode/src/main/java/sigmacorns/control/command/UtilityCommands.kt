package sigmacorns.control.command

import kotlin.time.Duration

class InstantCommand(
    private val action: () -> Unit = {},
    override val requirements: Set<Subsystem> = emptySet()
) : Command {
    override fun initialize() = action()
    override fun isFinished(): Boolean = true
}

class RunCommand(
    private val action: () -> Unit,
    override val requirements: Set<Subsystem> = emptySet()
) : Command {
    override fun execute() = action()
}

class WaitCommand(private val duration: Duration) : Command {
    private var startTime = Duration.ZERO
    private var timeSource: () -> Duration = { Duration.ZERO }

    internal fun bind(scheduler: CommandScheduler) {
        timeSource = scheduler.timeSource
    }

    override fun initialize() {
        startTime = timeSource()
    }

    override fun isFinished(): Boolean = timeSource() - startTime >= duration
}

class WaitUntilCommand(private val condition: () -> Boolean) : Command {
    override fun isFinished(): Boolean = condition()
}

class ConditionalCommand(
    internal val onTrue: Command,
    internal val onFalse: Command,
    private val condition: () -> Boolean
) : Command {
    private lateinit var selected: Command
    override val requirements: Set<Subsystem> =
        onTrue.requirements + onFalse.requirements

    override fun initialize() {
        selected = if (condition()) onTrue else onFalse
        selected.initialize()
    }

    override fun execute() = selected.execute()
    override fun end(interrupted: Boolean) = selected.end(interrupted)
    override fun isFinished(): Boolean = selected.isFinished()
}

class FunctionalCommand(
    private val onInit: () -> Unit = {},
    private val onExecute: () -> Unit = {},
    private val onEnd: (Boolean) -> Unit = {},
    private val isFinishedSupplier: () -> Boolean = { false },
    override val requirements: Set<Subsystem> = emptySet()
) : Command {
    override fun initialize() = onInit()
    override fun execute() = onExecute()
    override fun end(interrupted: Boolean) = onEnd(interrupted)
    override fun isFinished(): Boolean = isFinishedSupplier()
}
