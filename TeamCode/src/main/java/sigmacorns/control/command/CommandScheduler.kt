package sigmacorns.control.command

import kotlin.time.Duration

object CommandScheduler {
    internal var timeSource: () -> Duration = { Duration.ZERO }
        private set

    private val subsystems = mutableSetOf<Subsystem>()
    private val activeCommands = mutableSetOf<Command>()
    private val subsystemMap = mutableMapOf<Subsystem, Command>()
    private val defaultCommands = mutableMapOf<Subsystem, Command>()
    private val triggers = mutableListOf<Trigger>()

    private val onCommandInit = mutableListOf<(Command) -> Unit>()
    private val onCommandExecute = mutableListOf<(Command) -> Unit>()
    private val onCommandFinish = mutableListOf<(Command) -> Unit>()
    private val onCommandInterrupt = mutableListOf<(Command) -> Unit>()

    fun init(timeSource: () -> Duration) {
        this.timeSource = timeSource
    }

    fun register(vararg subs: Subsystem) {
        subsystems.addAll(subs)
    }

    fun setDefaultCommand(subsystem: Subsystem, command: Command) {
        require(subsystem in command.requirements) {
            "Default command must require its subsystem"
        }
        defaultCommands[subsystem] = command
    }

    fun schedule(command: Command) {
        if (command in activeCommands) return

        for (subsystem in command.requirements) {
            val occupant = subsystemMap[subsystem]
            if (occupant != null) cancelInternal(occupant, interrupted = true)
        }

        bindWaitCommands(command)
        activeCommands.add(command)
        for (subsystem in command.requirements) {
            subsystemMap[subsystem] = command
        }
        command.initialize()
        onCommandInit.forEach { it(command) }
    }

    fun cancel(command: Command) {
        if (command in activeCommands) cancelInternal(command, interrupted = true)
    }

    fun cancelAll() {
        for (cmd in activeCommands.toList()) cancelInternal(cmd, interrupted = true)
    }

    fun isScheduled(command: Command): Boolean = command in activeCommands

    fun requiring(subsystem: Subsystem): Command? = subsystemMap[subsystem]

    fun run() {
        for (subsystem in subsystems) subsystem.periodic()

        for (trigger in triggers) trigger.poll()

        for (cmd in activeCommands.toList()) {
            cmd.execute()
            onCommandExecute.forEach { it(cmd) }
            if (cmd.isFinished()) {
                cmd.end(false)
                activeCommands.remove(cmd)
                for (subsystem in cmd.requirements) {
                    if (subsystemMap[subsystem] == cmd) subsystemMap.remove(subsystem)
                }
                onCommandFinish.forEach { it(cmd) }
            }
        }

        for (subsystem in subsystems) {
            if (subsystem !in subsystemMap) {
                val default = defaultCommands[subsystem]
                if (default != null && default !in activeCommands) {
                    schedule(default)
                }
            }
        }
    }

    fun onCommandInit(listener: (Command) -> Unit) { onCommandInit.add(listener) }
    fun onCommandExecute(listener: (Command) -> Unit) { onCommandExecute.add(listener) }
    fun onCommandFinish(listener: (Command) -> Unit) { onCommandFinish.add(listener) }
    fun onCommandInterrupt(listener: (Command) -> Unit) { onCommandInterrupt.add(listener) }

    internal fun registerTrigger(trigger: Trigger) {
        triggers.add(trigger)
    }

    fun reset() {
        cancelAll()
        subsystems.clear()
        activeCommands.clear()
        subsystemMap.clear()
        defaultCommands.clear()
        triggers.clear()
        onCommandInit.clear()
        onCommandExecute.clear()
        onCommandFinish.clear()
        onCommandInterrupt.clear()
        timeSource = { Duration.ZERO }
    }

    private fun cancelInternal(command: Command, interrupted: Boolean) {
        command.end(interrupted)
        activeCommands.remove(command)
        for (subsystem in command.requirements) {
            if (subsystemMap[subsystem] == command) subsystemMap.remove(subsystem)
        }
        if (interrupted) onCommandInterrupt.forEach { it(command) }
        else onCommandFinish.forEach { it(command) }
    }

    private fun bindWaitCommands(command: Command) {
        if (command is WaitCommand) {
            command.bind(this)
            return
        }
        when (command) {
            is SequentialCommandGroup -> bindGroupChildren(command)
            is ParallelCommandGroup -> bindGroupChildren(command)
            is ParallelRaceGroup -> bindGroupChildren(command)
            is ParallelDeadlineGroup -> bindGroupChildren(command)
            is ConditionalCommand -> bindConditionalChildren(command)
        }
    }

    private fun bindGroupChildren(group: Any) {
        val commands: List<Command> = when (group) {
            is SequentialCommandGroup -> group.commands
            is ParallelCommandGroup -> group.commands
            is ParallelRaceGroup -> group.commands
            is ParallelDeadlineGroup -> group.all
            else -> return
        }
        for (cmd in commands) bindWaitCommands(cmd)
    }

    private fun bindConditionalChildren(cmd: ConditionalCommand) {
        bindWaitCommands(cmd.onTrue)
        bindWaitCommands(cmd.onFalse)
    }
}
