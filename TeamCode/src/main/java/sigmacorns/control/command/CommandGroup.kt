package sigmacorns.control.command

class SequentialCommandGroup(internal val commands: List<Command>) : Command {
    constructor(vararg commands: Command) : this(commands.toList())

    private var index = 0
    override val requirements: Set<Subsystem> = commands.flatMapTo(mutableSetOf()) { it.requirements }

    override fun initialize() {
        index = 0
        if (commands.isNotEmpty()) commands[index].initialize()
    }

    override fun execute() {
        if (index >= commands.size) return
        val current = commands[index]
        current.execute()
        if (current.isFinished()) {
            current.end(false)
            index++
            if (index < commands.size) commands[index].initialize()
        }
    }

    override fun end(interrupted: Boolean) {
        if (interrupted && index < commands.size) {
            commands[index].end(true)
        }
    }

    override fun isFinished(): Boolean = index >= commands.size
}

class ParallelCommandGroup(internal val commands: List<Command>) : Command {
    constructor(vararg commands: Command) : this(commands.toList())

    private val finished = mutableSetOf<Command>()
    override val requirements: Set<Subsystem> = commands.flatMapTo(mutableSetOf()) { it.requirements }

    init {
        val seen = mutableSetOf<Subsystem>()
        for (cmd in commands) {
            for (req in cmd.requirements) {
                require(seen.add(req)) {
                    "ParallelCommandGroup: multiple commands require the same subsystem"
                }
            }
        }
    }

    override fun initialize() {
        finished.clear()
        commands.forEach { it.initialize() }
    }

    override fun execute() {
        for (cmd in commands) {
            if (cmd in finished) continue
            cmd.execute()
            if (cmd.isFinished()) {
                cmd.end(false)
                finished.add(cmd)
            }
        }
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            for (cmd in commands) {
                if (cmd !in finished) cmd.end(true)
            }
        }
    }

    override fun isFinished(): Boolean = finished.size == commands.size
}

class ParallelRaceGroup(internal val commands: List<Command>) : Command {
    constructor(vararg commands: Command) : this(commands.toList())

    private var done = false
    override val requirements: Set<Subsystem> = commands.flatMapTo(mutableSetOf()) { it.requirements }

    init {
        val seen = mutableSetOf<Subsystem>()
        for (cmd in commands) {
            for (req in cmd.requirements) {
                require(seen.add(req)) {
                    "ParallelRaceGroup: multiple commands require the same subsystem"
                }
            }
        }
    }

    override fun initialize() {
        done = false
        commands.forEach { it.initialize() }
    }

    override fun execute() {
        for (cmd in commands) {
            cmd.execute()
            if (cmd.isFinished()) done = true
        }
    }

    override fun end(interrupted: Boolean) {
        for (cmd in commands) cmd.end(interrupted || !cmd.isFinished())
    }

    override fun isFinished(): Boolean = done
}

class ParallelDeadlineGroup(
    private val deadline: Command,
    private val others: List<Command>
) : Command {
    constructor(deadline: Command, vararg others: Command) : this(deadline, others.toList())

    internal val all = listOf(deadline) + others
    override val requirements: Set<Subsystem> = all.flatMapTo(mutableSetOf()) { it.requirements }

    init {
        val seen = mutableSetOf<Subsystem>()
        for (cmd in all) {
            for (req in cmd.requirements) {
                require(seen.add(req)) {
                    "ParallelDeadlineGroup: multiple commands require the same subsystem"
                }
            }
        }
    }

    override fun initialize() {
        all.forEach { it.initialize() }
    }

    override fun execute() {
        for (cmd in all) cmd.execute()
    }

    override fun end(interrupted: Boolean) {
        for (cmd in all) cmd.end(interrupted || !cmd.isFinished())
    }

    override fun isFinished(): Boolean = deadline.isFinished()
}
