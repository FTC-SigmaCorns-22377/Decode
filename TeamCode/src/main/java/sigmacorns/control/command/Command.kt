package sigmacorns.control.command

import kotlin.time.Duration

interface Command {
    fun initialize() {}
    fun execute() {}
    fun end(interrupted: Boolean) {}
    fun isFinished(): Boolean = false
    val requirements: Set<Subsystem>
}

fun Command.andThen(next: Command): Command =
    SequentialCommandGroup(this, next)

fun Command.alongWith(vararg others: Command): Command =
    ParallelCommandGroup(this, *others)

fun Command.raceWith(vararg others: Command): Command =
    ParallelRaceGroup(this, *others)

fun Command.deadlineWith(vararg others: Command): Command =
    ParallelDeadlineGroup(this, *others)

fun Command.withTimeout(duration: Duration): Command =
    raceWith(WaitCommand(duration))

fun Command.withInterrupt(condition: () -> Boolean): Command =
    raceWith(WaitUntilCommand(condition))
