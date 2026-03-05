package sigmacorns.subsystem

import sigmacorns.control.command.Command
import sigmacorns.control.command.Subsystem

class Intake: Subsystem {

    init {}

    override fun periodic() {

    }

}

class RunIntake(subsystem: Intake): Command {

    override val requirements = setOf(subsystem)

    override fun initialize() {

    }

    override fun execute() {

    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        return false;
    }

}