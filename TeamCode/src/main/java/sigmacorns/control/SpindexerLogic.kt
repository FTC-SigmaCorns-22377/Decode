package sigmacorns.control

//imports
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import sigmacorns.sim.Balls
import sigmacorns.io.SigmaIO
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

//need to import gamepad to have accesss to controls
//import PID and instaniate it
class SpindexerLogic(val io: SigmaIO) {
    //Essential Variables
    var spindexerRotation: Double = 0.0 //dummy for right now, once we have the motor rotation in radians from the class that thomas is making
    var spindexerState: List<Balls?> = listOf(
        null,
        null,
        null
    ) //nothing in here for now, append to this as we intake, and shift the positions of the balls as you pick up more balls
    var currentBall: Balls? = spindexerState[0]
    var robotState = States.IDLE


    //Essential Values
    val pi: Double = kotlin.math.PI
    val Rotate: Double = ((2 * pi) / 3) //value for rotation needed for intaking/shooting each ball
    val changeMode: Double = (pi / 3) //value of rotation needed `for switching from intaking to shooting
    val spindexerPID = PIDController(1.0,0.0,0.0,0.0)

    //events
    var intakeRequested: Boolean = false
    var intakeStopRequested: Boolean = false
    var shootRequsted: Boolean = false

    enum class States {
        IDLE,
        INTAKING,
        MOVING,
        FULL,
        MOVING_SHOOTING,
        SHOOTING
    }


    private fun idle() {
        if (intakeRequested) {
            robotState = States.INTAKING
        }
        if (shootRequsted) {
            robotState = States.SHOOTING
        }
    }

    private fun intaking() {
        if (spindexerState[0] != null) {
            robotState = States.MOVING
        }
        if (intakeStopRequested) {
            robotState = States.IDLE
        }
    }

    private fun startmoving() {
        spindexerState[1] == spindexerState[0] //shifting, also why does this work with == but not =???
        spindexerState[2] == spindexerState[1]
        spindexerState[0] == spindexerState[2]
        currentBall = spindexerState[0]
        spindexerRotation += Rotate

    }
    private fun moving() {
        if (currentBall == null) {
            robotState = States.INTAKING
        } else {
            robotState = States.FULL
        }

    }

    private fun full() {
        if (shootRequsted) {
            io.shooter =
                0.8 //0.8 is filler target power for now, we will do testing tmrw to see the actual target power)
            robotState = States.SHOOTING
        }
    }

    private fun startMoving_Shooting() {
        spindexerRotation += changeMode
        spindexerState[1] == spindexerState[0] //shifting, also why does this work with == but not =???
        spindexerState[2] == spindexerState[1]
        spindexerState[0] == spindexerState[2]
        currentBall = spindexerState[0]
        spindexerRotation = Rotate
    }
    private fun moving_shooting() {
        robotState = States.FULL
    }

    private fun startShooting() {
        startSpinupTime = io.time()
    }

    var startSpinupTime: Duration = 0.seconds //change value later with testing
    private fun shooting() {
        if (currentBall != null) {
            if (io.time() - startSpinupTime > 400.milliseconds) {
                io.transfer = 0.7 //test value for now, we'll do testing tmrw and adjust
            }
            io.shooter = 0.8
        } else {
            if (spindexerState.all { it == null }) {
                robotState = States.IDLE
            } else {
                robotState = States.MOVING_SHOOTING
            }
        }
    }

    fun update(
        io: SigmaIO,
        spindexerRotation: Double,
        currentBall: Balls?,
        deltaT: Duration
    ) {
        // val ball: Balls? = TODO()
        val robotStateOld = robotState
        when(robotState) {
            States.IDLE -> idle()
            States.INTAKING -> intaking()
            States.MOVING -> moving()
            States.FULL -> full()
            States.MOVING_SHOOTING -> moving_shooting()
            States.SHOOTING -> shooting()
        }
        if (robotState != robotStateOld) {
            when(robotState) {
                States.IDLE -> {}
                States.INTAKING -> {}
                States.MOVING -> startmoving()
                States.FULL -> {}
                States.MOVING_SHOOTING -> startMoving_Shooting()
                States.SHOOTING -> startShooting()
            }
        }
        intakeRequested = false
        intakeStopRequested = false
        shootRequsted = false

        //PID
        io.spindexer = spindexerPID.update(spindexerRotation,deltaT)
    }
}

