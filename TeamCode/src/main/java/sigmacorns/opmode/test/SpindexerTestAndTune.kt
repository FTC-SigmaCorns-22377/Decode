package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.control.SpindexerLogic
import kotlin.time.Duration.Companion.seconds

@TeleOp
class SpindexerTestAndTune(): SigmaOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val spindexerLogic = SpindexerLogic(io)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addLine("Spindexer Angle ${io.spindexer}")
            //telemetry.addLine("Color Sensed ${io.colorSensor}")

            telemetry.update()

            if (gamepad1.aWasReleased()) {
                spindexerLogic.fsm.sendEvent(SpindexerLogic.Event.START_INTAKING) //have to send in the event from the type level because enum classes are at the type level
            }
            if (gamepad1.bWasReleased()) {
                spindexerLogic.fsm.sendEvent(SpindexerLogic.Event.STOP_INTAKING) //have to send in the event from the type level because enum classes are at the type level
            }
            if (gamepad1.yWasReleased()) {
                spindexerLogic.fsm.sendEvent(SpindexerLogic.Event.SHOOT) //have to send in the event from the type level because enum classes are at the type level
            }
            if (gamepad1.xWasReleased()) {
                spindexerLogic.fsm.sendEvent(SpindexerLogic.Event.BALL_DETECTED)
            }

            spindexerLogic.update(io.spindexerPosition(), deltaT = 0.seconds)
            io.update()

        }
    }
}