package sigmacorns.opmode.tune

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer

@Configurable
object BlockerTunerConfig {
    @JvmField var engaged = IntakeTransfer.BLOCKER_ENGAGED
    @JvmField var disengaged = IntakeTransfer.BLOCKER_DISENGAGED
    @JvmField var step = 0.005
}

@TeleOp(name = "Blocker Tuner", group = "Tune")
class BlockerTunerOpMode : SigmaOpMode() {

    override fun runOpMode() {
        var outputDisengaged = false
        var tuningDisengaged = false

        var lastA = false
        var lastX = false
        var lastB = false
        var lastY = false
        var lastUp = false
        var lastDown = false

        telemetry.addLine("Blocker Tuner")
        telemetry.addLine("A: toggle output preset")
        telemetry.addLine("X: output engaged preset")
        telemetry.addLine("B: output disengaged preset")
        telemetry.addLine("Y: choose preset to edit")
        telemetry.addLine("D-pad Up/Down: adjust selected preset")
        telemetry.update()

        waitForStart()

        ioLoop { _, _ ->
            if (gamepad1.a && !lastA) outputDisengaged = !outputDisengaged
            if (gamepad1.x && !lastX) outputDisengaged = false
            if (gamepad1.b && !lastB) outputDisengaged = true
            if (gamepad1.y && !lastY) tuningDisengaged = !tuningDisengaged

            if (gamepad1.dpad_up && !lastUp) {
                if (tuningDisengaged) {
                    BlockerTunerConfig.disengaged = (BlockerTunerConfig.disengaged + BlockerTunerConfig.step).coerceIn(0.0, 1.0)
                } else {
                    BlockerTunerConfig.engaged = (BlockerTunerConfig.engaged + BlockerTunerConfig.step).coerceIn(0.0, 1.0)
                }
            }

            if (gamepad1.dpad_down && !lastDown) {
                if (tuningDisengaged) {
                    BlockerTunerConfig.disengaged = (BlockerTunerConfig.disengaged - BlockerTunerConfig.step).coerceIn(0.0, 1.0)
                } else {
                    BlockerTunerConfig.engaged = (BlockerTunerConfig.engaged - BlockerTunerConfig.step).coerceIn(0.0, 1.0)
                }
            }

            io.intake = 0.0
            io.blocker = if (outputDisengaged) BlockerTunerConfig.disengaged else BlockerTunerConfig.engaged

            lastA = gamepad1.a
            lastX = gamepad1.x
            lastB = gamepad1.b
            lastY = gamepad1.y
            lastUp = gamepad1.dpad_up
            lastDown = gamepad1.dpad_down

            telemetry.addData("Output", if (outputDisengaged) "DISENGAGED" else "ENGAGED")
            telemetry.addData("Editing", if (tuningDisengaged) "DISENGAGED" else "ENGAGED")
            telemetry.addData("Engaged preset", "%.3f".format(BlockerTunerConfig.engaged))
            telemetry.addData("Disengaged preset", "%.3f".format(BlockerTunerConfig.disengaged))
            telemetry.addData("Step", "%.3f".format(BlockerTunerConfig.step))
            telemetry.addData("Servo command", "%.3f".format(io.blocker))
            telemetry.addLine("Copy final values into IntakeTransfer constants")
            telemetry.update()

            false
        }
    }
}
