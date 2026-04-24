package sigmacorns.opmode.test

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode
import sigmacorns.subsystem.IntakeTransfer
import kotlin.math.max

/**
 * Intake + blocker test. Either gamepad works (inputs merged):
 *   RT → intake forward (0..1 from trigger)
 *   LT → intake reverse
 *   A  → blocker ENGAGED (closed)
 *   B  → blocker DISENGAGED (open)
 *
 * Diagnostic: probes the hardwareMap at INIT and reports which devices were
 * found. If you see "intake1 MISSING", add it to your DS robot config —
 * otherwise io.update() silently no-ops the motor write.
 */
@TeleOp(name = "Intake Test Bot 2", group = "Test")
class IntakeTestBot2 : SigmaOpMode() {

    override fun runOpMode() {
        val hasIntake1 = tryDevice(DcMotorEx::class.java, "intake1")
        val hasIntake2 = tryDevice(DcMotorEx::class.java, "intake2")
        val hasBlocker = tryDevice(Servo::class.java, "blocker")

        telemetry.addLine("Intake Test Bot 2")
        telemetry.addLine("--- hardware probe ---")
        telemetry.addData("intake1 (DcMotor)", if (hasIntake1) "FOUND" else "MISSING — add to DS config")
        telemetry.addData("intake2 (DcMotor)", if (hasIntake2) "FOUND" else "MISSING — add to DS config")
        telemetry.addData("blocker (Servo)",   if (hasBlocker) "FOUND" else "MISSING — add to DS config")
        telemetry.addLine("-----------------------")
        telemetry.addLine("Either gamepad: RT fwd, LT rev. A engaged / B open.")
        telemetry.update()

        io.blocker = IntakeTransfer.BLOCKER_ENGAGED

        waitForStart()

        while (opModeIsActive()) {
            val rt = max(gamepad1.right_trigger, gamepad2.right_trigger).toDouble()
            val lt = max(gamepad1.left_trigger,  gamepad2.left_trigger).toDouble()
            val a  = gamepad1.a || gamepad2.a
            val b  = gamepad1.b || gamepad2.b

            val power = when {
                rt > 0.05 -> rt
                lt > 0.05 -> -lt
                else -> 0.0
            }
            io.intake = power

            if (a) io.blocker = IntakeTransfer.BLOCKER_ENGAGED
            if (b) io.blocker = IntakeTransfer.BLOCKER_DISENGAGED

            io.update()

            telemetry.addData("intake1", if (hasIntake1) "OK" else "MISSING")
            telemetry.addData("intake2", if (hasIntake2) "OK" else "MISSING")
            telemetry.addData("blocker", if (hasBlocker) "OK" else "MISSING")
            telemetry.addData("intake pwr", "%+.2f".format(io.intake))
            telemetry.addData("blocker pos", "%.2f".format(io.blocker))
            telemetry.addData("beam breaks", "1=%b 2=%b 3=%b".format(
                io.beamBreak1(), io.beamBreak2(), io.beamBreak3()))
            telemetry.addData("intake1 RPM", "%.0f".format(io.intake1RPM()))
            telemetry.update()
        }

        io.intake = 0.0
        io.blocker = IntakeTransfer.BLOCKER_ENGAGED
        io.update()
    }

    private fun <T> tryDevice(cls: Class<T>, name: String): Boolean =
        try { hardwareMap.get(cls, name); true } catch (_: Exception) { false }
}
