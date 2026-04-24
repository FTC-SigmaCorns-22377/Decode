package sigmacorns.opmode.test

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.opmode.SigmaOpMode
import kotlin.math.max

/**
 * Flywheel + hood test. Direct motor-power control; no PID or velocity target.
 * Either gamepad works (inputs merged):
 *   RT        → flywheel power (0..1)
 *   A         → flywheel full power (1.0)
 *   B         → flywheel off
 *   LB press  → hood servo -0.02 (rising edge)
 *   RB press  → hood servo +0.02 (rising edge)
 *   D-pad up  → hood to 0.5 (midpoint)
 *
 * Diagnostic: probes the hardwareMap at INIT. If you see "shooter1 MISSING",
 * add it to your DS robot config; otherwise the motor write is a silent no-op
 * and the flywheel won't spin no matter what telemetry says.
 */
@TeleOp(name = "Shooter Test Bot 2", group = "Test")
class ShooterTestBot2 : SigmaOpMode() {

    override fun runOpMode() {
        val hasShooter1 = tryDevice(DcMotorEx::class.java, "shooter1")
        val hasShooter2 = tryDevice(DcMotorEx::class.java, "shooter2")
        val hasHood     = tryDevice(Servo::class.java,     "hood")

        var hoodPos = 0.5
        io.hood = hoodPos
        io.flywheel = 0.0

        telemetry.addLine("Shooter Test Bot 2")
        telemetry.addLine("--- hardware probe ---")
        telemetry.addData("shooter1 (DcMotor)", if (hasShooter1) "FOUND" else "MISSING — add to DS config")
        telemetry.addData("shooter2 (DcMotor)", if (hasShooter2) "FOUND" else "MISSING — add to DS config")
        telemetry.addData("hood (Servo)",       if (hasHood)     "FOUND" else "MISSING — add to DS config")
        telemetry.addLine("-----------------------")
        telemetry.addLine("Either gamepad: RT flywheel, A full, B off")
        telemetry.addLine("LB/RB hood -/+, D-pad up hood=0.5")
        telemetry.update()

        waitForStart()

        var lastLb = false
        var lastRb = false

        while (opModeIsActive()) {
            val lb = gamepad1.left_bumper  || gamepad2.left_bumper
            val rb = gamepad1.right_bumper || gamepad2.right_bumper
            val up = gamepad1.dpad_up || gamepad2.dpad_up
            val a  = gamepad1.a || gamepad2.a
            val b  = gamepad1.b || gamepad2.b
            val rt = max(gamepad1.right_trigger, gamepad2.right_trigger).toDouble()

            if (lb && !lastLb) hoodPos = (hoodPos - 0.02).coerceIn(0.0, 1.0)
            if (rb && !lastRb) hoodPos = (hoodPos + 0.02).coerceIn(0.0, 1.0)
            if (up) hoodPos = 0.5
            lastLb = lb
            lastRb = rb

            io.hood = hoodPos

            io.flywheel = when {
                a -> 1.0
                b -> 0.0
                rt > 0.05 -> rt
                else -> 0.0
            }

            io.update()

            telemetry.addData("shooter1", if (hasShooter1) "OK" else "MISSING")
            telemetry.addData("shooter2", if (hasShooter2) "OK" else "MISSING")
            telemetry.addData("hood",     if (hasHood)     "OK" else "MISSING")
            telemetry.addData("flywheel pwr", "%.2f".format(io.flywheel))
            telemetry.addData("flywheel vel", "%.1f rad/s".format(io.flywheelVelocity()))
            telemetry.addData("hood pos", "%.2f".format(hoodPos))
            telemetry.update()
        }

        io.flywheel = 0.0
        io.update()
    }

    private fun <T> tryDevice(cls: Class<T>, name: String): Boolean =
        try { hardwareMap.get(cls, name); true } catch (_: Exception) { false }
}
