package sigmacorns.control

import sigmacorns.io.SigmaIO
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.time.Duration

class Turret(
    val range: MotorRangeMapper,
    val io: SigmaIO
) {
    val angleController: PIDController = PIDController(4.0,1.0,0.15,0.0)

    // distance(m) from target
    var targetDistance: Double = 0.0

    // angle(rad) in robot-relative frame (Yaw)
    var targetAngle: Double = 0.0
    
    // Pitch value for servo (0.0 to 1.0)
    var targetPitch: Double = 0.0

    var pos = 0.0

    var staticCompensationThresh = 0.01
    var staticPower = 0.03


    fun update(dt: Duration) {
        val currentAngle = range.tickToPos(io.turretPosition())
        angleController.setpoint = targetAngle

        pos = currentAngle
        
        // Calculate turret motor power (Yaw)
        var turretPower = angleController.update(currentAngle, dt)

        if((pos-targetAngle).absoluteValue > staticCompensationThresh) turretPower += staticPower*turretPower.sign

        val limitedPower = turretPower

        val flywheelPower = flywheelSpeed()

        io.turret = limitedPower
        io.shooter = flywheelPower
    }

    private val MAX_LAUNCH_DIST = 10.0
    private fun flywheelSpeed(): Double {
        return (targetDistance/MAX_LAUNCH_DIST).coerceIn(0.0, 1.0)
    }
}