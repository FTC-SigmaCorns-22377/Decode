package sigmacorns.control

import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

class TrapezoidalProfile(
    var maxVelocity: Double,
    var maxAcceleration: Double
) {
    var currentPosition: Double = 0.0
    var currentVelocity: Double = 0.0

    data class State(val position: Double, val velocity: Double)

    fun reset(position: Double, velocity: Double = 0.0) {
        currentPosition = position
        currentVelocity = velocity
    }

    fun calculate(targetPosition: Double, dt: Double): State {
        val direction = sign(targetPosition - currentPosition)
        val distanceToTarget = abs(targetPosition - currentPosition)
        
        if (distanceToTarget < 1e-6) {
            currentVelocity = 0.0
            currentPosition = targetPosition
            return State(currentPosition, currentVelocity)
        }

        // Calculate the velocity we would have if we started decelerating now to hit 0 at target
        // v^2 = 2 * a * d => v = sqrt(2 * a * d)
        val maxReachableVelocity = sqrt(2 * maxAcceleration * distanceToTarget)

        val desiredVelocity = if (maxReachableVelocity < abs(currentVelocity)) {
            // We are going too fast to stop in time, must decelerate hard (or we are already decelerating)
            // Actually, if we are in this branch, it means we need to slow down to hit the target.
             maxReachableVelocity * direction
        } else {
            // We can accelerate or cruise
            val v = abs(currentVelocity) + maxAcceleration * dt
            min(v, maxVelocity) * direction
        }

        // Apply acceleration limit to velocity change
        val velocityStep = desiredVelocity - currentVelocity
        val maxStep = maxAcceleration * dt
        
        currentVelocity += velocityStep.coerceIn(-maxStep, maxStep)
        
        // Update position
        currentPosition += currentVelocity * dt
        
        return State(currentPosition, currentVelocity)
    }
}
