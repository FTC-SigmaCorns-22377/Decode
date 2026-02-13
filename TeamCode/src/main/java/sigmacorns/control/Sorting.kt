package sigmacorns.control

import kotlinx.coroutines.delay
import sigmacorns.control.subsystem.SpindexerLogic
import sigmacorns.sim.Balls
import sigmacorns.opmode.tune.ShooterFlywheelPIDConfig

/**
 * Sorted shooting that checks the color sensor on-demand.
 *
 * For each motif position:
 *   1. Rotate the spindexer until the color sensor sees the required color
 *      (or until we've rotated through all 3 slots without finding it).
 *   2. Spin up the flywheel.
 *   3. Fire the ball.
 *   4. Repeat for the next motif position.
 *
 * @param motif The desired shoot order, e.g. [Green, Purple, Green].
 *              null entries mean "shoot whatever is in that slot".
 */
suspend fun SpindexerLogic.sortedShoot(motif: List<Balls?> = listOf(Balls.Green, Balls.Green, Balls.Purple)) {
    // Ensure we're in shoot-mode offset
    if (!offsetActive) {
        spindexerRotation += MODE_CHANGE_ANGLE
        offsetActive = true
        awaitSpindexerTarget()
    }
    resetTransfer()

    for (requiredColor in motif) {
        var foundBall = false

        if (requiredColor != null) {
            // Rotate through slots looking for the required color
            for (rotations in 0..2) {
                if (io.colorSensorDetectsBall()) {
                    val currentColor = io.colorSensorGetBallColor()
                    if (currentColor == requiredColor) {
                        foundBall = true
                        break
                    }
                }

                // Try next slot
                if (rotations < 2) {
                    rotateFwd()
                }
            }

            if (!foundBall) {
                println("Sorted shoot: Color $requiredColor not found, skipping")
                continue
            }
        } else {
            // requiredColor is null — shoot whatever is at the current slot
            if (!io.colorSensorDetectsBall()) {
                // No ball at current slot, search for any ball
                for (rotations in 1..2) {
                    rotateFwd()
                    if (io.colorSensorDetectsBall()) {
                        foundBall = true
                        break
                    }
                }
                if (!foundBall) {
                    println("Sorted shoot: No balls remaining")
                    break
                }
            }
        }

        // Spin up flywheel
        val startTime = io.time()
        while (true) {
            val currentVelocity = io.flywheelVelocity()
            val targetVelocity = shotPower * ShooterFlywheelPIDConfig.maxVelocity
            val error = kotlin.math.abs(currentVelocity - targetVelocity)

            if (error < VELOCITY_ERROR_THRESHOLD) {
                break
            }
            if (io.time() - startTime > MAX_WAIT_TIME) {
                println("Warning: Flywheel spinup timeout (sorted shoot)")
                break
            }
            delay(10)
        }

        // Fire the ball
        activateTransfer()

        // Brief delay to let the ball clear before checking the next one
        delay(200)
    }

    // Stop flywheel if continuous shooting isn't requested
    if (!shootingRequested) {
        io.shooter = 0.0
    }
}

// ── Helper functions ────────────────────────────────────────────────────────

/**
 * Rotates the spindexer forward by one slot (120 degrees).
 * Waits for the motor to reach the target position.
 */
suspend fun SpindexerLogic.rotateFwd() {
    spindexerRotation += ROTATE_ANGLE
    awaitSpindexerTarget()
}

/**
 * Waits until the spindexer motor is within POSITION_ERROR_THRESHOLD
 * of the target rotation, with a timeout safety.
 */
internal suspend fun SpindexerLogic.awaitSpindexerTarget() {
    val startTime = io.time()
    while (true) {
        val error = kotlin.math.abs(spindexer.curRotation - spindexerRotation)
        if (error < POSITION_ERROR_THRESHOLD) {
            break
        }
        if (io.time() - startTime > MAX_WAIT_TIME) {
            println("Warning: Spindexer movement timeout")
            break
        }
        delay(10)
    }
}


