package sigmacorns.logic

import sigmacorns.sim.MecanumState
import kotlin.time.Duration

/**
 * A pre-planned shot target provided by an autonomous routine.
 *
 * When set on [sigmacorns.control.aim.AutoAim], [sigmacorns.logic.NativeAutoAim] uses
 * [timeToArrival] to:
 * - Prespin the flywheel as soon as [timeToArrival] drops within [AimConfig.spinupLeadTime]
 *   (rather than relying on the coarse zone-proximity estimate)
 * - Pass the exact remaining time to the C++ solver as [solverTRemaining] so it pre-aims
 *   hood and turret for the shot at [state]
 *
 * @param state         the [MecanumState] (position + velocity) the robot will occupy when it shoots.
 * @param timeToArrival estimated time from the moment this plan is handed to the auto-aim until
 *                      the robot reaches [state].
 */
data class PlannedShot(val state: MecanumState, val timeToArrival: Duration)
