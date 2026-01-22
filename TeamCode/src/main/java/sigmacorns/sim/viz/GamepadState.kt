package sigmacorns.sim.viz

data class GamepadState(
    val left_stick_x: Double = 0.0,
    val left_stick_y: Double = 0.0,
    val right_stick_x: Double = 0.0,
    val right_stick_y: Double = 0.0,
    val left_trigger: Double = 0.0,
    val right_trigger: Double = 0.0,
    val a: Boolean = false,
    val b: Boolean = false,
    val x: Boolean = false,
    val y: Boolean = false,
    val back: Boolean = false,
    val start: Boolean = false,
    val left_bumper: Boolean = false,
    val right_bumper: Boolean = false,
    val left_stick_button: Boolean = false,
    val right_stick_button: Boolean = false,
    val dpad_up: Boolean = false,
    val dpad_down: Boolean = false,
    val dpad_left: Boolean = false,
    val dpad_right: Boolean = false
)
