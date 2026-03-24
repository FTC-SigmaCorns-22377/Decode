package sigmacorns.sim

import com.qualcomm.robotcore.hardware.Gamepad
import org.lwjgl.glfw.GLFW

enum class ControllerType {
    XBOX,
    PLAYSTATION;

    companion object {
        private val PS_NAME_PATTERNS = listOf(
            "wireless controller", "dualsense", "dualshock", "ps5", "ps4", "sony"
        )

        fun fromJoystickName(name: String?): ControllerType {
            val lower = name?.lowercase() ?: return XBOX
            return if (PS_NAME_PATTERNS.any { it in lower }) PLAYSTATION else XBOX
        }
    }
}

data class AxisMapping(
    val leftStickX: Int,
    val leftStickY: Int,
    val rightStickX: Int,
    val rightStickY: Int,
    val leftTrigger: Int,
    val rightTrigger: Int,
    val triggerRange: TriggerRange,
) {
    enum class TriggerRange { ZERO_TO_ONE, MINUS_ONE_TO_ONE }

    fun normalizeTrigger(raw: Float): Float = when (triggerRange) {
        TriggerRange.ZERO_TO_ONE -> raw
        TriggerRange.MINUS_ONE_TO_ONE -> (raw + 1f) / 2f
    }

    companion object {
        val XBOX = AxisMapping(
            leftStickX = 0, leftStickY = 1,
            rightStickX = 3, rightStickY = 4,
            leftTrigger = 2, rightTrigger = 5,
            triggerRange = TriggerRange.ZERO_TO_ONE,
        )

        val PLAYSTATION = AxisMapping(
            leftStickX = 0, leftStickY = 1,
            rightStickX = 2, rightStickY = 5,
            leftTrigger = 3, rightTrigger = 4,
            triggerRange = TriggerRange.MINUS_ONE_TO_ONE,
        )

        fun forController(type: ControllerType): AxisMapping = when (type) {
            ControllerType.XBOX -> XBOX
            ControllerType.PLAYSTATION -> PLAYSTATION
        }
    }
}

data class ButtonMapping(
    val a: Int, val b: Int, val x: Int, val y: Int,
    val leftBumper: Int, val rightBumper: Int,
    val back: Int, val start: Int,
    val leftStickButton: Int, val rightStickButton: Int,
    val guide: Int,
    val dpadUp: Int, val dpadRight: Int, val dpadDown: Int, val dpadLeft: Int,
) {
    companion object {
        val XBOX = ButtonMapping(
            a = 0, b = 1, x = 2, y = 3,
            leftBumper = 4, rightBumper = 5,
            back = 6, start = 7,
            leftStickButton = 8, rightStickButton = 9,
            guide = 10,
            dpadUp = 11, dpadRight = 12, dpadDown = 13, dpadLeft = 14,
        )

        val PLAYSTATION = ButtonMapping(
            a = 0, b = 1, x = 2, y = 3,
            leftBumper = 4, rightBumper = 5,
            back = 8, start = 9,
            leftStickButton = 10, rightStickButton = 11,
            guide = 12,
            dpadUp = 14, dpadRight = 15, dpadDown = 16, dpadLeft = 17,
        )

        fun forController(type: ControllerType): ButtonMapping = when (type) {
            ControllerType.XBOX -> XBOX
            ControllerType.PLAYSTATION -> PLAYSTATION
        }
    }
}

/**
 * Reads a physical gamepad via GLFW and writes its state into an FTC SDK [Gamepad] object.
 *
 * Supports hotplug: if no joystick is connected at creation time, or if the current
 * joystick is disconnected, [tick] will scan for a newly connected joystick each call.
 *
 * Automatically detects Xbox vs PlayStation controllers and applies the correct
 * axis/button mapping. PlayStation triggers are normalized from [-1, 1] to [0, 1].
 *
 * Call [tick] each sim loop iteration to update the gamepad values.
 * GLFW is initialized automatically on first construction.
 */
class SimGamepad(private val gamepad: Gamepad) {

    private var jid: Int = -1
    private var axisMapping: AxisMapping = AxisMapping.XBOX
    private var buttonMapping: ButtonMapping = ButtonMapping.XBOX

    init {
        if (!glfwInitialized) {
            check(GLFW.glfwInit()) { "Unable to initialize GLFW" }
            glfwInitialized = true
        }
        scanForJoystick()
    }

    val connected: Boolean get() = jid >= 0 && GLFW.glfwJoystickPresent(jid)

    /**
     * Poll GLFW and write the current joystick state into the wrapped [Gamepad].
     * If no joystick is connected, scans for a newly plugged-in one.
     */
    fun tick() {
        if (!connected) {
            scanForJoystick()
            if (!connected) return
        }

        val axes = GLFW.glfwGetJoystickAxes(jid) ?: return
        val buttons = GLFW.glfwGetJoystickButtons(jid) ?: return
        val am = axisMapping
        val bm = buttonMapping

        gamepad.left_stick_x  = applyDeadzone(-axes.get(am.leftStickX))
        gamepad.left_stick_y  = applyDeadzone(axes.get(am.leftStickY))
        gamepad.right_stick_x = applyDeadzone(-axes.get(am.rightStickX))
        gamepad.right_stick_y = applyDeadzone(axes.get(am.rightStickY))
        gamepad.left_trigger  = am.normalizeTrigger(axes.get(am.leftTrigger))
        gamepad.right_trigger = am.normalizeTrigger(axes.get(am.rightTrigger))

        fun btn(index: Int): Boolean =
            index < buttons.limit() && buttons.get(index).toInt() == 1

        gamepad.a = btn(bm.a)
        gamepad.b = btn(bm.b)
        gamepad.x = btn(bm.x)
        gamepad.y = btn(bm.y)
        gamepad.left_bumper  = btn(bm.leftBumper)
        gamepad.right_bumper = btn(bm.rightBumper)
        gamepad.back  = btn(bm.back)
        gamepad.start = btn(bm.start)
        gamepad.left_stick_button  = btn(bm.leftStickButton)
        gamepad.right_stick_button = btn(bm.rightStickButton)
        gamepad.guide    = btn(bm.guide)
        gamepad.dpad_up    = btn(bm.dpadUp)
        gamepad.dpad_right = btn(bm.dpadRight)
        gamepad.dpad_down  = btn(bm.dpadDown)
        gamepad.dpad_left  = btn(bm.dpadLeft)
    }

    private fun scanForJoystick() {
        for (id in GLFW.GLFW_JOYSTICK_1..GLFW.GLFW_JOYSTICK_LAST) {
            if (GLFW.glfwJoystickPresent(id)) {
                if (id != jid) {
                    val name = GLFW.glfwGetJoystickName(id)
                    val controllerType = ControllerType.fromJoystickName(name)
                    axisMapping = AxisMapping.forController(controllerType)
                    buttonMapping = ButtonMapping.forController(controllerType)
                    println("Joystick $id connected: $name (detected: $controllerType)")
                }
                jid = id
                return
            }
        }
        jid = -1
    }

    companion object {
        private var glfwInitialized = false
        private const val STICK_DEADZONE = 0.1f

        private fun applyDeadzone(value: Float): Float {
            if (kotlin.math.abs(value) < STICK_DEADZONE) return 0f
            val sign = if (value > 0) 1f else -1f
            return sign * (kotlin.math.abs(value) - STICK_DEADZONE) / (1f - STICK_DEADZONE)
        }
    }
}
