package sigmacorns.sim

import com.qualcomm.robotcore.hardware.Gamepad
import org.lwjgl.glfw.GLFW

/**
 * Reads a physical gamepad via GLFW and writes its state into an FTC SDK [Gamepad] object.
 *
 * Supports hotplug: if no joystick is connected at creation time, or if the current
 * joystick is disconnected, [tick] will scan for a newly connected joystick each call.
 *
 * Call [tick] each sim loop iteration to update the gamepad values.
 * GLFW is initialized automatically on first construction.
 */
class SimGamepad(private val gamepad: Gamepad) {

    private var jid: Int = -1

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

        // Axes (standard Xbox-style mapping)
        gamepad.left_stick_x  = -axes.get(0)
        gamepad.left_stick_y  = axes.get(1)
        gamepad.right_stick_x = -axes.get(3)
        gamepad.right_stick_y = axes.get(4)
        gamepad.left_trigger  = axes.get(2)
        gamepad.right_trigger = axes.get(5)

        // Buttons (standard Xbox-style mapping)
        fun btn(index: Int): Boolean =
            index < buttons.limit() && buttons.get(index).toInt() == 1

        gamepad.a = btn(0)
        gamepad.b = btn(1)
        gamepad.x = btn(2)
        gamepad.y = btn(3)
        gamepad.left_bumper  = btn(4)
        gamepad.right_bumper = btn(5)
        gamepad.back  = btn(6)
        gamepad.start = btn(7)
        gamepad.left_stick_button  = btn(8)
        gamepad.right_stick_button = btn(9)
        gamepad.guide    = btn(10)
        gamepad.dpad_up    = btn(11)
        gamepad.dpad_right = btn(12)
        gamepad.dpad_down  = btn(13)
        gamepad.dpad_left  = btn(14)
    }

    private fun scanForJoystick() {
        for (id in GLFW.GLFW_JOYSTICK_1..GLFW.GLFW_JOYSTICK_LAST) {
            if (GLFW.glfwJoystickPresent(id)) {
                if (id != jid) {
                    println("Joystick $id connected: ${GLFW.glfwGetJoystickName(id)}")
                }
                jid = id
                return
            }
        }
        jid = -1
    }

    companion object {
        private var glfwInitialized = false
    }
}