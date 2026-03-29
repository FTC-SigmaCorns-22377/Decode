package sigmacorns.test

import com.qualcomm.robotcore.hardware.Gamepad
import org.junit.jupiter.api.Test
import org.lwjgl.glfw.GLFW
import sigmacorns.sim.AxisMapping
import sigmacorns.sim.ButtonMapping
import sigmacorns.sim.ControllerType
import sigmacorns.sim.SimGamepad

class GamepadTest {

    /**
     * Test 1: Can GLFW initialize at all?
     */
    @Test
    fun testGlfwInit() {
        val result = GLFW.glfwInit()
        println("GLFW init: $result")
        assert(result) { "GLFW failed to initialize" }
        GLFW.glfwTerminate()
    }

    /**
     * Test 2: Enumerate all GLFW joysticks and print their names/GUIDs.
     * This tells us if GLFW can see any controllers.
     */
    @Test
    fun testEnumerateJoysticks() {
        check(GLFW.glfwInit()) { "GLFW failed to initialize" }

        var found = 0
        for (id in GLFW.GLFW_JOYSTICK_1..GLFW.GLFW_JOYSTICK_LAST) {
            if (GLFW.glfwJoystickPresent(id)) {
                val name = GLFW.glfwGetJoystickName(id)
                val guid = GLFW.glfwGetJoystickGUID(id)
                val isGamepad = GLFW.glfwJoystickIsGamepad(id)
                val axes = GLFW.glfwGetJoystickAxes(id)
                val buttons = GLFW.glfwGetJoystickButtons(id)
                println("Joystick $id:")
                println("  Name:      $name")
                println("  GUID:      $guid")
                println("  Gamepad:   $isGamepad")
                println("  Axes:      ${axes?.limit() ?: "null"}")
                println("  Buttons:   ${buttons?.limit() ?: "null"}")
                found++
            }
        }

        if (found == 0) {
            println("NO JOYSTICKS FOUND BY GLFW")
            println("But the OS sees these joystick devices:")
            val proc = ProcessBuilder("cat", "/proc/bus/input/devices").start()
            val output = proc.inputStream.bufferedReader().readText()
            output.lines().windowed(6, 1).filter { window ->
                window.any { it.contains("js") }
            }.flatten().distinct().forEach { println("  $it") }
        }

        GLFW.glfwTerminate()
    }

    /**
     * Test 3: Check if GLFW detects the controller type correctly.
     */
    @Test
    fun testControllerTypeDetection() {
        check(GLFW.glfwInit()) { "GLFW failed to initialize" }

        for (id in GLFW.GLFW_JOYSTICK_1..GLFW.GLFW_JOYSTICK_LAST) {
            if (GLFW.glfwJoystickPresent(id)) {
                val name = GLFW.glfwGetJoystickName(id)
                val detected = ControllerType.fromJoystickName(name)
                println("Joystick $id: '$name' -> $detected")

                if (name?.lowercase()?.let { n ->
                    listOf("sony", "dualsense", "dualshock", "ps5", "ps4", "wireless controller").any { it in n }
                } == true) {
                    assert(detected == ControllerType.PLAYSTATION) {
                        "PS controller '$name' was detected as $detected instead of PLAYSTATION"
                    }
                    println("  -> Correctly detected as PlayStation")
                }
            }
        }

        GLFW.glfwTerminate()
    }

    /**
     * Test 4: Read raw GLFW axes and buttons for 3 seconds, printing values.
     * Move the sticks and press buttons during this test to see if data comes through.
     */
    @Test
    fun testRawJoystickInput() {
        check(GLFW.glfwInit()) { "GLFW failed to initialize" }

        var jid = -1
        for (id in GLFW.GLFW_JOYSTICK_1..GLFW.GLFW_JOYSTICK_LAST) {
            if (GLFW.glfwJoystickPresent(id) && GLFW.glfwJoystickIsGamepad(id)) {
                jid = id
                println("Using joystick $id: ${GLFW.glfwGetJoystickName(id)}")
                break
            }
        }

        if (jid == -1) {
            println("No joystick found, skipping raw input test")
            GLFW.glfwTerminate()
            return
        }

        println("Reading raw input for 3 seconds — move sticks and press buttons now...")
        val start = System.currentTimeMillis()
        var lastPrint = 0L
        while (System.currentTimeMillis() - start < 3000) {
            GLFW.glfwPollEvents()
            val axes = GLFW.glfwGetJoystickAxes(jid)
            val buttons = GLFW.glfwGetJoystickButtons(jid)

            val now = System.currentTimeMillis()
            if (now - lastPrint >= 200) {
                lastPrint = now
                if (axes != null) {
                    val axisVals = (0 until axes.limit()).map { "%.2f".format(axes.get(it)) }
                    println("  Axes: $axisVals")
                } else {
                    println("  Axes: null (controller disconnected?)")
                }
                if (buttons != null) {
                    val pressed = (0 until buttons.limit()).filter { buttons.get(it).toInt() == 1 }
                    if (pressed.isNotEmpty()) println("  Buttons pressed: $pressed")
                }
            }

            Thread.sleep(10)
        }

        GLFW.glfwTerminate()
    }

    /**
     * Test 5: Check the PlayStation axis mapping specifically.
     * Reads axes for 3 seconds using the PS mapping and prints mapped values.
     */
    @Test
    fun testPlayStationAxisMapping() {
        check(GLFW.glfwInit()) { "GLFW failed to initialize" }

        var jid = -1
        for (id in GLFW.GLFW_JOYSTICK_1..GLFW.GLFW_JOYSTICK_LAST) {
            if (GLFW.glfwJoystickPresent(id) && GLFW.glfwJoystickIsGamepad(id)) {
                jid = id
                break
            }
        }

        if (jid == -1) {
            println("No joystick found")
            GLFW.glfwTerminate()
            return
        }

        val name = GLFW.glfwGetJoystickName(jid)
        val type = ControllerType.fromJoystickName(name)
        val am = AxisMapping.forController(type)
        println("Controller: '$name' -> $type")
        println("Axis mapping: LS_X=${am.leftStickX}, LS_Y=${am.leftStickY}, " +
                "RS_X=${am.rightStickX}, RS_Y=${am.rightStickY}, " +
                "LT=${am.leftTrigger}, RT=${am.rightTrigger}")

        val axes = GLFW.glfwGetJoystickAxes(jid)
        val numAxes = axes?.limit() ?: 0
        println("Controller reports $numAxes axes")

        val maxIdx = maxOf(am.leftStickX, am.leftStickY, am.rightStickX, am.rightStickY,
            am.leftTrigger, am.rightTrigger)
        if (maxIdx >= numAxes) {
            println("ERROR: Mapping requires axis index $maxIdx but controller only has $numAxes axes!")
            println("This is likely the problem — the axis indices in the mapping are wrong for this controller.")
            println("All axis values:")
            if (axes != null) {
                for (i in 0 until numAxes) {
                    println("  Axis $i: ${axes.get(i)}")
                }
            }
        }

        println("\nReading mapped input for 3 seconds — move both sticks and triggers...")
        val start = System.currentTimeMillis()
        var lastPrint = 0L
        while (System.currentTimeMillis() - start < 3000) {
            GLFW.glfwPollEvents()
            val a = GLFW.glfwGetJoystickAxes(jid) ?: break

            val now = System.currentTimeMillis()
            if (now - lastPrint >= 200) {
                lastPrint = now
                fun safeGet(idx: Int) = if (idx < a.limit()) "%.2f".format(a.get(idx)) else "OOB!"
                println("  LS(${safeGet(am.leftStickX)}, ${safeGet(am.leftStickY)})  " +
                        "RS(${safeGet(am.rightStickX)}, ${safeGet(am.rightStickY)})  " +
                        "LT=${safeGet(am.leftTrigger)}  RT=${safeGet(am.rightTrigger)}")
            }
            Thread.sleep(10)
        }

        GLFW.glfwTerminate()
    }

    /**
     * Test 6: End-to-end SimGamepad test — creates a SimGamepad and reads
     * the FTC Gamepad object for 3 seconds.
     */
    @Test
    fun testSimGamepadEndToEnd() {
        val gamepad = Gamepad()
        val simGamepad = SimGamepad(gamepad)

        println("SimGamepad connected: ${simGamepad.connected}")
        if (!simGamepad.connected) {
            println("SimGamepad could not find a controller. Check previous tests for diagnosis.")
            return
        }

        println("Reading SimGamepad for 3 seconds — move sticks and press buttons...")
        val start = System.currentTimeMillis()
        var lastPrint = 0L
        while (System.currentTimeMillis() - start < 3000) {
            simGamepad.tick()

            val now = System.currentTimeMillis()
            if (now - lastPrint >= 200) {
                lastPrint = now
                println("  LS(%.2f, %.2f)  RS(%.2f, %.2f)  LT=%.2f  RT=%.2f  A=%s B=%s X=%s Y=%s".format(
                    gamepad.left_stick_x, gamepad.left_stick_y,
                    gamepad.right_stick_x, gamepad.right_stick_y,
                    gamepad.left_trigger, gamepad.right_trigger,
                    gamepad.a, gamepad.b, gamepad.x, gamepad.y
                ))
            }
            Thread.sleep(10)
        }
    }
}
