export const CONTROLLER_TYPES = { XBOX: 'Xbox', PLAYSTATION: 'PlayStation', GENERIC: 'Generic' };

export function detectControllerType(gamepadId) {
    const id = (gamepadId || '').toLowerCase();
    if (id.includes('xbox') || id.includes('xinput') || id.includes('microsoft') || id.includes('045e')) {
        return CONTROLLER_TYPES.XBOX;
    }
    if (id.includes('playstation') || id.includes('dualshock') || id.includes('dualsense') ||
        id.includes('054c') || id.includes('sony') || id.includes('wireless controller')) {
        return CONTROLLER_TYPES.PLAYSTATION;
    }
    return CONTROLLER_TYPES.GENERIC;
}

// W3C Standard Gamepad mapping — used when gp.mapping === "standard".
// Both Xbox and PlayStation controllers use this layout in Chrome and most modern browsers.
export const MAPPING_STANDARD = {
    name: 'Standard',
    axes: { LX: 0, LY: 1, RX: 2, RY: 3 },
    buttons: { A: 0, B: 1, X: 2, Y: 3, LB: 4, RB: 5, LT: 6, RT: 7,
               BACK: 8, START: 9, LS: 10, RS: 11,
               DPAD_UP: 12, DPAD_DOWN: 13, DPAD_LEFT: 14, DPAD_RIGHT: 15 },
    triggers: 'buttons',
    dpad: 'buttons'
};

// PlayStation non-standard with DPad on axes 6,7
// Common on Linux (Firefox/Chrome without standard mapping support for DS4/DualSense).
export const MAPPING_PS_AXES_DPAD = {
    name: 'PlayStation (non-standard, dpad-axes)',
    axes: { LX: 0, LY: 1, RX: 3, RY: 4, LT: 2, RT: 5, DPAD_X: 6, DPAD_Y: 7 },
    buttons: { A: 0, B: 1, X: 2, Y: 3, LB: 4, RB: 5, LT: 6, RT: 7,
               BACK: 8, START: 9, LS: 10, RS: 11 },
    triggers: 'axes',
    dpad: 'axes'
};

// PlayStation non-standard with DPad on buttons 14-17
export const MAPPING_PS_BTN_DPAD = {
    name: 'PlayStation (non-standard, dpad-buttons)',
    axes: { LX: 0, LY: 1, RX: 3, RY: 4, LT: 2, RT: 5 },
    buttons: { A: 0, B: 1, X: 2, Y: 3, LB: 4, RB: 5, LT: 6, RT: 7,
               BACK: 8, START: 9, LS: 10, RS: 11,
               DPAD_UP: 14, DPAD_DOWN: 15, DPAD_LEFT: 16, DPAD_RIGHT: 17 },
    triggers: 'axes',
    dpad: 'buttons'
};

export let activeMapping = MAPPING_STANDARD;
export let detectedControllerType = CONTROLLER_TYPES.GENERIC;
export let gamepadConnected = false;

const GAMEPAD_DEADZONE = 0.1;

export function resolveMapping(gamepad) {
    if (gamepad.mapping === 'standard') return MAPPING_STANDARD;
    const type = detectControllerType(gamepad.id);
    if (type === CONTROLLER_TYPES.PLAYSTATION) {
        return gamepad.axes.length >= 8 ? MAPPING_PS_AXES_DPAD : MAPPING_PS_BTN_DPAD;
    }
    return MAPPING_STANDARD;
}

export function applyDeadzone(value, deadzone = GAMEPAD_DEADZONE) {
    return Math.abs(value) < deadzone ? 0.0 : value;
}

export function readGamepadState(gp) {
    const m = activeMapping;
    const axis = (i) => gp.axes[i] || 0;
    const btn = (i) => gp.buttons[i] ? gp.buttons[i].pressed : false;
    const btnVal = (i) => gp.buttons[i] ? gp.buttons[i].value : 0;

    let lt, rt;
    if (m.triggers === 'axes') {
        lt = (axis(m.axes.LT) + 1) / 2;
        rt = (axis(m.axes.RT) + 1) / 2;
    } else {
        lt = btnVal(m.buttons.LT);
        rt = btnVal(m.buttons.RT);
    }

    let dpadUp, dpadDown, dpadLeft, dpadRight;
    if (m.dpad === 'axes') {
        const dx = axis(m.axes.DPAD_X);
        const dy = axis(m.axes.DPAD_Y);
        dpadLeft = dx < -0.5;
        dpadRight = dx > 0.5;
        dpadUp = dy < -0.5;
        dpadDown = dy > 0.5;
    } else {
        dpadUp = btn(m.buttons.DPAD_UP);
        dpadDown = btn(m.buttons.DPAD_DOWN);
        dpadLeft = btn(m.buttons.DPAD_LEFT);
        dpadRight = btn(m.buttons.DPAD_RIGHT);
    }

    return {
        left_stick_x: applyDeadzone(axis(m.axes.LX)),
        left_stick_y: applyDeadzone(axis(m.axes.LY)),
        right_stick_x: applyDeadzone(axis(m.axes.RX)),
        right_stick_y: applyDeadzone(axis(m.axes.RY)),
        left_trigger: lt, right_trigger: rt,
        a: btn(m.buttons.A), b: btn(m.buttons.B),
        x: btn(m.buttons.X), y: btn(m.buttons.Y),
        left_bumper: btn(m.buttons.LB), right_bumper: btn(m.buttons.RB),
        back: btn(m.buttons.BACK), start: btn(m.buttons.START),
        left_stick_button: btn(m.buttons.LS), right_stick_button: btn(m.buttons.RS),
        dpad_up: dpadUp, dpad_down: dpadDown,
        dpad_left: dpadLeft, dpad_right: dpadRight
    };
}

export let currentGamepadState = {
    left_stick_x: 0, left_stick_y: 0,
    right_stick_x: 0, right_stick_y: 0,
    left_trigger: 0, right_trigger: 0,
    a: false, b: false, x: false, y: false,
    left_bumper: false, right_bumper: false,
    back: false, start: false,
    left_stick_button: false, right_stick_button: false,
    dpad_up: false, dpad_down: false, dpad_left: false, dpad_right: false
};

export function updateGamepadStatusDisplay() {
    const el = document.getElementById('gamepad-status');
    if (gamepadConnected) {
        el.textContent = `🎮 ${detectedControllerType} (${activeMapping.name})`;
        el.style.color = 'green';
    } else {
        el.textContent = 'No Gamepad';
        el.style.color = 'gray';
    }
}

export function sendGamepadState(ws, gamepadId, state) {
    if (ws.readyState === WebSocket.OPEN && state) {
        ws.send(JSON.stringify({ type: 'gamepad', gamepadId, timestamp: Date.now(), input: state }));
    }
}

export function setupGamepadEvents(gamepadIndex, ws) {
    let logCounter = 0;
    const pollState = () => {
        const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
        const gp = gamepads[gamepadIndex];
        if (!gp) {
            if (logCounter++ % 100 === 0) console.log(`No gamepad at index ${gamepadIndex}`);
            return;
        }
        activeMapping = resolveMapping(gp);
        currentGamepadState = readGamepadState(gp);
        if (logCounter++ % 250 === 0) {
            console.log(`Gamepad [${detectedControllerType}] state: LX=${currentGamepadState.left_stick_x.toFixed(2)}, LY=${currentGamepadState.left_stick_y.toFixed(2)}, A=${currentGamepadState.a}`);
        }
        sendGamepadState(ws, 1, currentGamepadState);
    };
    return setInterval(pollState, 20);
}

export function initGamepad(ws) {
    if (typeof gameControl === 'undefined') {
        setTimeout(() => initGamepad(ws), 100);
        return;
    }
    let gamepadPollInterval = null;
    gameControl.on('connect', (gamepad) => {
        const rawGp = navigator.getGamepads()[gamepad.id];
        const gpName = rawGp?.id || 'Unknown';
        detectedControllerType = detectControllerType(gpName);
        activeMapping = rawGp ? resolveMapping(rawGp) : MAPPING_STANDARD;
        console.log(`Gamepad connected: index=${gamepad.id}, name=${gpName}, type=${detectedControllerType}, mapping=${activeMapping.name} (browser mapping="${rawGp?.mapping}"), axes=${rawGp?.axes?.length}, buttons=${rawGp?.buttons?.length}`);
        gamepadConnected = true;
        updateGamepadStatusDisplay();
        if (gamepadPollInterval) clearInterval(gamepadPollInterval);
        gamepadPollInterval = setupGamepadEvents(gamepad.id, ws);
    });
    gameControl.on('disconnect', () => {
        gamepadConnected = false;
        updateGamepadStatusDisplay();
        if (gamepadPollInterval) { clearInterval(gamepadPollInterval); gamepadPollInterval = null; }
    });
}
