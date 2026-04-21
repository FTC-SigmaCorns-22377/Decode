import { updateRobot, updateBalls, updateGoals, updateShotViz, updateTracker } from './scene.js';

const frames = [];
let isLive = true;
let isPaused = false;

const scrubber = document.getElementById('scrubber');
const timeDisplay = document.getElementById('time-display');
const posDisplay = document.getElementById('pos-display');
const playPauseBtn = document.getElementById('play-pause');
const liveBtn = document.getElementById('live-btn');

// WebSocket connection
const wsUrl = `ws://${window.location.host}/sim`;
let ws = null;

function connect() {
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        ws.send(JSON.stringify({ ready: true }));
    };

    ws.onmessage = (event) => {
        const state = JSON.parse(event.data);
        frames.push(state);
        scrubber.max = frames.length - 1;

        if (isLive && !isPaused) {
            scrubber.value = frames.length - 1;
            applyFrame(state);
        }
    };

    ws.onclose = () => {
        setTimeout(connect, 1000);
    };

    ws.onerror = () => {
        ws.close();
    };
}

function applyFrame(state) {
    if (!state) return;
    updateRobot(state.robot.x, state.robot.y, state.robot.theta,
        state.robot.turretAngle, state.robot.intakeAngle,
        state.robot.hoodAngle, state.robot.flywheelRPM, state.robot.intakeRollerRPM);
    updateBalls(state.balls || []);
    updateGoals(state.goals);
    updateShotViz(state.shotViz);
    updateTracker(state.tracker);
    timeDisplay.textContent = state.t.toFixed(3) + 's';
    const held = state.heldBalls ? state.heldBalls.length : 0;
    posDisplay.innerHTML = `X: ${(-state.robot.x).toFixed(3)}<br>Y: ${state.robot.y.toFixed(3)}<br>Flywheel: ${Math.round(state.robot.flywheelRPM)} RPM<br>Held: ${held}/3`;
}

// Controls
playPauseBtn.addEventListener('click', () => {
    isPaused = !isPaused;
    playPauseBtn.textContent = isPaused ? 'Play' : 'Pause';
});

liveBtn.addEventListener('click', () => {
    isLive = true;
    liveBtn.classList.add('active');
    if (frames.length > 0) {
        scrubber.value = frames.length - 1;
        applyFrame(frames[frames.length - 1]);
    }
});

scrubber.addEventListener('input', () => {
    isLive = false;
    liveBtn.classList.remove('active');
    const idx = parseInt(scrubber.value);
    if (idx >= 0 && idx < frames.length) {
        applyFrame(frames[idx]);
    }
});

// Keyboard input — driver (WASD) and operator (number keys, space, shift)
const keys = {
    w: false, a: false, s: false, d: false, q: false, e: false, r: false, f: false,
    o: false, p: false,
    n1: false, n2: false, n3: false, n4: false,
    space: false, shift: false,
};
const toggleKeys = { r: false, n1: false, n2: false }; // toggles, not hold

// Map browser key names to our key state names
function mapKey(e) {
    const k = e.key.toLowerCase();
    if (k === '1') return 'n1';
    if (k === '2') return 'n2';
    if (k === '3') return 'n3';
    if (k === '4') return 'n4';
    if (k === ' ') return 'space';
    if (k === 'shift' || e.shiftKey && e.key === 'Shift') return 'shift';
    return k;
}

// ---- Gamepad input ----
//
// We read raw Gamepad API state each frame and remap it into the W3C
// "standard" layout (A=0 … GUIDE=16, LSx=0, LSy=1, RSx=2, RSy=3) before
// sending it to the server. Chrome hands most controllers back with
// `mapping: "standard"` already; for the ones that don't (off-brand Xbox
// clones, bare HID pads, etc.), add an entry to CONTROLLER_MAPPINGS keyed by
// a substring of the raw `gamepad.id` string. Any field you omit falls
// through to the standard index, so you only have to describe what's weird
// about your controller.

// Standard-layout button names in the order they appear in the wire protocol.
const STD_BUTTONS = [
    'a', 'b', 'x', 'y',
    'leftBumper', 'rightBumper',
    'leftTrigger', 'rightTrigger',
    'back', 'start',
    'leftStickButton', 'rightStickButton',
    'dpadUp', 'dpadDown', 'dpadLeft', 'dpadRight',
    'guide',
];

// Mapping shape for one controller (all fields optional):
//   buttons:        { [stdIdx]: rawButtonIdx }
//   axes:           { [stdAxisIdx]: rawAxisIdx }
//   invertAxes:     [ stdAxisIdx, ... ]
//   buttonFromAxis: { [stdIdx]: [rawAxisIdx, threshold] }
//       — derive a button press from an axis (e.g. hat-switch dpad)
//   axisFromButtons: { [stdAxisIdx]: [negRawBtnIdx, posRawBtnIdx] }
//       — derive an analog axis from two buttons
//
// Keys of CONTROLLER_MAPPINGS are case-insensitive substrings matched
// against the raw Gamepad.id. First match wins.
const CONTROLLER_MAPPINGS = {
    "BDA Xbox ONE Enhanced Controller": {
        buttons: { 0:0, 1:1, 2:2, 3:3, 4:4, 5:5, 8:6, 9:7, 10:9, 11:10, 16:8 },
        axes: { 0:0, 1:1, 2:3, 3:4, },
        buttonFromAxis: { 6: [2,0.1], 7: [5,0.1], 12: [7, -0.5], 13: [7, 0.5], 14: [6, -0.5], 15: [6, 0.5], }
    }
    // Example (edit for your off-brand Xbox clone):
    // 'usb gamepad': {
    //     buttons: { 0: 2, 1: 1, 2: 3, 3: 0 },
    //     axes:    { 2: 3, 3: 4 },
    //     buttonFromAxis: { 12: [9, -0.5], 13: [9, 0.5] },
    // },
};

const loggedControllers = new Set();

function lookupMapping(id) {
    if (!id) return null;
    const lower = id.toLowerCase();
    for (const key of Object.keys(CONTROLLER_MAPPINGS)) {
        if (lower.includes(key.toLowerCase())) return CONTROLLER_MAPPINGS[key];
    }
    return null;
}

function readGamepad(slot) {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    if (!pads) return { connected: false };
    // Pick the Nth connected pad, sorted by hardware index.
    const connected = [];
    for (let i = 0; i < pads.length; i++) if (pads[i]) connected.push(pads[i]);
    connected.sort((a, b) => a.index - b.index);
    const p = connected[slot];
    if (!p) return { connected: false };

    if (!loggedControllers.has(p.index)) {
        loggedControllers.add(p.index);
        const map = lookupMapping(p.id);
        console.log(
            `Gamepad ${p.index} connected: "${p.id}" ` +
            `(mapping="${p.mapping || ''}", ${p.buttons.length} buttons, ${p.axes.length} axes` +
            `, ${map ? 'custom remap' : 'standard passthrough'})`
        );
    }

    const mapping = lookupMapping(p.id);
    const buttonMap = (mapping && mapping.buttons) || {};
    const axisMap = (mapping && mapping.axes) || {};
    const invert = new Set((mapping && mapping.invertAxes) || []);
    const buttonFromAxis = (mapping && mapping.buttonFromAxis) || {};
    const axisFromButtons = (mapping && mapping.axisFromButtons) || {};

    const rawBtn = (i) => p.buttons && p.buttons[i];
    const rawAxis = (i) => (p.axes && typeof p.axes[i] === 'number') ? p.axes[i] : 0;

    const readButton = (stdIdx) => {
        const fromAxis = buttonFromAxis[stdIdx];
        if (fromAxis) {
            const [axisIdx, threshold] = fromAxis;
            const v = rawAxis(axisIdx);
            return threshold >= 0 ? v >= threshold : v <= threshold;
        }
        const rawIdx = (stdIdx in buttonMap) ? buttonMap[stdIdx] : stdIdx;
        const b = rawBtn(rawIdx);
        return !!(b && b.pressed);
    };

    const readButtonValue = (stdIdx) => {
        const fromAxis = buttonFromAxis[stdIdx];
        if (fromAxis) {
            const [axisIdx, threshold] = fromAxis;
            const v = rawAxis(axisIdx);
            // Normalize to [0, 1] using the threshold direction.
            const sign = threshold >= 0 ? 1 : -1;
            return Math.max(0, Math.min(1, sign * v));
        }
        const rawIdx = (stdIdx in buttonMap) ? buttonMap[stdIdx] : stdIdx;
        const b = rawBtn(rawIdx);
        return (b && typeof b.value === 'number') ? b.value : (b && b.pressed ? 1 : 0);
    };

    const readAxis = (stdAxisIdx) => {
        const fromBtns = axisFromButtons[stdAxisIdx];
        if (fromBtns) {
            const [negIdx, posIdx] = fromBtns;
            const neg = rawBtn(negIdx);
            const pos = rawBtn(posIdx);
            const n = (neg && typeof neg.value === 'number') ? neg.value : (neg && neg.pressed ? 1 : 0);
            const v = (pos && typeof pos.value === 'number') ? pos.value : (pos && pos.pressed ? 1 : 0);
            return v - n;
        }
        const rawIdx = (stdAxisIdx in axisMap) ? axisMap[stdAxisIdx] : stdAxisIdx;
        const raw = rawAxis(rawIdx);
        return invert.has(stdAxisIdx) ? -raw : raw;
    };

    // Standard axis order: 0 LSx, 1 LSy, 2 RSx, 3 RSy.
    // Standard button indices referenced by name — see STD_BUTTONS.
    const btnIdx = (name) => STD_BUTTONS.indexOf(name);
    return {
        connected: true,
        leftStickX:  readAxis(0),
        leftStickY:  readAxis(1),
        rightStickX: readAxis(2),
        rightStickY: readAxis(3),
        leftTrigger:  readButtonValue(btnIdx('leftTrigger')),
        rightTrigger: readButtonValue(btnIdx('rightTrigger')),
        a:              readButton(btnIdx('a')),
        b:              readButton(btnIdx('b')),
        x:              readButton(btnIdx('x')),
        y:              readButton(btnIdx('y')),
        leftBumper:     readButton(btnIdx('leftBumper')),
        rightBumper:    readButton(btnIdx('rightBumper')),
        back:           readButton(btnIdx('back')),
        start:          readButton(btnIdx('start')),
        leftStickButton:  readButton(btnIdx('leftStickButton')),
        rightStickButton: readButton(btnIdx('rightStickButton')),
        dpadUp:         readButton(btnIdx('dpadUp')),
        dpadDown:       readButton(btnIdx('dpadDown')),
        dpadLeft:       readButton(btnIdx('dpadLeft')),
        dpadRight:      readButton(btnIdx('dpadRight')),
        guide:          readButton(btnIdx('guide')),
    };
}

function sendInput() {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify({
        keys,
        gamepad1: readGamepad(0),
        gamepad2: readGamepad(1),
    }));
}

// ---- Scenario selector + auto-chase toggle (ball tracker UI) ----
const scenarioSelect = document.getElementById('scenario-select');
if (scenarioSelect) {
    scenarioSelect.addEventListener('change', () => {
        if (!ws || ws.readyState !== WebSocket.OPEN) return;
        ws.send(JSON.stringify({ type: 'scenario', name: scenarioSelect.value }));
    });
}

const chaseBtn = document.getElementById('chase-btn');
let chaseOn = false;
if (chaseBtn) {
    chaseBtn.addEventListener('click', () => {
        chaseOn = !chaseOn;
        chaseBtn.textContent = 'AUTO-CHASE: ' + (chaseOn ? 'ON' : 'OFF');
        chaseBtn.classList.toggle('active', chaseOn);
        // Synthesize a gamepad1.a rising edge next tick via the `keys.space` path
        // if we wanted — but simpler: fire a one-shot WS message and let the test
        // treat it as a toggle.
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'chase', on: chaseOn }));
        }
    });
}

document.addEventListener('keydown', (e) => {
    const key = mapKey(e);
    if (key in toggleKeys) {
        if (!e.repeat) {
            toggleKeys[key] = !toggleKeys[key];
            keys[key] = toggleKeys[key];
            sendInput();
        }
    } else if (key in keys && !keys[key]) {
        keys[key] = true;
        sendInput();
    }
});

document.addEventListener('keyup', (e) => {
    const key = mapKey(e);
    if (key in toggleKeys) return;
    if (key in keys) {
        keys[key] = false;
        sendInput();
    }
});

// Poll input + push state at ~60 Hz. gamecontroller.js runs its own rAF loop
// for button edge detection; we still need this interval to push stick and
// trigger values (which are continuous, not event-driven) to the server.
setInterval(sendInput, 16);

connect();
