import { updateRobot, updateBalls, updateGoals } from './scene.js';

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
    timeDisplay.textContent = state.t.toFixed(3) + 's';
    posDisplay.innerHTML = `X: ${state.robot.x.toFixed(3)}<br>Y: ${state.robot.y.toFixed(3)}`;
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

function sendKeys() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(keys));
    }
}

document.addEventListener('keydown', (e) => {
    const key = mapKey(e);
    if (key in toggleKeys) {
        if (!e.repeat) {
            toggleKeys[key] = !toggleKeys[key];
            keys[key] = toggleKeys[key];
            sendKeys();
        }
    } else if (key in keys && !keys[key]) {
        keys[key] = true;
        sendKeys();
    }
});

document.addEventListener('keyup', (e) => {
    const key = mapKey(e);
    if (key in toggleKeys) return;
    if (key in keys) {
        keys[key] = false;
        sendKeys();
    }
});

connect();
