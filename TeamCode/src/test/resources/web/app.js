import { updateRobot, updateBalls } from './scene.js';

const frames = [];
let isLive = true;
let isPaused = false;

const scrubber = document.getElementById('scrubber');
const timeDisplay = document.getElementById('time-display');
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
    updateRobot(state.robot.x, state.robot.y, state.robot.theta, state.robot.turretAngle);
    updateBalls(state.balls || []);
    timeDisplay.textContent = state.t.toFixed(3) + 's';
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

// WASD keyboard drive input
const keys = { w: false, a: false, s: false, d: false, q: false, e: false };

function sendKeys() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(keys));
    }
}

document.addEventListener('keydown', (e) => {
    const key = e.key.toLowerCase();
    if (key in keys && !keys[key]) {
        keys[key] = true;
        sendKeys();
    }
});

document.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    if (key in keys) {
        keys[key] = false;
        sendKeys();
    }
});

connect();
