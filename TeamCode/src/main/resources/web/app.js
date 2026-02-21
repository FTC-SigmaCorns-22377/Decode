import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import URDFLoader from 'urdf-loader';

console.log("App starting...");

// --- Elements ---
const statusDisplay = document.getElementById('status');
const errorDisplay = document.getElementById('error-msg');
const timeDisplay = document.getElementById('time-display');
const gamepadStatusDisplay = document.getElementById('gamepad-status');
const btnReplay = document.getElementById('btn-replay');
const btnLive = document.getElementById('btn-live');
const btnPlayPause = document.getElementById('btn-play-pause');
const scrubber = document.getElementById('scrubber');
const toggleRobotMesh = document.getElementById('toggle-robot-mesh');
const toggleRobotPoint = document.getElementById('toggle-robot-point');
const togglePath = document.getElementById('toggle-path');
const toggleMpcTarget = document.getElementById('toggle-mpc-target');
const toggleForces = document.getElementById('toggle-forces');
const toggleBalls = document.getElementById('toggle-balls');
const toggleErrorChart = document.getElementById('toggle-error-chart');
const errorChartContainer = document.getElementById('error-chart-container');
const errorChartTitle = document.getElementById('error-chart-title');

// --- State ---
let history = [];
let isLive = true;
let isPaused = false;
let playbackIndex = 0;
let playbackInterval = null;
let scrubberDragging = false;
let robot = null;
let messageCount = 0; // For throttling chart updates
const balls = [];
const wheelForceArrows = new Map();
const wheelForceScale = 0.0025;
const wheelForceMaxLen = 0.6;
const wheelForceMinLen = 0.02;
const wheelForceConfig = [
    { link: 'fl_wheel', color: 0xff0000 },
    { link: 'bl_wheel', color: 0x0000ff },
    { link: 'br_wheel', color: 0xffff00 },
    { link: 'fr_wheel', color: 0x00ff00 }
];
const wheelForceTmp = new THREE.Vector3();
let pathLine = null;
let pathSignature = '';
let mpcTargetLine = null;
let mpcTargetSignature = '';
const robotPoint = new THREE.Mesh(
    new THREE.SphereGeometry(0.04, 12, 12),
    new THREE.MeshPhongMaterial({ color: 0xffaa00 })
);
const robotHeading = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.2, 0xffaa00);
robotPoint.visible = false;
robotHeading.visible = false;

// --- Gamepad State ---
let gamepadConnected = false;
const GAMEPAD_DEADZONE = 0.1;

// --- Controller Type Detection & Mapping ---
// Supports Xbox (all variants), PlayStation (DS4, DualSense), and generic controllers.
// When the browser reports mapping === "standard", all controllers use the same W3C layout.
// For non-standard mappings (common with PS controllers on Linux/Firefox), we detect the
// controller type and apply the correct axis/button layout.
const CONTROLLER_TYPES = { XBOX: 'Xbox', PLAYSTATION: 'PlayStation', GENERIC: 'Generic' };

function detectControllerType(gamepadId) {
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
const MAPPING_STANDARD = {
    name: 'Standard',
    axes: { LX: 0, LY: 1, RX: 2, RY: 3 },
    buttons: { A: 0, B: 1, X: 2, Y: 3, LB: 4, RB: 5, LT: 6, RT: 7,
               BACK: 8, START: 9, LS: 10, RS: 11,
               DPAD_UP: 12, DPAD_DOWN: 13, DPAD_LEFT: 14, DPAD_RIGHT: 15 },
    triggers: 'buttons', // triggers read via buttons[].value (0..1)
    dpad: 'buttons'      // dpad read via buttons[].pressed
};

// PlayStation non-standard with DPad on axes 6,7
// Common on Linux (Firefox/Chrome without standard mapping support for DS4/DualSense).
// Axes: 0=LX, 1=LY, 2=LT(-1..1), 3=RX, 4=RY, 5=RT(-1..1), 6=DpadX, 7=DpadY
const MAPPING_PS_AXES_DPAD = {
    name: 'PlayStation (non-standard, dpad-axes)',
    axes: { LX: 0, LY: 1, RX: 3, RY: 4, LT: 2, RT: 5, DPAD_X: 6, DPAD_Y: 7 },
    buttons: { A: 0, B: 1, X: 2, Y: 3, LB: 4, RB: 5, LT: 6, RT: 7,
               BACK: 8, START: 9, LS: 10, RS: 11 },
    triggers: 'axes', // triggers on axes, range -1..1, normalized to 0..1
    dpad: 'axes'      // dpad on axes 6 (X) and 7 (Y)
};

// PlayStation non-standard with DPad on buttons 14-17
// Some Linux HID drivers report DPad as buttons instead of axes.
// Axes: 0=LX, 1=LY, 2=LT(-1..1), 3=RX, 4=RY, 5=RT(-1..1)
const MAPPING_PS_BTN_DPAD = {
    name: 'PlayStation (non-standard, dpad-buttons)',
    axes: { LX: 0, LY: 1, RX: 3, RY: 4, LT: 2, RT: 5 },
    buttons: { A: 0, B: 1, X: 2, Y: 3, LB: 4, RB: 5, LT: 6, RT: 7,
               BACK: 8, START: 9, LS: 10, RS: 11,
               DPAD_UP: 14, DPAD_DOWN: 15, DPAD_LEFT: 16, DPAD_RIGHT: 17 },
    triggers: 'axes',
    dpad: 'buttons'
};

let activeMapping = MAPPING_STANDARD;
let detectedControllerType = CONTROLLER_TYPES.GENERIC;

function resolveMapping(gamepad) {
    if (gamepad.mapping === 'standard') return MAPPING_STANDARD;
    const type = detectControllerType(gamepad.id);
    if (type === CONTROLLER_TYPES.PLAYSTATION) {
        // Heuristic: >=8 axes means DPad is on axes 6,7; otherwise DPad is on buttons
        return gamepad.axes.length >= 8 ? MAPPING_PS_AXES_DPAD : MAPPING_PS_BTN_DPAD;
    }
    // Non-standard Xbox or generic: standard layout is the best guess
    return MAPPING_STANDARD;
}

function readGamepadState(gp) {
    const m = activeMapping;
    const axis = (i) => gp.axes[i] || 0;
    const btn = (i) => gp.buttons[i] ? gp.buttons[i].pressed : false;
    const btnVal = (i) => gp.buttons[i] ? gp.buttons[i].value : 0;

    // Triggers
    let lt, rt;
    if (m.triggers === 'axes') {
        // PS non-standard: triggers on axes with range -1..1, normalize to 0..1
        lt = (axis(m.axes.LT) + 1) / 2;
        rt = (axis(m.axes.RT) + 1) / 2;
    } else {
        lt = btnVal(m.buttons.LT);
        rt = btnVal(m.buttons.RT);
    }

    // D-pad
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

// Current gamepad state object
let currentGamepadState = {
    left_stick_x: 0, left_stick_y: 0,
    right_stick_x: 0, right_stick_y: 0,
    left_trigger: 0, right_trigger: 0,
    a: false, b: false, x: false, y: false,
    left_bumper: false, right_bumper: false,
    back: false, start: false,
    left_stick_button: false, right_stick_button: false,
    dpad_up: false, dpad_down: false, dpad_left: false, dpad_right: false
};

// --- Scene Setup ---
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x333333);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(2, 2, 2);
camera.up.set(0, 0, 1); 

const renderer = new THREE.WebGLRenderer({ antialias: true });
const container = document.getElementById('canvas-container');
renderer.setSize(container.clientWidth, container.clientHeight);
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// Lights
scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(5, 5, 10);
scene.add(dirLight);

// Grid & Axis
const gridHelper = new THREE.GridHelper(3.6576, 6);
gridHelper.rotation.x = Math.PI / 2;
scene.add(gridHelper);
scene.add(new THREE.AxesHelper(1));

// Field
const fieldSize = 3.6576; 
const wallHeight = 0.312;
const floor = new THREE.Mesh(new THREE.PlaneGeometry(fieldSize, fieldSize), new THREE.MeshPhongMaterial({ color: 0x444444, side: THREE.DoubleSide }));
scene.add(floor);

const wallMat = new THREE.MeshPhongMaterial({ color: 0xcccccc, transparent: true, opacity: 0.3 });
const sideWallGeo = new THREE.BoxGeometry(fieldSize, 0.02, wallHeight);
[[0, fieldSize/2], [0, -fieldSize/2]].forEach(pos => {
    const w = new THREE.Mesh(sideWallGeo, wallMat);
    w.position.set(pos[0], pos[1], wallHeight/2);
    scene.add(w);
});
const sideWallGeo2 = new THREE.BoxGeometry(0.02, fieldSize, wallHeight);
[[fieldSize/2, 0], [-fieldSize/2, 0]].forEach(pos => {
    const w = new THREE.Mesh(sideWallGeo2, wallMat);
    w.position.set(pos[0], pos[1], wallHeight/2);
    scene.add(w);
});

// Ramp
const stlLoader = new STLLoader();
stlLoader.load('/assets/Ramp Assembly - am-5715 (1).stl', geometry => {
    const mesh = new THREE.Mesh(geometry, new THREE.MeshPhongMaterial({ color: 0x0000ff, side: THREE.DoubleSide }));
    mesh.scale.set(0.001, 0.001, 0.001);
    mesh.rotation.x = Math.PI / 2; 
    geometry.computeBoundingBox();
    mesh.updateMatrixWorld();
    const worldBox = new THREE.Box3().setFromObject(mesh);
    const offsetX = -1.643476;
    const offsetY = -0.013388;
    mesh.position.x += offsetX;
    mesh.position.y += offsetY;
    scene.add(mesh);
    const redRamp = mesh.clone();
    redRamp.material = new THREE.MeshPhongMaterial({ color: 0xff0000, side: THREE.DoubleSide });
    redRamp.scale.x = -0.001; 
    redRamp.position.x = -mesh.position.x;
    scene.add(redRamp);
});

// Balls
const ballGeo = new THREE.SphereGeometry(0.0635, 12, 12);
const ballMaterials = {
    green: new THREE.MeshPhongMaterial({ color: 0x00ff00 }),
    purple: new THREE.MeshPhongMaterial({ color: 0x9932cc }),
    orange: new THREE.MeshPhongMaterial({ color: 0xff8800 })
};

function ensureBallCount(count) {
    while (balls.length < count) {
        const b = new THREE.Mesh(ballGeo, ballMaterials.orange);
        b.position.set(0, 0, -100);
        scene.add(b);
        balls.push(b);
    }
    // Hide excess balls
    for (let i = count; i < balls.length; i++) {
        balls[i].position.set(0, 0, -100);
    }
}

scene.add(robotPoint);
scene.add(robotHeading);

// Robot
const loader = new URDFLoader();
loader.load('/robot.urdf', result => {
    robot = result;
    scene.add(robot);
    wheelForceConfig.forEach(cfg => {
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), wheelForceMinLen, cfg.color);
        arrow.visible = false;
        scene.add(arrow);
        wheelForceArrows.set(cfg.link, arrow);
    });
});

function setPath(points) {
    if (!points || points.length === 0) {
        if (pathLine) {
            scene.remove(pathLine);
            pathLine.geometry.dispose();
        }
        pathLine = null;
        pathSignature = '';
        return;
    }
    const head = points[0];
    const tail = points[points.length - 1];
    const signature = `${points.length}:${head.x.toFixed(3)},${head.y.toFixed(3)}:${tail.x.toFixed(3)},${tail.y.toFixed(3)}`;
    if (signature === pathSignature) return;
    pathSignature = signature;
    if (pathLine) {
        scene.remove(pathLine);
        pathLine.geometry.dispose();
    }
    const verts = new Float32Array(points.length * 3);
    points.forEach((p, i) => {
        verts[i * 3] = p.x;
        verts[i * 3 + 1] = p.y;
        verts[i * 3 + 2] = 0.01;
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    const material = new THREE.LineBasicMaterial({ color: 0xffffff });
    pathLine = new THREE.Line(geometry, material);
    pathLine.visible = togglePath.checked;
    scene.add(pathLine);
}

function setMpcTarget(points) {
    if (!points || points.length === 0) {
        if (mpcTargetLine) {
            scene.remove(mpcTargetLine);
            mpcTargetLine.geometry.dispose();
        }
        mpcTargetLine = null;
        mpcTargetSignature = '';
        return;
    }
    const head = points[0];
    const tail = points[points.length - 1];
    const signature = `${points.length}:${head.x.toFixed(3)},${head.y.toFixed(3)}:${tail.x.toFixed(3)},${tail.y.toFixed(3)}`;
    if (signature === mpcTargetSignature) return;
    mpcTargetSignature = signature;
    if (mpcTargetLine) {
        scene.remove(mpcTargetLine);
        mpcTargetLine.geometry.dispose();
    }
    const verts = new Float32Array(points.length * 3);
    points.forEach((p, i) => {
        verts[i * 3] = p.x;
        verts[i * 3 + 1] = p.y;
        verts[i * 3 + 2] = 0.03;
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    const material = new THREE.LineBasicMaterial({ color: 0x00ffff });
    mpcTargetLine = new THREE.Line(geometry, material);
    mpcTargetLine.visible = toggleMpcTarget.checked;
    scene.add(mpcTargetLine);
}

// --- Gamepad Functions (using gamecontroller.js) ---
function updateGamepadStatusDisplay() {
    if (gamepadConnected) {
        gamepadStatusDisplay.textContent = `🎮 ${detectedControllerType} (${activeMapping.name})`;
        gamepadStatusDisplay.style.color = "green";
    } else {
        gamepadStatusDisplay.textContent = "🎮 No Gamepad";
        gamepadStatusDisplay.style.color = "gray";
    }
}

function applyDeadzone(value, deadzone = GAMEPAD_DEADZONE) {
    return Math.abs(value) < deadzone ? 0.0 : value;
}

function sendGamepadState(ws, gamepadId, state) {
    if (ws.readyState === WebSocket.OPEN && state) {
        const message = {
            type: "gamepad",
            gamepadId: gamepadId,
            timestamp: Date.now(),
            input: state
        };
        ws.send(JSON.stringify(message));
    }
}

function setupGamepadEvents(gamepadIndex, ws) {
    console.log(`Setting up gamepad polling for index ${gamepadIndex}`);
    let logCounter = 0;

    // Poll the raw gamepad state using navigator.getGamepads()
    const pollState = () => {
        const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
        const gp = gamepads[gamepadIndex];

        if (!gp) {
            if (logCounter++ % 100 === 0) console.log(`No gamepad at index ${gamepadIndex}`);
            return;
        }

        // Re-resolve mapping on each poll in case the browser updates the mapping property
        // (e.g. when switching between wired/wireless on some controllers)
        activeMapping = resolveMapping(gp);
        currentGamepadState = readGamepadState(gp);

        // Log occasionally to verify polling works
        if (logCounter++ % 250 === 0) {
            console.log(`Gamepad [${detectedControllerType}] state: LX=${currentGamepadState.left_stick_x.toFixed(2)}, LY=${currentGamepadState.left_stick_y.toFixed(2)}, A=${currentGamepadState.a}`);
        }

        sendGamepadState(ws, 1, currentGamepadState);
    };

    // Poll at 50Hz
    return setInterval(pollState, 20);
}

let gamepadPollInterval = null;

// --- Visual Updates ---
// Reusable objects to avoid per-frame allocation
const _euler = new THREE.Euler();
const _dir = new THREE.Vector3();
const _forceVec = new THREE.Vector3();

function updateVisuals(state) {
    if (!state) return;
    if (robot && state.base) {
        robot.position.set(state.base.x, state.base.y, state.base.z || 0);
        _euler.set(state.base.roll || 0, state.base.pitch || 0, state.base.yaw || 0, 'ZYX');
        robot.setRotationFromEuler(_euler);
    }
    if (state.base) {
        robotPoint.position.set(state.base.x, state.base.y, (state.base.z || 0) + 0.02);
        const yaw = state.base.yaw || 0;
        _dir.set(Math.cos(yaw), Math.sin(yaw), 0);
        robotHeading.position.copy(robotPoint.position);
        robotHeading.setDirection(_dir);
        robotHeading.setLength(0.2, 0.06, 0.03);
    }
    if (robot && state.joints) {
        for (const [name, val] of Object.entries(state.joints)) {
            if (robot.joints[name]) robot.joints[name].setJointValue(val);
        }
    }
    if (state.balls) {
        ensureBallCount(state.balls.length);
        state.balls.forEach((b, i) => {
            if (balls[i]) {
                balls[i].position.set(b.x, b.y, b.z);
                // Set ball color based on state
                const colorName = b.color || 'orange';
                balls[i].material = ballMaterials[colorName] || ballMaterials.orange;
            }
        });
    }
    if (state.wheelForces && robot && toggleForces.checked) {
        wheelForceConfig.forEach((cfg, i) => {
            const force = state.wheelForces[i];
            const arrow = wheelForceArrows.get(cfg.link);
            const link = robot.links ? robot.links[cfg.link] : null;
            if (!force || !arrow || !link) return;
            _forceVec.set(force.x, force.y, force.z);
            const magnitude = _forceVec.length();
            if (magnitude < 1e-6) {
                arrow.visible = false;
                return;
            }
            const length = Math.max(wheelForceMinLen, Math.min(magnitude * wheelForceScale, wheelForceMaxLen));
            _forceVec.normalize();
            link.getWorldPosition(wheelForceTmp);
            arrow.position.copy(wheelForceTmp);
            arrow.setDirection(_forceVec);
            arrow.setLength(length, length * 0.2, length * 0.1);
            arrow.visible = true;
        });
    } else {
        wheelForceArrows.forEach(arrow => { arrow.visible = false; });
    }
    if (robot) robot.visible = toggleRobotMesh.checked;
    robotPoint.visible = toggleRobotPoint.checked;
    robotHeading.visible = toggleRobotPoint.checked;
    balls.forEach(ball => { ball.visible = toggleBalls.checked; });
    setMpcTarget(state.mpcTarget || []);
    timeDisplay.textContent = `T: ${state.t.toFixed(2)}s`;
    if (!scrubberDragging) {
        scrubber.max = history.length - 1;
        scrubber.value = isLive ? history.length - 1 : playbackIndex;
    }
}

// --- Charts ---
function createChart(id, datasets) {
    const ctx = document.getElementById(id).getContext('2d');
    return new Chart(ctx, {
        type: 'line',
        data: { labels: [], datasets: datasets.map(ds => ({ label: ds.label, borderColor: ds.color, data: [], borderWidth: 1, pointRadius: 0 })) },
        options: { responsive: true, maintainAspectRatio: false, animation: false, scales: { x: { display: false } } }
    });
}
const chartWheels = createChart('chart-wheels', [{label:'FL',color:'red'},{label:'FR',color:'green'},{label:'BL',color:'blue'},{label:'BR',color:'yellow'}]);
const chartMechs = createChart('chart-mechanisms', [{label:'Flywheel',color:'cyan'},{label:'Turret',color:'magenta'},{label:'Spindexer Pwr',color:'orange'},{label:'Spindexer Ang',color:'lime'}]);
const chartPos = createChart('chart-pos', [{label:'X',color:'red'},{label:'Y',color:'green'},{label:'Heading',color:'blue'}]);
const chartError = createChart('chart-error', [
    {label:'Err X',color:'red'},
    {label:'Err Y',color:'green'},
    {label:'Err Heading',color:'blue'},
    {label:'Err Vx',color:'orange'},
    {label:'Err Vy',color:'purple'},
    {label:'Err Omega',color:'cyan'}
]);

function syncCharts(currentIndex) {
    if (history.length === 0) return;
    
    const limit = 500;
    const start = Math.max(0, currentIndex - limit);
    const window = history.slice(start, currentIndex + 1);

    const labels = window.map(s => s.t);
    
    const update = (chart, dataExtractors) => {
        chart.data.labels = labels;
        dataExtractors.forEach((extractor, i) => {
            chart.data.datasets[i].data = window.map(extractor);
        });
        chart.update();
    };

    update(chartWheels, [s=>s.telemetry.fl, s=>s.telemetry.fr, s=>s.telemetry.bl, s=>s.telemetry.br]);
    update(chartMechs, [s=>s.telemetry.flywheel, s=>s.telemetry.turret, s=>s.telemetry.spindexerPower, s=>s.telemetry.spindexerAngle]);
    update(chartPos, [s=>s.base.x, s=>s.base.y, s=>s.base.yaw]);
    update(chartError, [
        s=>s.error ? s.error.x : null,
        s=>s.error ? s.error.y : null,
        s=>s.error ? s.error.yaw : null,
        s=>s.error ? s.error.vx : null,
        s=>s.error ? s.error.vy : null,
        s=>s.error ? s.error.omega : null
    ]);
}

// --- Networking ---
const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const ws = new WebSocket(`${protocol}//${window.location.host}/sim`);

ws.onopen = () => { statusDisplay.textContent = "Connected"; statusDisplay.style.color = "green"; };
ws.onclose = (e) => { statusDisplay.textContent = "Disconnected"; statusDisplay.style.color = "red"; };
ws.onmessage = (e) => {
    const state = JSON.parse(e.data);
    if (history.length > 0 && state.t <= history[history.length - 1].t) return;
    history.push(state);
    if (history.length > 30000) history = history.slice(-20000); // Batch trim avoids per-frame O(n) shift
    messageCount++;
    if (isLive) {
        updateVisuals(state);
        // Throttle chart updates to every 5th frame (charts are expensive)
        if (messageCount % 5 === 0) {
            syncCharts(history.length - 1);
        }
    }
};

fetch('/history').then(r => r.json()).then(data => {
    console.log(`History: ${data.length}`);
    if (history.length > 0) {
        const lastT = history[history.length-1].t;
        const old = data.filter(s => s.t < history[0].t);
        history = [...old, ...history];
    } else {
        history = data;
    }
    if (history.length > 0 && isLive) {
        updateVisuals(history[history.length-1]);
        syncCharts(history.length - 1);
    }
}).catch(console.error);

// --- Gamepad Setup using gamecontroller.js ---
// Wait for gameControl to be available (loaded from CDN)
function initGamepad() {
    if (typeof gameControl === 'undefined') {
        console.log("Waiting for gameControl library...");
        setTimeout(initGamepad, 100);
        return;
    }

    console.log("gamecontroller.js loaded, setting up gamepad events");

    gameControl.on('connect', (gamepad) => {
        const rawGp = navigator.getGamepads()[gamepad.id];
        const gpName = rawGp?.id || 'Unknown';
        detectedControllerType = detectControllerType(gpName);
        activeMapping = rawGp ? resolveMapping(rawGp) : MAPPING_STANDARD;
        console.log(`Gamepad connected: index=${gamepad.id}, name=${gpName}, type=${detectedControllerType}, mapping=${activeMapping.name} (browser mapping="${rawGp?.mapping}"), axes=${rawGp?.axes?.length}, buttons=${rawGp?.buttons?.length}`);
        gamepadConnected = true;
        updateGamepadStatusDisplay();

        // Clear any existing poll interval
        if (gamepadPollInterval) {
            clearInterval(gamepadPollInterval);
        }

        // Start polling this gamepad (gamepad.id is the index in navigator.getGamepads())
        gamepadPollInterval = setupGamepadEvents(gamepad.id, ws);
    });

    gameControl.on('disconnect', (gamepad) => {
        console.log(`Gamepad disconnected: ${gamepad.id}`);
        gamepadConnected = false;
        updateGamepadStatusDisplay();

        if (gamepadPollInterval) {
            clearInterval(gamepadPollInterval);
            gamepadPollInterval = null;
        }
    });
}

// Initialize gamepad handling
initGamepad();

fetch('/path').then(r => r.json()).then(setPath).catch(console.error);
setInterval(() => { fetch('/path').then(r => r.json()).then(setPath).catch(() => {}); }, 2000);

// --- Controls ---
const startReplay = () => {
    if (playbackInterval) clearInterval(playbackInterval);
    playbackInterval = setInterval(() => {
        if (playbackIndex >= history.length - 1) { stopReplay(); isPaused = true; btnPlayPause.textContent = "Play"; return; }
        playbackIndex++;
        updateVisuals(history[playbackIndex]);
        syncCharts(playbackIndex);
    }, 20);
};
const stopReplay = () => { if (playbackInterval) clearInterval(playbackInterval); };

btnPlayPause.addEventListener('click', () => {
    if (isLive) { isLive = false; playbackIndex = history.length - 1; stopReplay(); isPaused = true; btnPlayPause.textContent = "Play"; btnLive.disabled = false; btnReplay.disabled = true; }
    else if (isPaused) { startReplay(); isPaused = false; btnPlayPause.textContent = "Pause"; }
    else { stopReplay(); isPaused = true; btnPlayPause.textContent = "Play"; }
});
btnReplay.addEventListener('click', () => { isLive = false; btnLive.disabled = false; btnReplay.disabled = true; playbackIndex = 0; isPaused = false; btnPlayPause.textContent = "Pause"; startReplay(); });
btnLive.addEventListener('click', () => { isLive = true; btnLive.disabled = true; btnReplay.disabled = false; btnPlayPause.textContent = "Pause"; stopReplay(); if (history.length > 0) { updateVisuals(history[history.length-1]); syncCharts(history.length - 1); } });
scrubber.addEventListener('mousedown', () => { scrubberDragging = true; isLive = false; stopReplay(); });
scrubber.addEventListener('mouseup', () => { scrubberDragging = false; if (!isPaused) startReplay(); });
scrubber.addEventListener('input', () => { playbackIndex = parseInt(scrubber.value); updateVisuals(history[playbackIndex]); syncCharts(playbackIndex); });

toggleRobotMesh.addEventListener('change', () => { if (robot) robot.visible = toggleRobotMesh.checked; });
toggleRobotPoint.addEventListener('change', () => { robotPoint.visible = toggleRobotPoint.checked; robotHeading.visible = toggleRobotPoint.checked; });
togglePath.addEventListener('change', () => { if (pathLine) pathLine.visible = togglePath.checked; });
toggleMpcTarget.addEventListener('change', () => { if (mpcTargetLine) mpcTargetLine.visible = toggleMpcTarget.checked; });
toggleForces.addEventListener('change', () => { if (!toggleForces.checked) wheelForceArrows.forEach(arrow => { arrow.visible = false; }); });
toggleBalls.addEventListener('change', () => { balls.forEach(ball => { ball.visible = toggleBalls.checked; }); });
toggleErrorChart.addEventListener('change', () => {
    const show = toggleErrorChart.checked;
    errorChartContainer.style.display = show ? '' : 'none';
    errorChartTitle.style.display = show ? '' : 'none';
});
toggleErrorChart.dispatchEvent(new Event('change'));

window.addEventListener('resize', () => { renderer.setSize(container.clientWidth, container.clientHeight); camera.aspect = container.clientWidth / container.clientHeight; camera.updateProjectionMatrix(); });
new ResizeObserver(() => { renderer.setSize(container.clientWidth, container.clientHeight); camera.aspect = container.clientWidth / container.clientHeight; camera.updateProjectionMatrix(); }).observe(container);

function animate() { requestAnimationFrame(animate); controls.update(); renderer.render(scene, camera); }
animate();
