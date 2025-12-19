import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';

console.log("App starting...");

const statusDisplay = document.getElementById('status');
const errorDisplay = document.getElementById('error-msg');
const timeDisplay = document.getElementById('time-display');

// --- Networking (Move to top to connect ASAP) ---
const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const wsUrl = `${protocol}//${window.location.host}/sim`;
console.log("Connecting to WebSocket:", wsUrl);
let ws = new WebSocket(wsUrl);

ws.onopen = () => {
    console.log("WebSocket opened successfully");
    statusDisplay.textContent = "Connected";
    statusDisplay.style.color = "green";
    errorDisplay.textContent = "";
};

ws.onclose = (event) => {
    console.log("WebSocket closed", event);
    statusDisplay.textContent = "Disconnected";
    statusDisplay.style.color = "red";
    errorDisplay.textContent = `WS Closed: ${event.code} ${event.reason}`;
};

ws.onerror = (error) => {
    console.error("WebSocket error", error);
    errorDisplay.textContent = "WS Error. Check Console.";
};

ws.onmessage = (event) => {
    const state = JSON.parse(event.data);
    history.push(state);
    if (history.length > 20000) history.shift(); 
    if (isLive) {
        updateVisuals(state);
        updateCharts(state);
    }
};

// --- Scene Setup ---
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x333333);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(2, 2, 2);
camera.up.set(0, 0, 1); // Z is up

const renderer = new THREE.WebGLRenderer({ antialias: true });
const container = document.getElementById('canvas-container');
renderer.setSize(container.clientWidth, container.clientHeight);
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// Lights
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(5, 5, 10);
scene.add(dirLight);

// Grid
const gridHelper = new THREE.GridHelper(10, 10);
gridHelper.rotation.x = Math.PI / 2;
scene.add(gridHelper);

// Axis
const axesHelper = new THREE.AxesHelper(1);
scene.add(axesHelper);

// Robot
let robot = null;
const loader = new URDFLoader();
console.log("Loading URDF from /robot.urdf...");
loader.load('/robot.urdf', result => {
    console.log("URDF loaded successfully", result);
    robot = result;
    scene.add(robot);
}, progress => {
    // console.log("URDF loading progress", progress);
}, error => {
    console.error("URDF loading error", error);
    errorDisplay.textContent = "URDF Error: " + error.message;
});

// Resize handler
window.addEventListener('resize', () => {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
});
new ResizeObserver(() => {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
}).observe(container);


// --- Charts ---
function createChart(id, label, datasets) {
    const ctx = document.getElementById(id).getContext('2d');
    return new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: datasets.map((ds, i) => ({
                label: ds.label,
                borderColor: ds.color,
                data: [],
                borderWidth: 1,
                pointRadius: 0
            }))
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            interaction: { mode: 'index', intersect: false },
            scales: { x: { display: false } }
        }
    });
}

const chartWheels = createChart('chart-wheels', 'Wheel Vels', [
    { label: 'FL', color: 'red' },
    { label: 'FR', color: 'green' },
    { label: 'BL', color: 'blue' },
    { label: 'BR', color: 'yellow' }
]);

const chartMechs = createChart('chart-mechanisms', 'Mechs', [
    { label: 'Flywheel', color: 'cyan' },
    { label: 'Turret', color: 'magenta' }
]);

const chartPos = createChart('chart-pos', 'Position', [
    { label: 'X', color: 'red' },
    { label: 'Y', color: 'green' },
    { label: 'Heading', color: 'blue' }
]);

// --- State Management ---
const history = []; // array of state objects
let isLive = true;
let playbackIndex = 0;
let playbackInterval = null;

const btnReplay = document.getElementById('btn-replay');
const btnLive = document.getElementById('btn-live');

function updateVisuals(state) {
    if (!robot) return;
    if (state.base) {
        robot.position.set(state.base.x, state.base.y, state.base.z || 0);
        const euler = new THREE.Euler(state.base.roll || 0, state.base.pitch || 0, state.base.yaw || 0, 'ZYX');
        robot.setRotationFromEuler(euler);
    }
    if (state.joints) {
        for (const [name, val] of Object.entries(state.joints)) {
            if (robot.joints[name]) {
                robot.joints[name].setJointValue(val);
            }
        }
    }
    timeDisplay.textContent = `T: ${state.t.toFixed(2)}s`;
}

function updateCharts(state) {
    const limit = 500;
    if (!state.telemetry) return;
    function pushData(chart, vals) {
        if (chart.data.labels.length > limit) {
            chart.data.labels.shift();
            chart.data.datasets.forEach(ds => ds.data.shift());
        }
        chart.data.labels.push(state.t);
        chart.data.datasets.forEach((ds, i) => {
            if (vals[i] !== undefined) ds.data.push(vals[i]);
        });
        chart.update();
    }
    pushData(chartWheels, [state.telemetry.fl, state.telemetry.fr, state.telemetry.bl, state.telemetry.br]);
    pushData(chartMechs, [state.telemetry.flywheel, state.telemetry.turret]);
    pushData(chartPos, [state.base.x, state.base.y, state.base.yaw]);
}

btnReplay.addEventListener('click', () => {
    isLive = false;
    btnLive.disabled = false;
    btnReplay.disabled = true;
    startReplay();
});

btnLive.addEventListener('click', () => {
    isLive = true;
    btnLive.disabled = true;
    btnReplay.disabled = false;
    stopReplay();
    if (history.length > 0) {
        updateVisuals(history[history.length - 1]);
    }
});

function startReplay() {
    playbackIndex = 0;
    if (playbackInterval) clearInterval(playbackInterval);
    playbackInterval = setInterval(() => {
        if (playbackIndex >= history.length) playbackIndex = 0;
        updateVisuals(history[playbackIndex]);
        playbackIndex++;
    }, 20);
}

function stopReplay() {
    if (playbackInterval) clearInterval(playbackInterval);
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();