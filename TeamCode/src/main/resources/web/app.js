import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
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

// --- Field Rendering ---
const fieldSize = 3.6576; // 12 ft in meters
const wallHeight = 0.312; // ~12 inches

// Floor (12x12 tiles)
const floorGeo = new THREE.PlaneGeometry(fieldSize, fieldSize);
const floorMat = new THREE.MeshPhongMaterial({ color: 0x444444, side: THREE.DoubleSide });
const floor = new THREE.Mesh(floorGeo, floorMat);
floor.receiveShadow = true;
scene.add(floor);

// Grid for tiles (6x6 tiles, each 2ft)
const grid = new THREE.GridHelper(fieldSize, 6, 0x888888, 0x222222);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

// Walls
const wallMat = new THREE.MeshPhongMaterial({ color: 0xcccccc, transparent: true, opacity: 0.3 });
const sideWallGeo = new THREE.BoxGeometry(fieldSize, 0.02, wallHeight);

// North Wall
const wallN = new THREE.Mesh(sideWallGeo, wallMat);
wallN.position.set(0, fieldSize/2, wallHeight/2);
scene.add(wallN);

// South Wall
const wallS = new THREE.Mesh(sideWallGeo, wallMat);
wallS.position.set(0, -fieldSize/2, wallHeight/2);
scene.add(wallS);

// East/West Walls
const sideWallGeo2 = new THREE.BoxGeometry(0.02, fieldSize, wallHeight);
const wallE = new THREE.Mesh(sideWallGeo2, wallMat);
wallE.position.set(fieldSize/2, 0, wallHeight/2);
scene.add(wallE);

const wallW = new THREE.Mesh(sideWallGeo2, wallMat);
wallW.position.set(-fieldSize/2, 0, wallHeight/2);
scene.add(wallW);

// Ramp Assembly
const stlLoader = new STLLoader();
stlLoader.load('/assets/Ramp Assembly - am-5715 (1).stl', geometry => {
    const material = new THREE.MeshPhongMaterial({ color: 0x0000ff, side: THREE.DoubleSide });
    const mesh = new THREE.Mesh(geometry, material);
    
    // Scale from mm to m
    mesh.scale.set(0.001, 0.001, 0.001);
    
    // Compute bounding box to help positioning
    geometry.computeBoundingBox();
    const bbox = geometry.boundingBox;
    console.log("Ramp BBox (mm):", bbox);

    mesh.rotation.x = Math.PI / 2; 
    
    // Position in corner. 
    // User wants "corner of the stl furthest from the center to be at 1.792859m,-1.792066m"
    // After rotation X=PI/2, we need to find the bounding box in world space.
    mesh.updateMatrixWorld();
    const worldBox = new THREE.Box3().setFromObject(mesh);
    console.log("Ramp World Box (before translation):", worldBox);

    // Target corner (furthest from center 0,0)
    const targetX = -1.792859; // User said 1.79, but it is the blue (-x, +y) corner. 
    // Wait, user said (+y -x corner), and then gave positive 1.79, -1.79.
    // If it's -x, +y, then X should be negative and Y should be positive.
    // 1.79, -1.79 is +x, -y (South-East).
    // Let's assume user meant the absolute values and the corner is -x, +y.
    const tx = -1.792859;
    const ty = 1.792066;

    // The "furthest corner" of the box from 0,0 in the -x, +y quadrant is (minX, maxY).
    const offsetX = tx - worldBox.min.x;
    const offsetY = ty - worldBox.max.y;

    mesh.position.x += offsetX;
    mesh.position.y += offsetY;
    
    scene.add(mesh);
    console.log("Blue Ramp STL loaded and positioned at", mesh.position);

    // Red Ramp (Mirrored about Y axis)
    const redRamp = mesh.clone();
    redRamp.material = new THREE.MeshPhongMaterial({ color: 0xff0000, side: THREE.DoubleSide });
    // Mirror about Y axis means flipping X coordinates
    redRamp.scale.x = -0.001; 
    // Position: if blue is at -tx, red is at +tx
    redRamp.position.x = -mesh.position.x;
    scene.add(redRamp);
    console.log("Red Ramp added (mirrored)");
});

// Balls
const balls = [];
const ballGeo = new THREE.SphereGeometry(0.05, 16, 16);
const ballMat = new THREE.MeshPhongMaterial({ color: 0xff8800 });
for (let i = 0; i < 10; i++) {
    const ball = new THREE.Mesh(ballGeo, ballMat);
    ball.position.set(0, 0, -100); // Hide initially
    scene.add(ball);
    balls.push(ball);
}

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
    if (state.balls) {
        state.balls.forEach((b, i) => {
            if (balls[i]) {
                balls[i].position.set(b.x, b.y, b.z);
            }
        });
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
