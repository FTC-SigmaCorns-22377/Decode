import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import URDFLoader from 'urdf-loader';

console.log("App starting...");

// --- Elements ---
const statusDisplay = document.getElementById('status');
const errorDisplay = document.getElementById('error-msg');
const timeDisplay = document.getElementById('time-display');
const btnReplay = document.getElementById('btn-replay');
const btnLive = document.getElementById('btn-live');
const btnPlayPause = document.getElementById('btn-play-pause');
const scrubber = document.getElementById('scrubber');

// --- State ---
let history = []; 
let isLive = true;
let isPaused = false;
let playbackIndex = 0;
let playbackInterval = null;
let scrubberDragging = false;
let robot = null;
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
    const offsetX = -1.792859 - worldBox.min.x;
    const offsetY = 1.792066 - worldBox.max.y;
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
const ballGeo = new THREE.SphereGeometry(0.05, 16, 16);
const ballMat = new THREE.MeshPhongMaterial({ color: 0xff8800 });
for (let i = 0; i < 10; i++) {
    const b = new THREE.Mesh(ballGeo, ballMat);
    b.position.set(0, 0, -100);
    scene.add(b);
    balls.push(b);
}

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

// --- Visual Updates ---
function updateVisuals(state) {
    if (!state) return;
    if (robot && state.base) {
        robot.position.set(state.base.x, state.base.y, state.base.z || 0);
        robot.setRotationFromEuler(new THREE.Euler(state.base.roll || 0, state.base.pitch || 0, state.base.yaw || 0, 'ZYX'));
    }
    if (robot && state.joints) {
        for (const [name, val] of Object.entries(state.joints)) {
            if (robot.joints[name]) robot.joints[name].setJointValue(val);
        }
    }
    if (state.balls) {
        state.balls.forEach((b, i) => { if (balls[i]) balls[i].position.set(b.x, b.y, b.z); });
    }
    if (state.wheelForces && robot) {
        wheelForceConfig.forEach((cfg, i) => {
            const force = state.wheelForces[i];
            const arrow = wheelForceArrows.get(cfg.link);
            const link = robot.links ? robot.links[cfg.link] : null;
            if (!force || !arrow || !link) return;
            const forceVec = new THREE.Vector3(force.x, force.y, force.z);
            const magnitude = forceVec.length();
            if (magnitude < 1e-6) {
                arrow.visible = false;
                return;
            }
            const length = Math.max(wheelForceMinLen, Math.min(magnitude * wheelForceScale, wheelForceMaxLen));
            forceVec.normalize();
            link.getWorldPosition(wheelForceTmp);
            arrow.position.copy(wheelForceTmp);
            arrow.setDirection(forceVec);
            arrow.setLength(length, length * 0.2, length * 0.1);
            arrow.visible = true;
        });
    } else {
        wheelForceArrows.forEach(arrow => { arrow.visible = false; });
    }
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
const chartMechs = createChart('chart-mechanisms', [{label:'Flywheel',color:'cyan'},{label:'Turret',color:'magenta'}]);
const chartPos = createChart('chart-pos', [{label:'X',color:'red'},{label:'Y',color:'green'},{label:'Heading',color:'blue'}]);

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
    update(chartMechs, [s=>s.telemetry.flywheel, s=>s.telemetry.turret]);
    update(chartPos, [s=>s.base.x, s=>s.base.y, s=>s.base.yaw]);
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
    if (history.length > 30000) history.shift();
    if (isLive) { 
        updateVisuals(state); 
        syncCharts(history.length - 1); 
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

window.addEventListener('resize', () => { renderer.setSize(container.clientWidth, container.clientHeight); camera.aspect = container.clientWidth / container.clientHeight; camera.updateProjectionMatrix(); });
new ResizeObserver(() => { renderer.setSize(container.clientWidth, container.clientHeight); camera.aspect = container.clientWidth / container.clientHeight; camera.updateProjectionMatrix(); }).observe(container);

function animate() { requestAnimationFrame(animate); controls.update(); renderer.render(scene, camera); }
animate();
