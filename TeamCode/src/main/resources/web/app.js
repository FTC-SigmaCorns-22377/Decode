import * as THREE from 'three';
import { scene, camera, renderer, controls, robotRef, robotPoint, robotHeading,
         balls, ballMaterials, ensureBallCount,
         wheelForceArrows, wheelForceConfig, wheelForceTmp,
         wheelForceScale, wheelForceMaxLen, wheelForceMinLen } from './scene.js';
import { initGamepad } from './gamepad.js';
import { syncCharts } from './charts.js';
import { updateMpcPredicted, updateMpcContours, updateMpcHorizon } from './viz/mpc.js';
import { updateGtsamViz, updateTagProjections, updateCameraView } from './viz/gtsam.js';
import { updateAimingViz, goalRing, goalRingMat } from './viz/aim.js';

console.log("App starting...");

// --- DOM Elements ---
const statusDisplay = document.getElementById('status');
const timeDisplay = document.getElementById('time-display');
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
const toggleCameraView = document.getElementById('toggle-camera-view');
const cameraViewContainer = document.getElementById('camera-view-container');
const uncertaintyScaleSlider = document.getElementById('uncertainty-scale');
const uncertaintyScaleVal = document.getElementById('uncertainty-scale-val');

const hudMpcHorizon = document.getElementById('hud-mpc-horizon');
const hudMpcSample = document.getElementById('hud-mpc-sample');
const hudMpcComplete = document.getElementById('hud-mpc-complete');

// --- State ---
let history = [];
let isLive = true;
let isPaused = false;
let playbackIndex = 0;
let playbackInterval = null;
let scrubberDragging = false;
let messageCount = 0;

let pathLine = null;
let pathSignature = '';
let mpcTargetLine = null;
let mpcTargetSignature = '';

const _euler = new THREE.Euler();
const _dir = new THREE.Vector3();
const _forceVec = new THREE.Vector3();

// --- Path / MPC Target ---
function setPath(points) {
    if (!points || points.length === 0) {
        if (pathLine) { scene.remove(pathLine); pathLine.geometry.dispose(); }
        pathLine = null;
        pathSignature = '';
        return;
    }
    const head = points[0], tail = points[points.length - 1];
    const signature = `${points.length}:${head.x.toFixed(3)},${head.y.toFixed(3)}:${tail.x.toFixed(3)},${tail.y.toFixed(3)}`;
    if (signature === pathSignature) return;
    pathSignature = signature;
    if (pathLine) { scene.remove(pathLine); pathLine.geometry.dispose(); }
    const verts = new Float32Array(points.length * 3);
    points.forEach((p, i) => { verts[i * 3] = p.x; verts[i * 3 + 1] = p.y; verts[i * 3 + 2] = 0.01; });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    pathLine = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0xffffff }));
    pathLine.visible = togglePath.checked;
    scene.add(pathLine);
}

function setMpcTarget(points) {
    if (!points || points.length === 0) {
        if (mpcTargetLine) { scene.remove(mpcTargetLine); mpcTargetLine.geometry.dispose(); }
        mpcTargetLine = null;
        mpcTargetSignature = '';
        return;
    }
    const head = points[0], tail = points[points.length - 1];
    const signature = `${points.length}:${head.x.toFixed(3)},${head.y.toFixed(3)}:${tail.x.toFixed(3)},${tail.y.toFixed(3)}`;
    if (signature === mpcTargetSignature) return;
    mpcTargetSignature = signature;
    if (mpcTargetLine) { scene.remove(mpcTargetLine); mpcTargetLine.geometry.dispose(); }
    const verts = new Float32Array(points.length * 3);
    points.forEach((p, i) => { verts[i * 3] = p.x; verts[i * 3 + 1] = p.y; verts[i * 3 + 2] = 0.03; });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    mpcTargetLine = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0x00ffff }));
    mpcTargetLine.visible = toggleMpcTarget.checked;
    scene.add(mpcTargetLine);
}

// --- Visual Updates ---
function updateVisuals(state) {
    if (!state) return;
    const robot = robotRef.value;
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
                balls[i].material = ballMaterials[b.color || 'orange'] || ballMaterials.orange;
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
            if (magnitude < 1e-6) { arrow.visible = false; return; }
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

    updateMpcPredicted(state.mpcPredicted);
    updateMpcContours(state.mpcContours);
    updateMpcHorizon(state.mpcHorizon, state.mpcPredicted);
    updateGtsamViz(state.gtsam, state.base);
    updateTagProjections(state.simVision);
    updateCameraView(state.simVision);
    updateAimingViz(state.aiming, state.base);

    if (state.mpcHorizon) {
        hudMpcHorizon.textContent = `${state.mpcHorizon.horizonSec.toFixed(2)}s`;
        hudMpcSample.textContent = `${state.mpcHorizon.sampleIndex}`;
        hudMpcComplete.textContent = state.mpcHorizon.trajectoryComplete ? 'YES' : 'NO';
        hudMpcComplete.className = 'hud-value' + (state.mpcHorizon.trajectoryComplete ? ' active' : '');
    }

    timeDisplay.textContent = `T: ${state.t.toFixed(2)}s`;
    if (!scrubberDragging) {
        scrubber.max = history.length - 1;
        scrubber.value = isLive ? history.length - 1 : playbackIndex;
    }
}

// --- Networking ---
const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const ws = new WebSocket(`${protocol}//${window.location.host}/sim`);

ws.onopen = () => { statusDisplay.textContent = 'Connected'; statusDisplay.style.color = '#00ff88'; };
ws.onclose = () => { statusDisplay.textContent = 'Disconnected'; statusDisplay.style.color = '#ff4444'; };
ws.onmessage = (e) => {
    const state = JSON.parse(e.data);
    if (history.length > 0 && state.t <= history[history.length - 1].t) return;
    history.push(state);
    if (history.length > 30000) history = history.slice(-20000);
    messageCount++;
    if (isLive) {
        updateVisuals(state);
        if (messageCount % 5 === 0) syncCharts(history, history.length - 1);
    }
};

fetch('/history').then(r => r.json()).then(data => {
    console.log(`History: ${data.length}`);
    if (history.length > 0) {
        history = [...data.filter(s => s.t < history[0].t), ...history];
    } else {
        history = data;
    }
    if (history.length > 0 && isLive) {
        updateVisuals(history[history.length - 1]);
        syncCharts(history, history.length - 1);
    }
}).catch(console.error);

// --- Gamepad ---
initGamepad(ws);

// --- Path polling ---
fetch('/path').then(r => r.json()).then(setPath).catch(console.error);
setInterval(() => { fetch('/path').then(r => r.json()).then(setPath).catch(() => {}); }, 2000);

// --- Playback Controls ---
const startReplay = () => {
    if (playbackInterval) clearInterval(playbackInterval);
    playbackInterval = setInterval(() => {
        if (playbackIndex >= history.length - 1) { stopReplay(); isPaused = true; btnPlayPause.textContent = 'Play'; return; }
        playbackIndex++;
        updateVisuals(history[playbackIndex]);
        syncCharts(history, playbackIndex);
    }, 20);
};
const stopReplay = () => { if (playbackInterval) clearInterval(playbackInterval); };

btnPlayPause.addEventListener('click', () => {
    if (isLive) { isLive = false; playbackIndex = history.length - 1; stopReplay(); isPaused = true; btnPlayPause.textContent = 'Play'; btnLive.disabled = false; btnReplay.disabled = true; }
    else if (isPaused) { startReplay(); isPaused = false; btnPlayPause.textContent = 'Pause'; }
    else { stopReplay(); isPaused = true; btnPlayPause.textContent = 'Play'; }
});
btnReplay.addEventListener('click', () => {
    isLive = false; btnLive.disabled = false; btnReplay.disabled = true;
    playbackIndex = 0; isPaused = false; btnPlayPause.textContent = 'Pause'; startReplay();
});
btnLive.addEventListener('click', () => {
    isLive = true; btnLive.disabled = true; btnReplay.disabled = false; btnPlayPause.textContent = 'Pause';
    stopReplay();
    if (history.length > 0) { updateVisuals(history[history.length - 1]); syncCharts(history, history.length - 1); }
});
scrubber.addEventListener('mousedown', () => { scrubberDragging = true; isLive = false; stopReplay(); });
scrubber.addEventListener('mouseup', () => { scrubberDragging = false; if (!isPaused) startReplay(); });
scrubber.addEventListener('input', () => {
    playbackIndex = parseInt(scrubber.value);
    updateVisuals(history[playbackIndex]);
    syncCharts(history, playbackIndex);
});

toggleRobotMesh.addEventListener('change', () => { if (robotRef.value) robotRef.value.visible = toggleRobotMesh.checked; });
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
toggleCameraView.addEventListener('change', () => { cameraViewContainer.style.display = toggleCameraView.checked ? 'block' : 'none'; });
uncertaintyScaleSlider.addEventListener('input', () => { uncertaintyScaleVal.textContent = uncertaintyScaleSlider.value; });

const canvasContainer = document.getElementById('canvas-container');
window.addEventListener('resize', () => {
    renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
    camera.aspect = canvasContainer.clientWidth / canvasContainer.clientHeight;
    camera.updateProjectionMatrix();
});
new ResizeObserver(() => {
    renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
    camera.aspect = canvasContainer.clientWidth / canvasContainer.clientHeight;
    camera.updateProjectionMatrix();
}).observe(canvasContainer);

// --- Animation Loop ---
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    if (goalRing.visible) {
        goalRingMat.opacity = 0.3 + 0.15 * Math.sin(Date.now() * 0.005);
    }
    renderer.render(scene, camera);
}
animate();
