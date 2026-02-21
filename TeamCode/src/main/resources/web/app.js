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

// New control viz toggles
const toggleMpcPredicted = document.getElementById('toggle-mpc-predicted');
const toggleMpcContours = document.getElementById('toggle-mpc-contours');
const toggleMpcHorizon = document.getElementById('toggle-mpc-horizon');
const toggleGtsamFused = document.getElementById('toggle-gtsam-fused');
const toggleGtsamLandmarks = document.getElementById('toggle-gtsam-landmarks');
const toggleGtsamVision = document.getElementById('toggle-gtsam-vision');
const toggleGtsamUncertainty = document.getElementById('toggle-gtsam-uncertainty');
const toggleAimTurret = document.getElementById('toggle-aim-turret');
const toggleAimGoal = document.getElementById('toggle-aim-goal');
const toggleAimArc = document.getElementById('toggle-aim-arc');
const toggleAimLead = document.getElementById('toggle-aim-lead');

// HUD elements
const hudMpcHorizon = document.getElementById('hud-mpc-horizon');
const hudMpcSample = document.getElementById('hud-mpc-sample');
const hudMpcComplete = document.getElementById('hud-mpc-complete');
const hudGtsamPose = document.getElementById('hud-gtsam-pose');
const hudGtsamUncertainty = document.getElementById('hud-gtsam-uncertainty');
const hudGtsamVision = document.getElementById('hud-gtsam-vision');
const hudGtsamTags = document.getElementById('hud-gtsam-tags');
const hudGtsamMode = document.getElementById('hud-gtsam-mode');
const hudAimLock = document.getElementById('hud-aim-lock');
const hudAimDist = document.getElementById('hud-aim-dist');
const hudAimTurret = document.getElementById('hud-aim-turret');
const hudAimFlywheel = document.getElementById('hud-aim-flywheel');
const hudAimVel = document.getElementById('hud-aim-vel');

// --- State ---
let history = [];
let isLive = true;
let isPaused = false;
let playbackIndex = 0;
let playbackInterval = null;
let scrubberDragging = false;
let robot = null;
let messageCount = 0;
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
scene.background = new THREE.Color(0x1a1a2e);

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
const floor = new THREE.Mesh(new THREE.PlaneGeometry(fieldSize, fieldSize), new THREE.MeshPhongMaterial({ color: 0x2a2a3a, side: THREE.DoubleSide }));
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

// =====================================================================
//  MPC VISUALIZATION LAYER
// =====================================================================

// MPC predicted trajectory line (green-yellow gradient, shows predicted robot evolution)
let mpcPredictedLine = null;
// MPC contour heading arrows (show target heading at each knot point)
let mpcContourGroup = new THREE.Group();
mpcContourGroup.name = 'mpcContours';
scene.add(mpcContourGroup);
// MPC knot point markers
let mpcKnotGroup = new THREE.Group();
mpcKnotGroup.name = 'mpcKnots';
scene.add(mpcKnotGroup);

function updateMpcPredicted(points) {
    // Remove old line
    if (mpcPredictedLine) {
        scene.remove(mpcPredictedLine);
        mpcPredictedLine.geometry.dispose();
        mpcPredictedLine.material.dispose();
        mpcPredictedLine = null;
    }
    if (!points || points.length < 2 || !toggleMpcPredicted.checked) return;

    const verts = new Float32Array(points.length * 3);
    const colors = new Float32Array(points.length * 3);
    points.forEach((p, i) => {
        verts[i * 3] = p.x;
        verts[i * 3 + 1] = p.y;
        verts[i * 3 + 2] = 0.04;
        // Gradient from bright green (near) to yellow (far) with glow effect
        const t = i / (points.length - 1);
        colors[i * 3] = 0.2 + 0.8 * t;     // R: increases to yellow
        colors[i * 3 + 1] = 1.0;             // G: always full
        colors[i * 3 + 2] = 0.2 * (1 - t);  // B: slight blue at start
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    const material = new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 2 });
    mpcPredictedLine = new THREE.Line(geometry, material);
    scene.add(mpcPredictedLine);
}

const contourArrowPool = [];
const contourDotPool = [];
const contourVelPool = [];

function updateMpcContours(contours) {
    // Hide all pooled objects
    contourArrowPool.forEach(a => { a.visible = false; });
    contourDotPool.forEach(d => { d.visible = false; });
    contourVelPool.forEach(v => { v.visible = false; });

    if (!contours || contours.length === 0 || !toggleMpcContours.checked) return;

    contours.forEach((c, i) => {
        // Contour dot (knot point marker)
        let dot = contourDotPool[i];
        if (!dot) {
            dot = new THREE.Mesh(
                new THREE.SphereGeometry(0.015, 8, 8),
                new THREE.MeshBasicMaterial({ color: 0x00ffff })
            );
            scene.add(dot);
            contourDotPool.push(dot);
        }
        dot.position.set(c.x, c.y, 0.04);
        dot.visible = true;

        // Heading arrow at each knot
        let arrow = contourArrowPool[i];
        if (!arrow) {
            arrow = new THREE.ArrowHelper(
                new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.08, 0x00ddff, 0.03, 0.015
            );
            scene.add(arrow);
            contourArrowPool.push(arrow);
        }
        arrow.position.set(c.x, c.y, 0.04);
        const dir = new THREE.Vector3(Math.cos(c.theta), Math.sin(c.theta), 0);
        arrow.setDirection(dir);
        arrow.setLength(0.08, 0.03, 0.015);
        arrow.visible = true;

        // Velocity vector at each knot (smaller, dimmer)
        if (Math.abs(c.vx) > 0.01 || Math.abs(c.vy) > 0.01) {
            let vel = contourVelPool[i];
            if (!vel) {
                vel = new THREE.ArrowHelper(
                    new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.05, 0x44aaff, 0.02, 0.01
                );
                scene.add(vel);
                contourVelPool.push(vel);
            }
            const speed = Math.sqrt(c.vx * c.vx + c.vy * c.vy);
            const velDir = new THREE.Vector3(c.vx / speed, c.vy / speed, 0);
            vel.position.set(c.x, c.y, 0.05);
            vel.setDirection(velDir);
            const velLen = Math.min(speed * 0.1, 0.15);
            vel.setLength(velLen, velLen * 0.3, velLen * 0.15);
            vel.visible = true;
        }
    });
}

// MPC horizon band (shaded region showing the variable-timestep planning window)
let mpcHorizonMesh = null;

function updateMpcHorizon(horizon, predicted, base) {
    if (mpcHorizonMesh) {
        scene.remove(mpcHorizonMesh);
        mpcHorizonMesh.geometry.dispose();
        mpcHorizonMesh.material.dispose();
        mpcHorizonMesh = null;
    }
    if (!horizon || !toggleMpcHorizon.checked || !predicted || predicted.length < 2) return;

    // Create a translucent ribbon along the predicted path with width proportional to timestep
    const ribbonVerts = [];
    const ribbonColors = [];
    const knotTimes = horizon.knotTimesMs || [];

    for (let i = 0; i < predicted.length - 1; i++) {
        const p0 = predicted[i];
        const p1 = predicted[i + 1];
        const dx = p1.x - p0.x;
        const dy = p1.y - p0.y;
        const len = Math.sqrt(dx * dx + dy * dy);
        if (len < 1e-6) continue;

        // Normal direction
        const nx = -dy / len;
        const ny = dx / len;

        // Width based on timestep (wider = slower timestep = more time at this knot)
        const dtMs = (knotTimes[i + 1] || 40) - (knotTimes[i] || 0);
        const width = 0.01 + dtMs * 0.0004;

        // Color: fast steps = bright cyan, slow steps = dim blue
        const tNorm = i / (predicted.length - 1);
        const r = 0.0;
        const g = 0.5 + 0.5 * (1 - tNorm);
        const b = 1.0;

        ribbonVerts.push(
            p0.x + nx * width, p0.y + ny * width, 0.02,
            p0.x - nx * width, p0.y - ny * width, 0.02,
            p1.x + nx * width, p1.y + ny * width, 0.02,
            p1.x - nx * width, p1.y - ny * width, 0.02,
            p1.x + nx * width, p1.y + ny * width, 0.02,
            p0.x - nx * width, p0.y - ny * width, 0.02
        );
        for (let j = 0; j < 6; j++) {
            ribbonColors.push(r, g, b);
        }
    }

    if (ribbonVerts.length === 0) return;

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(ribbonVerts), 3));
    geom.setAttribute('color', new THREE.BufferAttribute(new Float32Array(ribbonColors), 3));
    const mat = new THREE.MeshBasicMaterial({
        vertexColors: true, transparent: true, opacity: 0.25,
        side: THREE.DoubleSide, depthWrite: false
    });
    mpcHorizonMesh = new THREE.Mesh(geom, mat);
    scene.add(mpcHorizonMesh);
}

// =====================================================================
//  GTSAM FACTOR GRAPH VISUALIZATION LAYER
// =====================================================================

// Fused pose marker (distinct from raw odometry robot point)
const gtsamFusedMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.03, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x00ff88 })
);
gtsamFusedMarker.visible = false;
scene.add(gtsamFusedMarker);

const gtsamFusedHeading = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.15, 0x00ff88, 0.04, 0.02
);
gtsamFusedHeading.visible = false;
scene.add(gtsamFusedHeading);

// Fused pose trail
const FUSED_TRAIL_MAX = 300;
const fusedTrail = [];
let fusedTrailLine = null;

function updateFusedTrail(x, y) {
    fusedTrail.push({ x, y });
    if (fusedTrail.length > FUSED_TRAIL_MAX) fusedTrail.shift();

    if (fusedTrailLine) {
        scene.remove(fusedTrailLine);
        fusedTrailLine.geometry.dispose();
        fusedTrailLine.material.dispose();
    }
    if (fusedTrail.length < 2 || !toggleGtsamFused.checked) return;

    const verts = new Float32Array(fusedTrail.length * 3);
    const colors = new Float32Array(fusedTrail.length * 3);
    fusedTrail.forEach((p, i) => {
        verts[i * 3] = p.x;
        verts[i * 3 + 1] = p.y;
        verts[i * 3 + 2] = 0.005;
        const t = i / (fusedTrail.length - 1);
        colors[i * 3] = 0;
        colors[i * 3 + 1] = t;
        colors[i * 3 + 2] = 0.5 * t;
    });
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    fusedTrailLine = new THREE.Line(geom, new THREE.LineBasicMaterial({ vertexColors: true }));
    scene.add(fusedTrailLine);
}

// Uncertainty ring (grows/shrinks with localization uncertainty)
const uncertaintyRingGeo = new THREE.RingGeometry(0.05, 0.06, 32);
const uncertaintyRingMat = new THREE.MeshBasicMaterial({
    color: 0x00ff88, transparent: true, opacity: 0.4, side: THREE.DoubleSide, depthWrite: false
});
const uncertaintyRing = new THREE.Mesh(uncertaintyRingGeo, uncertaintyRingMat);
uncertaintyRing.rotation.x = 0; // Already in XY plane
uncertaintyRing.visible = false;
scene.add(uncertaintyRing);

// Landmark markers (AprilTag positions on the field)
const landmarkGroup = new THREE.Group();
landmarkGroup.name = 'landmarks';
scene.add(landmarkGroup);
let landmarksCreated = false;

function createLandmarkMarkers(landmarks) {
    // Clear existing
    while (landmarkGroup.children.length > 0) {
        const child = landmarkGroup.children[0];
        landmarkGroup.remove(child);
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
    }

    landmarks.forEach(lm => {
        // Tag billboard (small square to represent the AprilTag)
        const tagGeo = new THREE.PlaneGeometry(0.165, 0.165);
        const tagMat = new THREE.MeshBasicMaterial({
            color: 0xff8800, transparent: true, opacity: 0.6,
            side: THREE.DoubleSide, depthWrite: false
        });
        const tagMesh = new THREE.Mesh(tagGeo, tagMat);
        tagMesh.position.set(lm.x, lm.y, lm.z);
        // Rotate to face outward based on yaw
        tagMesh.rotation.set(Math.PI / 2, 0, lm.yaw);
        tagMesh.userData = { tagId: lm.tagId };
        landmarkGroup.add(tagMesh);

        // Tag border wireframe
        const borderGeo = new THREE.EdgesGeometry(tagGeo);
        const borderMat = new THREE.LineBasicMaterial({ color: 0xffaa00 });
        const border = new THREE.LineSegments(borderGeo, borderMat);
        border.position.copy(tagMesh.position);
        border.rotation.copy(tagMesh.rotation);
        landmarkGroup.add(border);

        // Vertical pillar from ground to tag
        const pillarVerts = new Float32Array([lm.x, lm.y, 0, lm.x, lm.y, lm.z]);
        const pillarGeo = new THREE.BufferGeometry();
        pillarGeo.setAttribute('position', new THREE.BufferAttribute(pillarVerts, 3));
        const pillar = new THREE.Line(pillarGeo, new THREE.LineBasicMaterial({ color: 0x664400, transparent: true, opacity: 0.3 }));
        landmarkGroup.add(pillar);
    });
    landmarksCreated = true;
}

// Vision rays from robot to detected landmarks
const visionRayGroup = new THREE.Group();
visionRayGroup.name = 'visionRays';
scene.add(visionRayGroup);

function updateVisionRays(gtsam, base) {
    // Clear old rays
    while (visionRayGroup.children.length > 0) {
        const child = visionRayGroup.children[0];
        visionRayGroup.remove(child);
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
    }

    if (!gtsam || !toggleGtsamVision.checked || !gtsam.detectedTags || gtsam.detectedTags.length === 0) return;

    const robotX = base.x;
    const robotY = base.y;
    const robotZ = (base.z || 0) + 0.22;  // Camera height

    gtsam.detectedTags.forEach(tagId => {
        const lm = (gtsam.landmarks || []).find(l => l.tagId === tagId);
        if (!lm) return;

        // Pulsing ray from robot to landmark
        const verts = new Float32Array([robotX, robotY, robotZ, lm.x, lm.y, lm.z]);
        const colors = new Float32Array([0, 1, 0.5, 1, 0.5, 0]);  // Green at robot, orange at tag
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
        geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        const ray = new THREE.Line(geom, new THREE.LineBasicMaterial({
            vertexColors: true, transparent: true, opacity: 0.7
        }));
        visionRayGroup.add(ray);

        // Detection marker at the tag (bright ring to show it's being tracked)
        const ringGeo = new THREE.RingGeometry(0.1, 0.12, 16);
        const ringMat = new THREE.MeshBasicMaterial({
            color: 0x00ff44, transparent: true, opacity: 0.6,
            side: THREE.DoubleSide, depthWrite: false
        });
        const ring = new THREE.Mesh(ringGeo, ringMat);
        ring.position.set(lm.x, lm.y, lm.z);
        ring.rotation.set(Math.PI / 2, 0, lm.yaw);
        visionRayGroup.add(ring);
    });
}

function updateGtsamViz(gtsam, base) {
    if (!gtsam) {
        gtsamFusedMarker.visible = false;
        gtsamFusedHeading.visible = false;
        uncertaintyRing.visible = false;
        return;
    }

    // Fused pose marker
    if (toggleGtsamFused.checked) {
        gtsamFusedMarker.position.set(gtsam.fusedX, gtsam.fusedY, 0.025);
        gtsamFusedMarker.visible = true;

        const dir = new THREE.Vector3(Math.cos(gtsam.fusedTheta), Math.sin(gtsam.fusedTheta), 0);
        gtsamFusedHeading.position.set(gtsam.fusedX, gtsam.fusedY, 0.025);
        gtsamFusedHeading.setDirection(dir);
        gtsamFusedHeading.setLength(0.15, 0.04, 0.02);
        gtsamFusedHeading.visible = true;

        updateFusedTrail(gtsam.fusedX, gtsam.fusedY);
    } else {
        gtsamFusedMarker.visible = false;
        gtsamFusedHeading.visible = false;
    }

    // Uncertainty ring
    if (toggleGtsamUncertainty.checked && gtsam.uncertainty > 0.001) {
        const radius = 0.05 + gtsam.uncertainty * 0.3;
        uncertaintyRing.scale.set(radius / 0.055, radius / 0.055, 1);
        uncertaintyRing.position.set(gtsam.fusedX, gtsam.fusedY, 0.01);
        // Color: green=low uncertainty, yellow=medium, red=high
        const u = Math.min(gtsam.uncertainty, 1.0);
        const r = Math.min(u * 2, 1);
        const g = Math.min((1 - u) * 2, 1);
        uncertaintyRingMat.color.setRGB(r, g, 0.1);
        uncertaintyRingMat.opacity = 0.2 + u * 0.5;
        uncertaintyRing.visible = true;
    } else {
        uncertaintyRing.visible = false;
    }

    // Landmarks
    if (toggleGtsamLandmarks.checked) {
        if (!landmarksCreated && gtsam.landmarks && gtsam.landmarks.length > 0) {
            createLandmarkMarkers(gtsam.landmarks);
        }
        landmarkGroup.visible = true;
    } else {
        landmarkGroup.visible = false;
    }

    // Vision rays
    updateVisionRays(gtsam, base);

    // Update HUD
    hudGtsamPose.textContent = `(${gtsam.fusedX.toFixed(3)}, ${gtsam.fusedY.toFixed(3)}, ${(gtsam.fusedTheta * 180 / Math.PI).toFixed(1)}°)`;

    const u = gtsam.uncertainty;
    hudGtsamUncertainty.textContent = u.toFixed(3);
    hudGtsamUncertainty.className = 'hud-value' + (u < 0.1 ? ' active' : u < 0.5 ? ' warning' : ' error');

    hudGtsamVision.textContent = gtsam.hasVision ? 'TRACKING' : 'NO VISION';
    hudGtsamVision.className = 'hud-value' + (gtsam.hasVision ? ' active' : ' warning');

    hudGtsamTags.textContent = (gtsam.detectedTags || []).join(', ') || 'none';

    hudGtsamMode.textContent = gtsam.usingPrediction ? 'PREDICTION' : (gtsam.initialized ? 'FUSED' : 'INIT...');
    hudGtsamMode.className = 'hud-value' + (gtsam.usingPrediction ? ' warning' : (gtsam.initialized ? ' active' : ''));
}

// =====================================================================
//  AIMING / MOVE-WHILE-SHOOT VISUALIZATION LAYER
// =====================================================================

// Goal marker (basket position)
const goalMarkerGeo = new THREE.CylinderGeometry(0.08, 0.08, 0.02, 24);
const goalMarkerMat = new THREE.MeshBasicMaterial({ color: 0xff4444, transparent: true, opacity: 0.6 });
const goalMarker = new THREE.Mesh(goalMarkerGeo, goalMarkerMat);
goalMarker.rotation.x = Math.PI / 2;
goalMarker.visible = false;
scene.add(goalMarker);

// Goal ring (pulsing targeting ring)
const goalRingGeo = new THREE.RingGeometry(0.12, 0.14, 32);
const goalRingMat = new THREE.MeshBasicMaterial({
    color: 0xff6666, transparent: true, opacity: 0.4, side: THREE.DoubleSide, depthWrite: false
});
const goalRing = new THREE.Mesh(goalRingGeo, goalRingMat);
goalRing.visible = false;
scene.add(goalRing);

// Vertical pillar at goal
const goalPillarVerts = new Float32Array([0, 0, 0, 0, 0, 1.0]);
const goalPillarGeo = new THREE.BufferGeometry();
goalPillarGeo.setAttribute('position', new THREE.BufferAttribute(goalPillarVerts, 3));
const goalPillar = new THREE.Line(goalPillarGeo, new THREE.LineBasicMaterial({
    color: 0xff4444, transparent: true, opacity: 0.3
}));
goalPillar.visible = false;
scene.add(goalPillar);

// Turret aiming ray
const turretRay = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1.0, 0xff2222, 0.05, 0.025
);
turretRay.visible = false;
scene.add(turretRay);

// Turret target direction (where it wants to aim)
const turretTargetRay = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.8, 0xff8888, 0.04, 0.02
);
turretTargetRay.visible = false;
scene.add(turretTargetRay);

// Projectile arc line
let projectileArcLine = null;

function updateProjectileArc(arcPoints) {
    if (projectileArcLine) {
        scene.remove(projectileArcLine);
        projectileArcLine.geometry.dispose();
        projectileArcLine.material.dispose();
        projectileArcLine = null;
    }
    if (!arcPoints || arcPoints.length < 2 || !toggleAimArc.checked) return;

    const verts = new Float32Array(arcPoints.length * 3);
    const colors = new Float32Array(arcPoints.length * 3);
    arcPoints.forEach((p, i) => {
        verts[i * 3] = p.x;
        verts[i * 3 + 1] = p.y;
        verts[i * 3 + 2] = p.z;
        // Red-orange-yellow gradient along arc with fade
        const t = i / (arcPoints.length - 1);
        colors[i * 3] = 1.0;
        colors[i * 3 + 1] = 0.3 + 0.7 * t;
        colors[i * 3 + 2] = 0.1 * t;
    });
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    projectileArcLine = new THREE.Line(geom, new THREE.LineBasicMaterial({
        vertexColors: true, transparent: true, opacity: 0.8
    }));
    scene.add(projectileArcLine);
}

// Lead compensation vector (shows robot velocity contribution to shot)
const leadArrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.1, 0xffaa00, 0.03, 0.015
);
leadArrow.visible = false;
scene.add(leadArrow);

// Targeting line from robot to goal (dashed look via segments)
let targetingLine = null;

function updateTargetingLine(base, aiming) {
    if (targetingLine) {
        scene.remove(targetingLine);
        targetingLine.geometry.dispose();
        targetingLine.material.dispose();
        targetingLine = null;
    }
    if (!aiming || !toggleAimGoal.checked) return;

    const robotX = base.x;
    const robotY = base.y;
    const robotZ = (base.z || 0) + 0.22;

    // Create dashed line by using segments
    const segments = 20;
    const verts = [];
    const colors = [];
    for (let i = 0; i < segments; i++) {
        if (i % 2 === 1) continue;  // Skip odd segments for dashed effect
        const t0 = i / segments;
        const t1 = (i + 1) / segments;
        const x0 = robotX + (aiming.goalX - robotX) * t0;
        const y0 = robotY + (aiming.goalY - robotY) * t0;
        const z0 = robotZ + (0.8 - robotZ) * t0;
        const x1 = robotX + (aiming.goalX - robotX) * t1;
        const y1 = robotY + (aiming.goalY - robotY) * t1;
        const z1 = robotZ + (0.8 - robotZ) * t1;
        verts.push(x0, y0, z0, x1, y1, z1);
        // Fade from green (locked) or red (no lock) to white
        const locked = aiming.hasTarget;
        colors.push(
            locked ? 0 : 1, locked ? 1 : 0.3, 0.2,
            0.8, 0.8, 0.8
        );
    }
    if (verts.length === 0) return;

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(verts), 3));
    geom.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
    targetingLine = new THREE.LineSegments(geom, new THREE.LineBasicMaterial({
        vertexColors: true, transparent: true, opacity: 0.5
    }));
    scene.add(targetingLine);
}

function updateAimingViz(aiming, base) {
    if (!aiming) {
        goalMarker.visible = false;
        goalRing.visible = false;
        goalPillar.visible = false;
        turretRay.visible = false;
        turretTargetRay.visible = false;
        leadArrow.visible = false;
        return;
    }

    // Goal marker
    if (toggleAimGoal.checked) {
        goalMarker.position.set(aiming.goalX, aiming.goalY, 0.8);
        goalMarker.visible = true;

        goalRing.position.set(aiming.goalX, aiming.goalY, 0.01);
        // Pulse effect via time-based opacity
        const pulse = 0.3 + 0.3 * Math.sin(Date.now() * 0.003);
        goalRingMat.opacity = aiming.hasTarget ? pulse : 0.15;
        goalRingMat.color.set(aiming.hasTarget ? 0x00ff44 : 0xff4444);
        goalRing.visible = true;

        goalPillar.position.set(aiming.goalX, aiming.goalY, 0);
        goalPillar.visible = true;
    } else {
        goalMarker.visible = false;
        goalRing.visible = false;
        goalPillar.visible = false;
    }

    // Turret aiming ray
    if (toggleAimTurret.checked && base) {
        const robotZ = (base.z || 0) + 0.22;

        // Actual turret direction
        const dir = new THREE.Vector3(
            Math.cos(aiming.turretFieldYaw),
            Math.sin(aiming.turretFieldYaw),
            0
        );
        turretRay.position.set(base.x, base.y, robotZ);
        turretRay.setDirection(dir);
        const rayLen = Math.min(aiming.targetDistance * 0.8, 2.0);
        turretRay.setLength(rayLen, 0.05, 0.025);
        // Color: green when locked, red when no target, yellow when using prediction
        const rayColor = aiming.hasTarget ? (aiming.usingPrediction ? 0xffaa00 : 0x00ff44) : 0xff4444;
        turretRay.setColor(rayColor);
        turretRay.visible = true;

        // Target turret direction (where PID wants it)
        const targetYaw = base.yaw + aiming.turretTargetAngle;
        const targetDir = new THREE.Vector3(Math.cos(targetYaw), Math.sin(targetYaw), 0);
        turretTargetRay.position.set(base.x, base.y, robotZ + 0.01);
        turretTargetRay.setDirection(targetDir);
        turretTargetRay.setLength(rayLen * 0.6, 0.04, 0.02);
        turretTargetRay.setColor(0x888888);
        turretTargetRay.visible = true;
    } else {
        turretRay.visible = false;
        turretTargetRay.visible = false;
    }

    // Projectile arc
    updateProjectileArc(aiming.projectileArc);

    // Lead compensation vector
    if (toggleAimLead.checked && base) {
        const speed = Math.sqrt(aiming.robotVx * aiming.robotVx + aiming.robotVy * aiming.robotVy);
        if (speed > 0.01) {
            const velDir = new THREE.Vector3(aiming.robotVx / speed, aiming.robotVy / speed, 0);
            leadArrow.position.set(base.x, base.y, (base.z || 0) + 0.25);
            leadArrow.setDirection(velDir);
            leadArrow.setLength(speed * 0.3, 0.03, 0.015);
            leadArrow.setColor(0xffaa00);
            leadArrow.visible = true;
        } else {
            leadArrow.visible = false;
        }
    } else {
        leadArrow.visible = false;
    }

    // Targeting line
    updateTargetingLine(base, aiming);

    // Update HUD
    hudAimLock.textContent = aiming.hasTarget ? (aiming.usingPrediction ? 'PREDICT' : 'LOCKED') : 'NO TARGET';
    hudAimLock.className = 'hud-value' + (aiming.hasTarget ? (aiming.usingPrediction ? ' warning' : ' active') : ' error');

    hudAimDist.textContent = `${aiming.targetDistance.toFixed(2)}m`;

    const turretDeg = (aiming.turretActualAngle * 180 / Math.PI).toFixed(1);
    const targetDeg = (aiming.turretTargetAngle * 180 / Math.PI).toFixed(1);
    hudAimTurret.textContent = `${turretDeg}° / ${targetDeg}°`;

    const fwPct = aiming.flywheelTarget > 0 ? ((aiming.flywheelOmega / aiming.flywheelTarget) * 100).toFixed(0) : '0';
    hudAimFlywheel.textContent = `${aiming.flywheelOmega.toFixed(0)} rad/s (${fwPct}%)`;
    hudAimFlywheel.className = 'hud-value' + (aiming.flywheelReady ? ' active' : ' warning');

    const vel = Math.sqrt(aiming.robotVx * aiming.robotVx + aiming.robotVy * aiming.robotVy);
    hudAimVel.textContent = `${vel.toFixed(2)} m/s`;
}


// =====================================================================
//  EXISTING VISUALIZATION (refactored)
// =====================================================================

function setPath(points) {
    if (!points || points.length === 0) {
        if (pathLine) { scene.remove(pathLine); pathLine.geometry.dispose(); }
        pathLine = null;
        pathSignature = '';
        return;
    }
    const head = points[0];
    const tail = points[points.length - 1];
    const signature = `${points.length}:${head.x.toFixed(3)},${head.y.toFixed(3)}:${tail.x.toFixed(3)},${tail.y.toFixed(3)}`;
    if (signature === pathSignature) return;
    pathSignature = signature;
    if (pathLine) { scene.remove(pathLine); pathLine.geometry.dispose(); }
    const verts = new Float32Array(points.length * 3);
    points.forEach((p, i) => {
        verts[i * 3] = p.x;
        verts[i * 3 + 1] = p.y;
        verts[i * 3 + 2] = 0.01;
    });
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
    const head = points[0];
    const tail = points[points.length - 1];
    const signature = `${points.length}:${head.x.toFixed(3)},${head.y.toFixed(3)}:${tail.x.toFixed(3)},${tail.y.toFixed(3)}`;
    if (signature === mpcTargetSignature) return;
    mpcTargetSignature = signature;
    if (mpcTargetLine) { scene.remove(mpcTargetLine); mpcTargetLine.geometry.dispose(); }
    const verts = new Float32Array(points.length * 3);
    points.forEach((p, i) => {
        verts[i * 3] = p.x;
        verts[i * 3 + 1] = p.y;
        verts[i * 3 + 2] = 0.03;
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    mpcTargetLine = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0x00ffff }));
    mpcTargetLine.visible = toggleMpcTarget.checked;
    scene.add(mpcTargetLine);
}

// --- Gamepad Functions ---
function updateGamepadStatusDisplay() {
    if (gamepadConnected) {
        gamepadStatusDisplay.textContent = "Gamepad Connected";
        gamepadStatusDisplay.style.color = "green";
    } else {
        gamepadStatusDisplay.textContent = "No Gamepad";
        gamepadStatusDisplay.style.color = "gray";
    }
}

function applyDeadzone(value, deadzone = GAMEPAD_DEADZONE) {
    return Math.abs(value) < deadzone ? 0.0 : value;
}

function sendGamepadState(ws, gamepadId, state) {
    if (ws.readyState === WebSocket.OPEN && state) {
        ws.send(JSON.stringify({ type: "gamepad", gamepadId, timestamp: Date.now(), input: state }));
    }
}

function setupGamepadEvents(gamepadIndex, ws) {
    let logCounter = 0;
    const pollState = () => {
        const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
        const gp = gamepads[gamepadIndex];
        if (!gp) return;
        currentGamepadState = {
            left_stick_x: applyDeadzone(gp.axes[0] || 0),
            left_stick_y: applyDeadzone(gp.axes[1] || 0),
            right_stick_x: applyDeadzone(gp.axes[2] || 0),
            right_stick_y: applyDeadzone(gp.axes[3] || 0),
            left_trigger: gp.buttons[6] ? gp.buttons[6].value : 0,
            right_trigger: gp.buttons[7] ? gp.buttons[7].value : 0,
            a: gp.buttons[0] ? gp.buttons[0].pressed : false,
            b: gp.buttons[1] ? gp.buttons[1].pressed : false,
            x: gp.buttons[2] ? gp.buttons[2].pressed : false,
            y: gp.buttons[3] ? gp.buttons[3].pressed : false,
            left_bumper: gp.buttons[4] ? gp.buttons[4].pressed : false,
            right_bumper: gp.buttons[5] ? gp.buttons[5].pressed : false,
            back: gp.buttons[8] ? gp.buttons[8].pressed : false,
            start: gp.buttons[9] ? gp.buttons[9].pressed : false,
            left_stick_button: gp.buttons[10] ? gp.buttons[10].pressed : false,
            right_stick_button: gp.buttons[11] ? gp.buttons[11].pressed : false,
            dpad_up: gp.buttons[12] ? gp.buttons[12].pressed : false,
            dpad_down: gp.buttons[13] ? gp.buttons[13].pressed : false,
            dpad_left: gp.buttons[14] ? gp.buttons[14].pressed : false,
            dpad_right: gp.buttons[15] ? gp.buttons[15].pressed : false
        };
        if (logCounter++ % 250 === 0) {
            console.log(`Gamepad: LX=${currentGamepadState.left_stick_x.toFixed(2)}, LY=${currentGamepadState.left_stick_y.toFixed(2)}`);
        }
        sendGamepadState(ws, 1, currentGamepadState);
    };
    return setInterval(pollState, 20);
}

let gamepadPollInterval = null;

// --- Visual Updates ---
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

    // --- Control Visualization Layers ---
    updateMpcPredicted(state.mpcPredicted);
    updateMpcContours(state.mpcContours);
    updateMpcHorizon(state.mpcHorizon, state.mpcPredicted, state.base);
    updateGtsamViz(state.gtsam, state.base);
    updateAimingViz(state.aiming, state.base);

    // MPC HUD
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

// --- Charts ---
function createChart(id, datasets) {
    const ctx = document.getElementById(id).getContext('2d');
    return new Chart(ctx, {
        type: 'line',
        data: { labels: [], datasets: datasets.map(ds => ({ label: ds.label, borderColor: ds.color, data: [], borderWidth: 1, pointRadius: 0 })) },
        options: {
            responsive: true, maintainAspectRatio: false, animation: false,
            scales: { x: { display: false } },
            plugins: { legend: { labels: { color: '#ccc', font: { size: 10 } } } }
        }
    });
}
const chartWheels = createChart('chart-wheels', [{label:'FL',color:'#ff4444'},{label:'FR',color:'#44ff44'},{label:'BL',color:'#4444ff'},{label:'BR',color:'#ffff44'}]);
const chartMechs = createChart('chart-mechanisms', [{label:'Flywheel',color:'#00ffff'},{label:'Turret',color:'#ff44ff'},{label:'Spindexer Pwr',color:'#ff8800'},{label:'Spindexer Ang',color:'#88ff00'}]);
const chartPos = createChart('chart-pos', [{label:'X',color:'#ff4444'},{label:'Y',color:'#44ff44'},{label:'Heading',color:'#4488ff'}]);
const chartError = createChart('chart-error', [
    {label:'Err X',color:'#ff4444'},
    {label:'Err Y',color:'#44ff44'},
    {label:'Err Heading',color:'#4488ff'},
    {label:'Err Vx',color:'#ff8800'},
    {label:'Err Vy',color:'#aa44ff'},
    {label:'Err Omega',color:'#00ffff'}
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

ws.onopen = () => { statusDisplay.textContent = "Connected"; statusDisplay.style.color = "#00ff88"; };
ws.onclose = () => { statusDisplay.textContent = "Disconnected"; statusDisplay.style.color = "#ff4444"; };
ws.onmessage = (e) => {
    const state = JSON.parse(e.data);
    if (history.length > 0 && state.t <= history[history.length - 1].t) return;
    history.push(state);
    if (history.length > 30000) history = history.slice(-20000);
    messageCount++;
    if (isLive) {
        updateVisuals(state);
        if (messageCount % 5 === 0) {
            syncCharts(history.length - 1);
        }
    }
};

fetch('/history').then(r => r.json()).then(data => {
    console.log(`History: ${data.length}`);
    if (history.length > 0) {
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

// --- Gamepad Setup ---
function initGamepad() {
    if (typeof gameControl === 'undefined') {
        setTimeout(initGamepad, 100);
        return;
    }
    gameControl.on('connect', (gamepad) => {
        console.log(`Gamepad connected: ${gamepad.id}`);
        gamepadConnected = true;
        updateGamepadStatusDisplay();
        if (gamepadPollInterval) clearInterval(gamepadPollInterval);
        gamepadPollInterval = setupGamepadEvents(gamepad.id, ws);
    });
    gameControl.on('disconnect', (gamepad) => {
        gamepadConnected = false;
        updateGamepadStatusDisplay();
        if (gamepadPollInterval) { clearInterval(gamepadPollInterval); gamepadPollInterval = null; }
    });
}
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

function animate() {
    requestAnimationFrame(animate);
    controls.update();

    // Animate pulsing goal ring
    if (goalRing.visible) {
        const pulse = 0.3 + 0.15 * Math.sin(Date.now() * 0.005);
        goalRingMat.opacity = pulse;
    }

    renderer.render(scene, camera);
}
animate();
