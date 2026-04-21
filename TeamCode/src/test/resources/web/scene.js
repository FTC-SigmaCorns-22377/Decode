import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const FIELD_SIZE = 3.6576;
const WALL_HEIGHT = 0.312;
const WALL_THICKNESS = 0.02;
const ROBOT_SIZE = 0.4;
const ROBOT_HEIGHT = 0.15;
const BALL_RADIUS = 0.0635;

// Scene setup
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a2e);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / (window.innerHeight - 40), 0.01, 50);
camera.position.set(2, 2, 2);
camera.lookAt(0, 0, 0);

const container = document.getElementById('canvas-container');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);

// Lighting
scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(3, 5, 3);
scene.add(dirLight);

// Grid
const grid = new THREE.GridHelper(FIELD_SIZE, 6, 0x444444, 0x333333);
scene.add(grid);

// Floor
const floorGeo = new THREE.PlaneGeometry(FIELD_SIZE, FIELD_SIZE);
const floorMat = new THREE.MeshStandardMaterial({ color: 0x2a2a3a, side: THREE.DoubleSide });
const floor = new THREE.Mesh(floorGeo, floorMat);
floor.rotation.x = -Math.PI / 2;
floor.position.y = -0.001;
scene.add(floor);

// Walls
const wallMat = new THREE.MeshStandardMaterial({ color: 0x555577, transparent: true, opacity: 0.4 });
const half = FIELD_SIZE / 2;
function addWall(sx, sy, sz, px, py, pz) {
    const geo = new THREE.BoxGeometry(sx, sy, sz);
    const mesh = new THREE.Mesh(geo, wallMat);
    mesh.position.set(px, py, pz);
    scene.add(mesh);
}
addWall(WALL_THICKNESS, WALL_HEIGHT, FIELD_SIZE, half + WALL_THICKNESS/2, WALL_HEIGHT/2, 0);
addWall(WALL_THICKNESS, WALL_HEIGHT, FIELD_SIZE, -half - WALL_THICKNESS/2, WALL_HEIGHT/2, 0);
addWall(FIELD_SIZE, WALL_HEIGHT, WALL_THICKNESS, 0, WALL_HEIGHT/2, half + WALL_THICKNESS/2);
addWall(FIELD_SIZE, WALL_HEIGHT, WALL_THICKNESS, 0, WALL_HEIGHT/2, -half - WALL_THICKNESS/2);

// Robot
const robotGeo = new THREE.BoxGeometry(ROBOT_SIZE, ROBOT_HEIGHT, ROBOT_SIZE);
const robotMat = new THREE.MeshStandardMaterial({ color: 0xff6600 });
const robotMesh = new THREE.Mesh(robotGeo, robotMat);
robotMesh.position.y = ROBOT_HEIGHT / 2;
scene.add(robotMesh);

// Direction indicator on robot (yellow cone pointing forward = local +Z)
const arrowGeo = new THREE.ConeGeometry(0.04, 0.1, 8);
const arrowMat = new THREE.MeshStandardMaterial({ color: 0xffcc00 });
const arrowMesh = new THREE.Mesh(arrowGeo, arrowMat);
arrowMesh.rotation.x = Math.PI / 2;
arrowMesh.position.set(0, ROBOT_HEIGHT / 2, ROBOT_SIZE / 2 - 0.02);
robotMesh.add(arrowMesh);

// Shooter turret (~1/4 robot volume, sits on top of robot)
const SHOOTER_WIDTH = 0.2;
const SHOOTER_LENGTH = 0.2;
const SHOOTER_HEIGHT = 0.075;
// Centered 2/3 back from front: front is +Z = ROBOT_SIZE/2, offset = 0.2 - 2/3*0.4 = -0.067
const SHOOTER_Z_OFFSET = ROBOT_SIZE / 2 - (2 / 3) * ROBOT_SIZE;

const shooterGroup = new THREE.Group();
shooterGroup.position.set(0, ROBOT_HEIGHT / 2 + SHOOTER_HEIGHT / 2, SHOOTER_Z_OFFSET);
robotMesh.add(shooterGroup);

const shooterGeo = new THREE.BoxGeometry(SHOOTER_WIDTH, SHOOTER_HEIGHT, SHOOTER_LENGTH);
const shooterMat = new THREE.MeshStandardMaterial({ color: 0x3366cc });
const shooterMesh = new THREE.Mesh(shooterGeo, shooterMat);
shooterGroup.add(shooterMesh);

// Shooter direction indicator (small cone on the front face of the shooter)
const shooterArrowGeo = new THREE.ConeGeometry(0.02, 0.06, 6);
const shooterArrowMat = new THREE.MeshStandardMaterial({ color: 0xff3333 });
const shooterArrowMesh = new THREE.Mesh(shooterArrowGeo, shooterArrowMat);
shooterArrowMesh.rotation.x = Math.PI / 2;
shooterArrowMesh.position.set(0, SHOOTER_HEIGHT / 2, SHOOTER_LENGTH / 2);
shooterGroup.add(shooterArrowMesh);

// --- Intake roller assembly (pivots at top-front edge of chassis) ---
const INTAKE_BAR_LENGTH = 0.127;
const INTAKE_ROLLER_RADIUS = 0.0254;
const INTAKE_WIDTH_VIS = 0.3;

const intakeGroup = new THREE.Group();
intakeGroup.position.set(0, ROBOT_HEIGHT / 2, ROBOT_SIZE / 2);
robotMesh.add(intakeGroup);

// Two bar meshes connecting pivot to roller.
// With the chirality-preserving remap, robot's left is at mesh local +X.
const barMat = new THREE.MeshStandardMaterial({ color: 0x888888 });
const barGeo = new THREE.BoxGeometry(0.02, 0.02, INTAKE_BAR_LENGTH);
const barLeft = new THREE.Mesh(barGeo, barMat);
barLeft.position.set(INTAKE_WIDTH_VIS / 2 - 0.02, 0, INTAKE_BAR_LENGTH / 2);
intakeGroup.add(barLeft);
const barRight = new THREE.Mesh(barGeo, barMat);
barRight.position.set(-INTAKE_WIDTH_VIS / 2 + 0.02, 0, INTAKE_BAR_LENGTH / 2);
intakeGroup.add(barRight);

// Roller cylinder at end of bars
const rollerMat = new THREE.MeshStandardMaterial({ color: 0x333333 });
const rollerGeo = new THREE.CylinderGeometry(INTAKE_ROLLER_RADIUS, INTAKE_ROLLER_RADIUS, INTAKE_WIDTH_VIS, 12);
rollerGeo.rotateZ(Math.PI / 2); // align along X
const rollerMesh = new THREE.Mesh(rollerGeo, rollerMat);
rollerMesh.position.set(0, 0, INTAKE_BAR_LENGTH);
intakeGroup.add(rollerMesh);

// --- Flywheel (front of shooter) ---
const FLYWHEEL_RADIUS = 0.05;
const flywheelMat = new THREE.MeshStandardMaterial({ color: 0x444444 });
const flywheelGeo = new THREE.CylinderGeometry(FLYWHEEL_RADIUS, FLYWHEEL_RADIUS, SHOOTER_WIDTH * 0.8, 16);
flywheelGeo.rotateZ(Math.PI / 2); // align along X
const flywheelMesh = new THREE.Mesh(flywheelGeo, flywheelMat);
flywheelMesh.position.set(0, 0, SHOOTER_LENGTH / 2);
shooterGroup.add(flywheelMesh);

// --- Hood (pivots at flywheel shaft, extends backward) ---
const hoodGroup = new THREE.Group();
hoodGroup.position.set(0, 0, SHOOTER_LENGTH / 2);
shooterGroup.add(hoodGroup);

const HOOD_RADIUS = FLYWHEEL_RADIUS * 4.2;
const HOOD_ARC = (40 / 180) * Math.PI; // 40-degree arc
const HOOD_OFFSET = (120 / 180) * Math.PI; // rotate 120 degrees backward
const hoodMat = new THREE.MeshStandardMaterial({ color: 0xcc6633, side: THREE.DoubleSide });
const hoodGeo = new THREE.CylinderGeometry(HOOD_RADIUS, HOOD_RADIUS, SHOOTER_WIDTH * 0.8, 16, 1, false, -HOOD_ARC / 2 + HOOD_OFFSET, HOOD_ARC);
hoodGeo.rotateZ(Math.PI / 2); // align along X like flywheel
const hoodMesh = new THREE.Mesh(hoodGeo, hoodMat);
hoodGroup.add(hoodMesh);

// ---- Goal structures (DECODE game-accurate) ----
// Jolt XYZ maps directly to Three.js XYZ for static geometry.
// Three.js rotation.y has OPPOSITE sign convention from Jolt Quat around Y.
// Goals: right-triangle structures in field corners. -X wall = back, ±Z walls = sides.
// Red: corner (-HF, +HF). Blue: corner (-HF, -HF).

const HF = FIELD_SIZE / 2;
const GOAL_LEG = 0.6858;
const GOAL_WALL_THICK_VIS = 0.03; // match physics thickness for accurate collision visualization
const GOAL_LIP_HEIGHT = 0.9843;
const GOAL_TOTAL_HEIGHT = 1.3716;
const CRAMP_LENGTH = 1.00;
const CRAMP_WIDTH = 0.16;
const CRAMP_START_H = 0.49;    // input end height (~halfway up goal)
const CRAMP_END_H = 0.127;     // output end height (~ball diameter)
const CRAMP_MID_H = (CRAMP_START_H + CRAMP_END_H) / 2;
const CRAMP_SLOPE_ANGLE = Math.atan2(CRAMP_START_H - CRAMP_END_H, CRAMP_LENGTH);
const CRAMP_RAIL_HEIGHT = 0.10;
const CRAMP_RAIL_THICK = 0.01;
const CRAMP_WALL_THICK = 0.01;
const GATE_CLOSED_H = 0.14;
const GATE_WIDTH = 0.16;
const GATE_THICK = 0.02;
const LEVER_LENGTH = 0.15;
const LEVER_WIDTH = 0.04;
const LEVER_THICKNESS = 0.02;

// Creates a tapered wall mesh using BufferGeometry with explicit vertices.
// p1/p2: {x,z} endpoints. h1/h2: heights at each end. nx/nz: thickness direction.
function createTaperedWall(p1, h1, p2, h2, thick, nx, nz, material) {
    // 8 vertices: 4 bottom, 4 top (inner face then outer face)
    const verts = new Float32Array([
        // Inner face (toward goal interior)
        p1.x, 0,  p1.z,       // 0: bottom p1
        p2.x, 0,  p2.z,       // 1: bottom p2
        p2.x, h2, p2.z,       // 2: top p2
        p1.x, h1, p1.z,       // 3: top p1
        // Outer face (offset by thickness)
        p1.x+thick*nx, 0,  p1.z+thick*nz,   // 4
        p2.x+thick*nx, 0,  p2.z+thick*nz,   // 5
        p2.x+thick*nx, h2, p2.z+thick*nz,   // 6
        p1.x+thick*nx, h1, p1.z+thick*nz,   // 7
    ]);
    const idx = [
        0,1,2, 0,2,3,  // inner face
        5,4,7, 5,7,6,  // outer face
        3,2,6, 3,6,7,  // top
        4,5,1, 4,1,0,  // bottom
        4,0,3, 4,3,7,  // p1 end
        1,5,6, 1,6,2,  // p2 end
    ];
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
    geo.setIndex(idx);
    geo.computeVertexNormals();
    return new THREE.Mesh(geo, material);
}

function createGoalStructure(zSign, color) {
    const group = new THREE.Group();
    // Goal corner lives at +X under the chirality-preserving mapping.
    const cX = HF;
    const cZ = zSign * HF;
    const t = GOAL_WALL_THICK_VIS;

    const goalMat = new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.55, side: THREE.DoubleSide });
    const rampMat = new THREE.MeshStandardMaterial({ color: 0x999999, transparent: true, opacity: 0.7 });
    const gateMat = new THREE.MeshStandardMaterial({ color: 0xcccc00 });
    const leverMat = new THREE.MeshStandardMaterial({ color: 0x44cc44 });

    const goalOffset = CRAMP_WIDTH; // triangle inset from ±Z wall

    // --- Side wall A: along +X (back) wall, runs in Z direction ---
    // Tapered: lip height at front end, total height at corner end
    const wallA = createTaperedWall(
        {x: cX, z: cZ - zSign * (goalOffset + GOAL_LEG)}, GOAL_LIP_HEIGHT,  // front (low)
        {x: cX, z: cZ - zSign * goalOffset}, GOAL_TOTAL_HEIGHT,              // corner (tall)
        t, -1, 0, goalMat  // thickness in -X direction (toward field interior)
    );
    group.add(wallA);

    // --- Side wall B: flush against ±Z (side) wall, runs in X direction ---
    const wallB = createTaperedWall(
        {x: cX - GOAL_LEG, z: cZ}, GOAL_LIP_HEIGHT,  // front (low)
        {x: cX, z: cZ}, GOAL_TOTAL_HEIGHT,             // corner (tall)
        t, 0, -zSign, goalMat  // thickness toward field center
    );
    group.add(wallB);

    // Corner connector: along +X wall, bridges wall B (at ±Z wall) to wall A (offset)
    const cornerConnGeo = new THREE.BoxGeometry(GOAL_WALL_THICK_VIS, GOAL_TOTAL_HEIGHT, goalOffset);
    const cornerConnMesh = new THREE.Mesh(cornerConnGeo, goalMat);
    cornerConnMesh.position.set(cX - GOAL_WALL_THICK_VIS / 2, GOAL_TOTAL_HEIGHT / 2, cZ - zSign * goalOffset / 2);
    group.add(cornerConnMesh);

    // --- Front wall (diagonal hypotenuse) at lip height ---
    const frontLen = GOAL_LEG * Math.SQRT2;
    const frontMidX = cX - GOAL_LEG / 2;
    const frontMidZ = cZ - zSign * (goalOffset + GOAL_LEG / 2);
    const frontAngleJS = zSign * Math.PI / 4;

    const frontGeo = new THREE.BoxGeometry(frontLen, GOAL_LIP_HEIGHT, GOAL_WALL_THICK_VIS);
    const frontMesh = new THREE.Mesh(frontGeo, goalMat);
    frontMesh.position.set(frontMidX, GOAL_LIP_HEIGHT / 2, frontMidZ);
    frontMesh.rotation.y = frontAngleJS;
    group.add(frontMesh);

    // --- Rectangular extension + Classifier ramp ---
    // Ramp is flush against the ±Z wall. Triangle is offset inward by CRAMP_WIDTH.
    // A rectangular extension bridges the offset triangle to the wall-hugging ramp.
    const rampStartX = cX - GOAL_LEG;
    const rampEndX = rampStartX - CRAMP_LENGTH;
    const rampMidX = (rampStartX + rampEndX) / 2;
    const rampZ = cZ - zSign * CRAMP_WIDTH / 2; // flush against ±Z wall

    // Perpendicular wall at triangle front, connecting ±Z wall to offset triangle
    const perpLen = goalOffset;
    const perpMidZ = cZ - zSign * perpLen / 2;
    const perpGeo = new THREE.BoxGeometry(GOAL_WALL_THICK_VIS, GOAL_LIP_HEIGHT, perpLen);
    const perpMesh = new THREE.Mesh(perpGeo, goalMat);
    perpMesh.position.set(rampStartX, GOAL_LIP_HEIGHT / 2, perpMidZ);
    group.add(perpMesh);

    // Ramp floor with steep slope
    const rampFloorGeo = new THREE.BoxGeometry(CRAMP_LENGTH, 0.01, CRAMP_WIDTH);
    const rampFloorMesh = new THREE.Mesh(rampFloorGeo, rampMat);
    rampFloorMesh.position.set(rampMidX, CRAMP_MID_H, rampZ);
    rampFloorMesh.rotation.z = CRAMP_SLOPE_ANGLE; // lifts +X end (input near goal)
    group.add(rampFloorMesh);

    // Ramp side rails: top edge follows the ramp slope
    const railStartH = CRAMP_START_H + CRAMP_RAIL_HEIGHT;
    const railEndH = CRAMP_END_H + CRAMP_RAIL_HEIGHT;
    const ht = CRAMP_RAIL_THICK / 2; // half thickness

    function createSlopedRail(railZ) {
        const v = new Float32Array([
            rampStartX, 0,          railZ - ht,  // 0: bottom start inner
            rampEndX,   0,          railZ - ht,  // 1: bottom end inner
            rampEndX,   railEndH,   railZ - ht,  // 2: top end inner
            rampStartX, railStartH, railZ - ht,  // 3: top start inner
            rampStartX, 0,          railZ + ht,  // 4: bottom start outer
            rampEndX,   0,          railZ + ht,  // 5: bottom end outer
            rampEndX,   railEndH,   railZ + ht,  // 6: top end outer
            rampStartX, railStartH, railZ + ht,  // 7: top start outer
        ]);
        const idx = [
            0,1,2, 0,2,3,  5,4,7, 5,7,6,  // inner/outer faces
            3,2,6, 3,6,7,  4,5,1, 4,1,0,  // top/bottom
            4,0,3, 4,3,7,  1,5,6, 1,6,2,  // start/end caps
        ];
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
        geo.setIndex(idx);
        geo.computeVertexNormals();
        return new THREE.Mesh(geo, rampMat);
    }

    const railAMesh = createSlopedRail(rampZ - CRAMP_WIDTH / 2);
    group.add(railAMesh);
    const railBMesh = createSlopedRail(rampZ + CRAMP_WIDTH / 2);
    group.add(railBMesh);

    // Trapezoidal wall below the ramp (red/blue colored)
    // Bottom: on floor, full length. Top: parallel with ramp bottom (sloped).
    // 8 vertices forming a wedge/trapezoid shape.
    const hw = CRAMP_WIDTH / 2 + CRAMP_WALL_THICK; // slightly wider than ramp
    const trapVerts = new Float32Array([
        // Front face (toward field center, -zSign side)
        rampStartX, 0,          rampZ - zSign * hw,  // 0: bottom start
        rampEndX,   0,          rampZ - zSign * hw,  // 1: bottom end
        rampEndX,   CRAMP_END_H,  rampZ - zSign * hw,  // 2: top end (low)
        rampStartX, CRAMP_START_H, rampZ - zSign * hw,  // 3: top start (high)
        // Back face (toward wall, +zSign side)
        rampStartX, 0,          rampZ + zSign * hw,  // 4
        rampEndX,   0,          rampZ + zSign * hw,  // 5
        rampEndX,   CRAMP_END_H,  rampZ + zSign * hw,  // 6
        rampStartX, CRAMP_START_H, rampZ + zSign * hw,  // 7
    ]);
    const trapIdx = [
        0,1,2, 0,2,3,  5,4,7, 5,7,6,  // front/back
        3,2,6, 3,6,7,  4,5,1, 4,1,0,  // top/bottom
        4,0,3, 4,3,7,  1,5,6, 1,6,2,  // left/right ends
    ];
    const trapGeo = new THREE.BufferGeometry();
    trapGeo.setAttribute('position', new THREE.Float32BufferAttribute(trapVerts, 3));
    trapGeo.setIndex(trapIdx);
    trapGeo.computeVertexNormals();
    const trapMat = new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.4, side: THREE.DoubleSide });
    group.add(new THREE.Mesh(trapGeo, trapMat));

    // Gate at output end (at ramp end height)
    const gateGeo = new THREE.BoxGeometry(GATE_THICK, GATE_CLOSED_H, GATE_WIDTH);
    const gateMesh = new THREE.Mesh(gateGeo, gateMat);
    gateMesh.position.set(rampEndX, CRAMP_END_H + GATE_CLOSED_H / 2, rampZ);
    group.add(gateMesh);

    // --- Lever (see-saw at ramp output end, extends toward field center) ---
    const leverZ = rampZ - zSign * (CRAMP_WIDTH / 2 + LEVER_LENGTH / 2);
    const leverGeo = new THREE.BoxGeometry(LEVER_WIDTH, LEVER_THICKNESS, LEVER_LENGTH);
    const leverMesh = new THREE.Mesh(leverGeo, leverMat);
    leverMesh.position.set(rampEndX, CRAMP_END_H, leverZ);
    group.add(leverMesh);

    scene.add(group);

    // Score sprite
    const scoreCanvas = document.createElement('canvas');
    scoreCanvas.width = 128;
    scoreCanvas.height = 64;
    const scoreCtx = scoreCanvas.getContext('2d');
    const scoreTex = new THREE.CanvasTexture(scoreCanvas);
    const scoreSprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: scoreTex }));
    scoreSprite.scale.set(0.5, 0.25, 1);
    scoreSprite.position.set(cX - GOAL_LEG / 3, GOAL_TOTAL_HEIGHT + 0.15, cZ - zSign * GOAL_LEG / 3);
    scene.add(scoreSprite);

    return { group, gateMesh, leverMesh, scoreCanvas, scoreCtx, scoreTex, scoreSprite };
}

const redGoal = createGoalStructure(1, 0x3333cc);
const blueGoal = createGoalStructure(-1, 0xcc3333);

function updateScoreSprite(goal, label, score, bgColor) {
    const { scoreCanvas, scoreCtx, scoreTex } = goal;
    scoreCtx.clearRect(0, 0, scoreCanvas.width, scoreCanvas.height);
    scoreCtx.fillStyle = bgColor;
    scoreCtx.fillRect(0, 0, scoreCanvas.width, scoreCanvas.height);
    scoreCtx.fillStyle = '#ffffff';
    scoreCtx.font = 'bold 28px monospace';
    scoreCtx.textAlign = 'center';
    scoreCtx.textBaseline = 'middle';
    scoreCtx.fillText(`${label}: ${score}`, scoreCanvas.width / 2, scoreCanvas.height / 2);
    scoreTex.needsUpdate = true;
}

// ---- Ball pool ----
const ballMeshes = [];
const greenMat = new THREE.MeshStandardMaterial({ color: 0x00cc44 });
const purpleMat = new THREE.MeshStandardMaterial({ color: 0x9933ff });
const ballGeo = new THREE.SphereGeometry(BALL_RADIUS, 16, 12);

function updateRobot(x, y, theta, turretAngle, intakeAngle, hoodAngle, flywheelRPM, intakeRollerRPM) {
    // sim (x=forward, y=left, z=up) -> Three.js (x=sim.y, y=sim.z, z=sim.x).
    // This mapping is chirality-preserving (det = +1) so sim's "left" renders
    // on the robot's left. Sim theta (CCW about sim.z) becomes positive
    // rotation.y in three.js.
    robotMesh.position.set(y, ROBOT_HEIGHT / 2, x);
    robotMesh.rotation.y = theta;
    // Turret rotates about Y axis relative to robot
    if (turretAngle !== undefined) {
        shooterGroup.rotation.y = turretAngle;
    }
    // Intake pivots around X axis
    if (intakeAngle !== undefined) {
        intakeGroup.rotation.x = -intakeAngle;
    }
    // Hood pivots around X axis (negative = tilts upward)
    if (hoodAngle !== undefined) {
        hoodGroup.rotation.x = -hoodAngle;
    }
    // Flywheel visual spin
    if (flywheelRPM !== undefined && flywheelRPM !== 0) {
        flywheelMesh.rotation.x += (flywheelRPM / 60) * 2 * Math.PI * 0.016; // ~60fps
    }
    // Cone color indicators: green = active, default = inactive
    shooterArrowMat.color.set(Math.abs(flywheelRPM || 0) > 10 ? 0x00ff00 : 0xff3333);
    arrowMat.color.set(Math.abs(intakeRollerRPM || 0) > 10 ? 0x00ff00 : 0xffcc00);
}

function updateBalls(balls) {
    // Remove excess meshes
    while (ballMeshes.length > balls.length) {
        const m = ballMeshes.pop();
        scene.remove(m);
    }
    // Add/update meshes
    for (let i = 0; i < balls.length; i++) {
        let mesh;
        if (i < ballMeshes.length) {
            mesh = ballMeshes[i];
        } else {
            mesh = new THREE.Mesh(ballGeo, greenMat);
            scene.add(mesh);
            ballMeshes.push(mesh);
        }
        const b = balls[i];
        mesh.position.set(b.y, b.z, b.x); // sim (x=fwd, y=left, z=up) -> Three.js (x=y, y=z, z=x)
        mesh.material = b.color === 'green' ? greenMat : purpleMat;
    }
}

// ---- Shot visualization (goal marker + ballistic trajectories) ----
const goalMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.05, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xff3333 })
);
goalMarker.visible = false;
scene.add(goalMarker);

// One persistent line per kind. Materials are fixed at construction so we
// don't allocate per frame. Dashed lines need computeLineDistances() after
// each geometry update.
const TRAJECTORY_KINDS = {
    current:     { color: 0x2288ff, opacity: 0.9 },
    target:      { color: 0x00dd44, opacity: 0.9 },
    secondary:   { color: 0x00dd44, opacity: 0.45 },
    tertiary:    { color: 0x00dd44, opacity: 0.25 },
    preposition: { color: 0xffaa00, opacity: 0.6 },
};

const trajectoryLines = {};
for (const [kind, cfg] of Object.entries(TRAJECTORY_KINDS)) {
    const mat = new THREE.LineBasicMaterial({
        color: cfg.color,
        transparent: true,
        opacity: cfg.opacity,
    });
    const geo = new THREE.BufferGeometry();
    const line = new THREE.Line(geo, mat);
    line.frustumCulled = false;
    line.visible = false;
    trajectoryLines[kind] = line;
    scene.add(line);
}

function updateShotViz(shotViz) {
    if (!shotViz) {
        goalMarker.visible = false;
        for (const line of Object.values(trajectoryLines)) line.visible = false;
        return;
    }

    const goal = shotViz.goal;
    if (goal) {
        goalMarker.position.set(goal.y, goal.z, goal.x);
        goalMarker.visible = true;
    } else {
        goalMarker.visible = false;
    }

    const seen = new Set();
    const trajs = shotViz.trajectories || [];
    for (const t of trajs) {
        const line = trajectoryLines[t.kind];
        if (!line) continue;
        const pts = t.points || [];
        if (pts.length < 2) { line.visible = false; continue; }
        const v3 = new Array(pts.length);
        for (let i = 0; i < pts.length; i++) {
            const p = pts[i]; // [x, y, z] in sim frame
            v3[i] = new THREE.Vector3(p[1], p[2], p[0]);
        }
        line.geometry.setFromPoints(v3);
        line.visible = true;
        seen.add(t.kind);
    }

    for (const kind of Object.keys(trajectoryLines)) {
        if (!seen.has(kind)) trajectoryLines[kind].visible = false;
    }
}

// ---- Ball-tracker overlays + robot-POV camera panel ----

/**
 * Convert a field-frame point (x, y, z) to three.js world coords.
 *   three.x <- field.y
 *   three.y <- field.z      (height/up)
 *   three.z <- field.x
 * Consistent with the transform used by updateBalls / updateShotViz.
 */
function fieldToThree(fx, fy, fz = 0) { return new THREE.Vector3(fy, fz, fx); }

// Pools: we keep stable mesh arrays and show/hide them, so there is no
// per-frame allocation. Growing the pool is the only thing that allocates.
const MAX_TRACKS = 16;
const MAX_TRUTH_BALLS = 64;

const groundTruthGroup = new THREE.Group();
scene.add(groundTruthGroup);
const truthBallMeshes = [];
function ensureTruthBall(i) {
    while (truthBallMeshes.length <= i) {
        const geo = new THREE.SphereGeometry(0.04, 12, 8);
        const mat = new THREE.MeshBasicMaterial({
            color: 0x8affa3, transparent: true, opacity: 0.55,
        });
        const m = new THREE.Mesh(geo, mat);
        m.visible = false;
        groundTruthGroup.add(m);
        truthBallMeshes.push(m);
    }
    return truthBallMeshes[i];
}

const trackGroup = new THREE.Group();
scene.add(trackGroup);
const trackMeshes = [];       // small solid spheres at track position
const trackLabels = [];       // Sprite labels with track id
const trackEllipses = [];     // 2σ position ellipses as thin rings
function makeTrackMesh() {
    const group = new THREE.Group();
    const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(0.035, 12, 8),
        new THREE.MeshBasicMaterial({ color: 0xffd54f }),
    );
    group.add(sphere);
    const labelCanvas = document.createElement('canvas');
    labelCanvas.width = 128; labelCanvas.height = 64;
    const labelTex = new THREE.CanvasTexture(labelCanvas);
    const labelMat = new THREE.SpriteMaterial({ map: labelTex, transparent: true });
    const sprite = new THREE.Sprite(labelMat);
    sprite.scale.set(0.20, 0.10, 1);
    sprite.position.set(0, 0.08, 0);
    group.add(sprite);
    return { group, sphere, sprite, labelCanvas, labelTex };
}
function ensureTrack(i) {
    while (trackMeshes.length <= i) {
        const t = makeTrackMesh();
        t.group.visible = false;
        trackGroup.add(t.group);
        trackMeshes.push(t);

        // 2σ ellipse: we use a thin ring deformed to match the 2x2 cov.
        // Build a unit circle once; scale per frame.
        const segs = 48;
        const positions = new Float32Array((segs + 1) * 3);
        for (let s = 0; s <= segs; s++) {
            const a = (s / segs) * 2 * Math.PI;
            positions[s * 3] = Math.cos(a);
            positions[s * 3 + 1] = 0;
            positions[s * 3 + 2] = Math.sin(a);
        }
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        const mat = new THREE.LineBasicMaterial({
            color: 0x80deea, transparent: true, opacity: 0.9,
        });
        const line = new THREE.Line(geo, mat);
        line.frustumCulled = false;
        line.visible = false;
        trackGroup.add(line);
        trackEllipses.push(line);
    }
    return { track: trackMeshes[i], ellipse: trackEllipses[i] };
}

// Eigendecomposition of a 2x2 symmetric matrix. Returns { angle, l1, l2 }.
function eig2(a, b, c, d) {
    // Matrix is [[a,b],[b,d]] assuming symmetry; clamp to symmetric.
    const sym = (b + c) / 2;
    const tr = a + d;
    const det = a * d - sym * sym;
    const disc = Math.max(0, tr * tr / 4 - det);
    const s = Math.sqrt(disc);
    const l1 = tr / 2 + s;
    const l2 = tr / 2 - s;
    const angle = Math.atan2(l1 - a, sym || 1e-12);
    return { angle, l1: Math.max(l1, 0), l2: Math.max(l2, 0) };
}

const frustumLine = (() => {
    const geo = new THREE.BufferGeometry();
    const mat = new THREE.LineBasicMaterial({
        color: 0xffffff, transparent: true, opacity: 0.35,
    });
    const l = new THREE.LineLoop(geo, mat);
    l.frustumCulled = false;
    l.visible = false;
    scene.add(l);
    return l;
})();

// ---- Robot-POV camera panel ----
const camPanelWrap = document.getElementById('camera-panel-wrap');
const camPanelCanvas = document.createElement('canvas');
camPanelWrap.insertBefore(camPanelCanvas, camPanelWrap.firstChild);
const camPanelRenderer = new THREE.WebGLRenderer({ canvas: camPanelCanvas, antialias: true });
camPanelRenderer.setPixelRatio(window.devicePixelRatio);
const camPanelCamera = new THREE.PerspectiveCamera(60, 16 / 9, 0.02, 50);

const camPanelOverlayCanvas = document.getElementById('camera-overlay');
const overlayCtx = camPanelOverlayCanvas.getContext('2d');

function sizeCameraPanel(imageW, imageH) {
    // Panel width = side-panel width; height chosen to preserve camera aspect.
    const w = camPanelWrap.clientWidth;
    const aspect = (imageW && imageH) ? (imageW / imageH) : (16 / 9);
    const h = Math.round(w / aspect);
    camPanelWrap.style.height = h + 'px';
    camPanelRenderer.setSize(w, h, false);
    camPanelOverlayCanvas.width = w * window.devicePixelRatio;
    camPanelOverlayCanvas.height = h * window.devicePixelRatio;
    camPanelOverlayCanvas.style.width = w + 'px';
    camPanelOverlayCanvas.style.height = h + 'px';
    overlayCtx.setTransform(window.devicePixelRatio, 0, 0, window.devicePixelRatio, 0, 0);
    camPanelCamera.aspect = aspect;
    camPanelCamera.updateProjectionMatrix();
}
sizeCameraPanel(1280, 720);

let lastCamera = null;     // cached intrinsics so we only resize on change

// Core exported function — called each frame from app.js.
function updateTracker(trackerState) {
    // Ground truth balls.
    for (let i = 0; i < truthBallMeshes.length; i++) truthBallMeshes[i].visible = false;
    frustumLine.visible = false;
    for (let i = 0; i < trackMeshes.length; i++) {
        trackMeshes[i].group.visible = false;
        trackEllipses[i].visible = false;
    }
    // Clear pixel overlay unconditionally.
    overlayCtx.clearRect(0, 0, camPanelOverlayCanvas.clientWidth, camPanelOverlayCanvas.clientHeight);

    if (!trackerState || !trackerState.camera) return;

    const cam = trackerState.camera;
    if (!lastCamera || lastCamera.width !== cam.width || lastCamera.height !== cam.height) {
        sizeCameraPanel(cam.width, cam.height);
        lastCamera = { width: cam.width, height: cam.height };
        // Vertical FOV derived from intrinsics.
        const fovVRad = 2 * Math.atan(cam.height / 2 / cam.fy);
        camPanelCamera.fov = fovVRad * 180 / Math.PI;
        camPanelCamera.updateProjectionMatrix();
    }

    // Ground-truth markers.
    const truths = trackerState.ballTruth || [];
    for (let i = 0; i < Math.min(truths.length, MAX_TRUTH_BALLS); i++) {
        const b = truths[i];
        const m = ensureTruthBall(i);
        const v = fieldToThree(b.x, b.y, b.z);
        m.position.copy(v);
        m.visible = true;
    }

    // Camera frustum on the floor.
    const frustum = cam.frustumGround || [];
    if (frustum.length >= 2) {
        const pts = frustum.map(([fx, fy]) => fieldToThree(fx, fy, 0.002));
        frustumLine.geometry.setFromPoints(pts);
        frustumLine.visible = true;
    }

    // Tracks + ellipses.
    const tracks = trackerState.tracks || [];
    const targetId = trackerState.targetId;
    for (let i = 0; i < Math.min(tracks.length, MAX_TRACKS); i++) {
        const tr = tracks[i];
        const { track, ellipse } = ensureTrack(i);
        const pos = fieldToThree(tr.x, tr.y, 0.04);
        track.group.position.copy(pos);

        const isTarget = (tr.id === targetId);
        track.sphere.material.color.set(isTarget ? 0xff6e40 : (tr.confirmed ? 0xffd54f : 0x888888));
        track.sphere.scale.setScalar(isTarget ? 1.3 : 1.0);

        // Label: id + "*" if target + "✓" if confirmed.
        const ctx2d = track.labelCanvas.getContext('2d');
        ctx2d.clearRect(0, 0, 128, 64);
        ctx2d.fillStyle = isTarget ? '#ff6e40' : (tr.confirmed ? '#ffd54f' : '#bbb');
        ctx2d.font = 'bold 28px monospace';
        ctx2d.textAlign = 'center';
        ctx2d.fillText('#' + tr.id + (isTarget ? '*' : ''), 64, 40);
        track.labelTex.needsUpdate = true;

        track.group.visible = true;

        // 2σ ellipse from 2x2 cov [a,b,c,d].
        const a = tr.cov[0], b = tr.cov[1], c = tr.cov[2], d = tr.cov[3];
        const e = eig2(a, b, c, d);
        // 2σ scaling for 2-dof 95% ~= sqrt(5.991); use 2σ for visibility.
        const k = 2.0;
        ellipse.position.copy(pos); ellipse.position.y = 0.003;
        // Rotate about Y (three.js up); eigenvector angle measured in field XY plane.
        ellipse.rotation.set(0, -e.angle, 0);
        ellipse.scale.set(k * Math.sqrt(e.l1), 1, k * Math.sqrt(e.l2));
        ellipse.material.color.set(isTarget ? 0xff6e40 : 0x80deea);
        ellipse.visible = true;
    }

    // Camera panel: position camPanelCamera from camera extrinsics.
    const [ox, oy, oz] = cam.origin;
    const R = cam.Rfc;
    // Columns of R_field_camera = camera axes in field frame.
    //   col0 = +X_cam (image right) = R[0..2]
    //   col1 = +Y_cam (image down)  = R[3..5]
    //   col2 = +Z_cam (forward)     = R[6..8]
    const fx = R[6], fy = R[7], fz = R[8];
    const uxF = -R[3], uyF = -R[4], uzF = -R[5];  // camera "up" in field = -Y_cam

    const camPos = fieldToThree(ox, oy, oz);
    const lookAtF = { x: ox + fx, y: oy + fy, z: oz + fz };
    const lookAt = fieldToThree(lookAtF.x, lookAtF.y, lookAtF.z);

    camPanelCamera.position.copy(camPos);
    // Three.js `up` vector in three.js coords (from field up).
    camPanelCamera.up.set(uyF, uzF, uxF);  // maps (fx_field, fy_field, fz_field) -> (y, z, x)
    camPanelCamera.lookAt(lookAt);

    camPanelRenderer.render(scene, camPanelCamera);

    // 2D pixel overlays over the panel.
    drawPixelOverlays(trackerState);
}

function drawPixelOverlays(state) {
    const cam = state.camera;
    const w = camPanelOverlayCanvas.clientWidth;
    const h = camPanelOverlayCanvas.clientHeight;
    if (!cam || !w || !h) return;
    const sx = w / cam.width;
    const sy = h / cam.height;

    // Intake mask band.
    const maskTop = cam.intakeMaskYMinFrac * cam.height * sy;
    overlayCtx.fillStyle = 'rgba(255, 0, 0, 0.18)';
    overlayCtx.fillRect(0, maskTop, w, h - maskTop);
    overlayCtx.strokeStyle = 'rgba(255, 80, 80, 0.55)';
    overlayCtx.lineWidth = 1;
    overlayCtx.strokeRect(0, maskTop, w, h - maskTop);

    // Raw detections (crosses).
    overlayCtx.strokeStyle = '#ff1744';
    overlayCtx.lineWidth = 2;
    for (const d of state.detectionsPx || []) {
        const x = d.u * sx; const y = d.v * sy;
        overlayCtx.beginPath();
        overlayCtx.moveTo(x - 6, y); overlayCtx.lineTo(x + 6, y);
        overlayCtx.moveTo(x, y - 6); overlayCtx.lineTo(x, y + 6);
        overlayCtx.stroke();
    }

    // Track re-projected covariance ellipses in pixel space.
    for (const tr of state.tracks || []) {
        if (tr.u == null || tr.v == null || !tr.covPx) continue;
        const x = tr.u * sx; const y = tr.v * sy;
        const e = eig2(tr.covPx[0], tr.covPx[1], tr.covPx[2], tr.covPx[3]);
        const k = 2.0;
        const rx = k * Math.sqrt(e.l1) * sx;
        const ry = k * Math.sqrt(e.l2) * sy;
        overlayCtx.save();
        overlayCtx.translate(x, y);
        overlayCtx.rotate(e.angle);
        overlayCtx.strokeStyle = (tr.id === state.targetId) ? '#ff6e40' : '#80deea';
        overlayCtx.lineWidth = 1.5;
        overlayCtx.beginPath();
        overlayCtx.ellipse(0, 0, Math.max(rx, 1), Math.max(ry, 1), 0, 0, 2 * Math.PI);
        overlayCtx.stroke();
        overlayCtx.restore();

        overlayCtx.fillStyle = (tr.id === state.targetId) ? '#ff6e40' : '#ffd54f';
        overlayCtx.beginPath();
        overlayCtx.arc(x, y, 3, 0, 2 * Math.PI);
        overlayCtx.fill();
    }

    // HUD text.
    overlayCtx.fillStyle = 'rgba(0,0,0,0.45)';
    overlayCtx.fillRect(0, 0, w, 22);
    overlayCtx.fillStyle = '#cfd8ff';
    overlayCtx.font = '12px monospace';
    overlayCtx.fillText(
        `scenario=${state.scenario}  rms=${state.rmsErrorM == null ? '—' : state.rmsErrorM.toFixed(3) + 'm'}` +
        `  tracks=${(state.tracks || []).length}  target=${state.targetId == null ? '—' : '#' + state.targetId}`,
        6, 15,
    );
}

// Resize handler
window.addEventListener('resize', () => {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
    if (lastCamera) sizeCameraPanel(lastCamera.width, lastCamera.height);
});

// Render loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();

function updateGoals(goals) {
    if (!goals) return;

    // Update lever tilts (rotation around X — outer end pushed down by robot)
    if (goals.redLeverAngle !== undefined) {
        redGoal.leverMesh.rotation.x = -goals.redLeverAngle;
    }
    if (goals.blueLeverAngle !== undefined) {
        blueGoal.leverMesh.rotation.x = goals.blueLeverAngle;
    }

    // Update score displays
    updateScoreSprite(redGoal, 'BLUE', goals.redScore || 0, 'rgba(40,40,180,0.8)');
    updateScoreSprite(blueGoal, 'RED', goals.blueScore || 0, 'rgba(180,40,40,0.8)');
}

export { updateRobot, updateBalls, updateGoals, updateShotViz, updateTracker };
