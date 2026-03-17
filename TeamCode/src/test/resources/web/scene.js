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

// Direction indicator on robot (yellow cone pointing forward)
const arrowGeo = new THREE.ConeGeometry(0.04, 0.1, 8);
const arrowMat = new THREE.MeshStandardMaterial({ color: 0xffcc00 });
const arrowMesh = new THREE.Mesh(arrowGeo, arrowMat);
arrowMesh.rotation.x = -Math.PI / 2;
arrowMesh.position.set(0, ROBOT_HEIGHT / 2, ROBOT_SIZE / 2 - 0.02);
robotMesh.add(arrowMesh);

// Shooter turret (~1/4 robot volume, sits on top of robot)
const SHOOTER_WIDTH = 0.2;
const SHOOTER_LENGTH = 0.2;
const SHOOTER_HEIGHT = 0.15;
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
shooterArrowMesh.rotation.x = -Math.PI / 2;
shooterArrowMesh.position.set(0, SHOOTER_HEIGHT / 2, SHOOTER_LENGTH / 2);
shooterGroup.add(shooterArrowMesh);

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
    const cX = -HF;
    const cZ = zSign * HF;
    const t = GOAL_WALL_THICK_VIS;

    const goalMat = new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.55, side: THREE.DoubleSide });
    const rampMat = new THREE.MeshStandardMaterial({ color: 0x999999, transparent: true, opacity: 0.7 });
    const gateMat = new THREE.MeshStandardMaterial({ color: 0xcccc00 });
    const leverMat = new THREE.MeshStandardMaterial({ color: 0x44cc44 });

    const goalOffset = CRAMP_WIDTH; // triangle inset from ±Z wall

    // --- Side wall A: along -X (back) wall, runs in Z direction ---
    // Tapered: lip height at front end, total height at corner end
    // Offset from wall by goalOffset
    const wallA = createTaperedWall(
        {x: cX, z: cZ - zSign * (goalOffset + GOAL_LEG)}, GOAL_LIP_HEIGHT,  // front (low)
        {x: cX, z: cZ - zSign * goalOffset}, GOAL_TOTAL_HEIGHT,              // corner (tall)
        t, 1, 0, goalMat  // thickness in +X direction (toward field)
    );
    group.add(wallA);

    // --- Side wall B: flush against ±Z (side) wall, runs in X direction ---
    const wallB = createTaperedWall(
        {x: cX + GOAL_LEG, z: cZ}, GOAL_LIP_HEIGHT,  // front (low)
        {x: cX, z: cZ}, GOAL_TOTAL_HEIGHT,             // corner (tall)
        t, 0, -zSign, goalMat  // thickness toward field center
    );
    group.add(wallB);

    // Corner connector: along -X wall, bridges wall B (at ±Z wall) to wall A (offset)
    const cornerConnGeo = new THREE.BoxGeometry(GOAL_WALL_THICK_VIS, GOAL_TOTAL_HEIGHT, goalOffset);
    const cornerConnMesh = new THREE.Mesh(cornerConnGeo, goalMat);
    cornerConnMesh.position.set(cX + GOAL_WALL_THICK_VIS / 2, GOAL_TOTAL_HEIGHT / 2, cZ - zSign * goalOffset / 2);
    group.add(cornerConnMesh);

    // --- Front wall (diagonal hypotenuse) at lip height ---
    const frontLen = GOAL_LEG * Math.SQRT2;
    const frontMidX = cX + GOAL_LEG / 2;
    const frontMidZ = cZ - zSign * (goalOffset + GOAL_LEG / 2);
    const frontAngleJS = -zSign * Math.PI / 4;

    const frontGeo = new THREE.BoxGeometry(frontLen, GOAL_LIP_HEIGHT, GOAL_WALL_THICK_VIS);
    const frontMesh = new THREE.Mesh(frontGeo, goalMat);
    frontMesh.position.set(frontMidX, GOAL_LIP_HEIGHT / 2, frontMidZ);
    frontMesh.rotation.y = frontAngleJS;
    group.add(frontMesh);

    // --- Rectangular extension + Classifier ramp ---
    // Ramp is flush against the ±Z wall. Triangle is offset inward by CRAMP_WIDTH.
    // A rectangular extension bridges the offset triangle to the wall-hugging ramp.
    const rampStartX = cX + GOAL_LEG;
    const rampEndX = rampStartX + CRAMP_LENGTH;
    const rampMidX = (rampStartX + rampEndX) / 2;
    const rampZ = cZ - zSign * CRAMP_WIDTH / 2; // flush against ±Z wall

    // Inner wall along ±Z wall covering ramp section (extends above field wall height)
    const rectInnerGeo = new THREE.BoxGeometry(CRAMP_LENGTH, GOAL_LIP_HEIGHT, GOAL_WALL_THICK_VIS);
    const rectInnerMesh = new THREE.Mesh(rectInnerGeo, goalMat);
    rectInnerMesh.position.set(rampMidX, GOAL_LIP_HEIGHT / 2, cZ - zSign * GOAL_WALL_THICK_VIS / 2);
    group.add(rectInnerMesh);

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
    rampFloorMesh.rotation.z = -CRAMP_SLOPE_ANGLE; // lifts -X end (input near goal)
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
    gateMesh.position.set(rampEndX, CRAMP_END_H / 2, rampZ);
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
    scoreSprite.position.set(cX + GOAL_LEG / 3, GOAL_TOTAL_HEIGHT + 0.15, cZ - zSign * GOAL_LEG / 3);
    scene.add(scoreSprite);

    return { group, gateMesh, leverMesh, scoreCanvas, scoreCtx, scoreTex, scoreSprite };
}

const redGoal = createGoalStructure(1, 0xcc3333);
const blueGoal = createGoalStructure(-1, 0x3333cc);

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

function updateRobot(x, y, theta, turretAngle) {
    // sim (x=forward, y=left) -> Three.js (x=-y, z=x), sim theta -> rotation about -Y
    robotMesh.position.set(-y, ROBOT_HEIGHT / 2, x);
    robotMesh.rotation.y = -theta;
    // Turret rotates about Y axis relative to robot
    if (turretAngle !== undefined) {
        shooterGroup.rotation.y = -turretAngle;
    }
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
        mesh.position.set(-b.y, b.z, b.x); // sim (x=fwd, y=left, z=up) -> Three.js (x=-y, y=z, z=x)
        mesh.material = b.color === 'green' ? greenMat : purpleMat;
    }
}

// Resize handler
window.addEventListener('resize', () => {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
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
    updateScoreSprite(redGoal, 'RED', goals.redScore || 0, 'rgba(180,40,40,0.8)');
    updateScoreSprite(blueGoal, 'BLUE', goals.blueScore || 0, 'rgba(40,40,180,0.8)');
}

export { updateRobot, updateBalls, updateGoals };
