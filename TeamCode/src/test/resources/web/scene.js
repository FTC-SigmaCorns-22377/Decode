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

// Ball pool
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

export { updateRobot, updateBalls };
