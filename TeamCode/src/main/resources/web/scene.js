import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import URDFLoader from 'urdf-loader';

export const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a2e);

export const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(2, 2, 2);
camera.up.set(0, 0, 1);

export const renderer = new THREE.WebGLRenderer({ antialias: true });
const container = document.getElementById('canvas-container');
renderer.setSize(container.clientWidth, container.clientHeight);
container.appendChild(renderer.domElement);

export const controls = new OrbitControls(camera, renderer.domElement);
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
const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(fieldSize, fieldSize),
    new THREE.MeshPhongMaterial({ color: 0x2a2a3a, side: THREE.DoubleSide })
);
scene.add(floor);

const wallMat = new THREE.MeshPhongMaterial({ color: 0xcccccc, transparent: true, opacity: 0.3 });
const sideWallGeo = new THREE.BoxGeometry(fieldSize, 0.02, wallHeight);
[[0, fieldSize / 2], [0, -fieldSize / 2]].forEach(pos => {
    const w = new THREE.Mesh(sideWallGeo, wallMat);
    w.position.set(pos[0], pos[1], wallHeight / 2);
    scene.add(w);
});
const sideWallGeo2 = new THREE.BoxGeometry(0.02, fieldSize, wallHeight);
[[fieldSize / 2, 0], [-fieldSize / 2, 0]].forEach(pos => {
    const w = new THREE.Mesh(sideWallGeo2, wallMat);
    w.position.set(pos[0], pos[1], wallHeight / 2);
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
    mesh.position.x += -1.643476;
    mesh.position.y += -0.013388;
    scene.add(mesh);
    const redRamp = mesh.clone();
    redRamp.material = new THREE.MeshPhongMaterial({ color: 0xff0000, side: THREE.DoubleSide });
    redRamp.scale.x = -0.001;
    redRamp.position.x = -mesh.position.x;
    scene.add(redRamp);
});

// Balls
const ballGeo = new THREE.SphereGeometry(0.0635, 12, 12);
export const ballMaterials = {
    green: new THREE.MeshPhongMaterial({ color: 0x00ff00 }),
    purple: new THREE.MeshPhongMaterial({ color: 0x9932cc }),
    orange: new THREE.MeshPhongMaterial({ color: 0xff8800 })
};
export const balls = [];

export function ensureBallCount(count) {
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

export const robotPoint = new THREE.Mesh(
    new THREE.SphereGeometry(0.04, 12, 12),
    new THREE.MeshPhongMaterial({ color: 0xffaa00 })
);
export const robotHeading = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.2, 0xffaa00);
robotPoint.visible = false;
robotHeading.visible = false;
scene.add(robotPoint);
scene.add(robotHeading);

export const wheelForceScale = 0.0025;
export const wheelForceMaxLen = 0.6;
export const wheelForceMinLen = 0.02;
export const wheelForceConfig = [
    { link: 'fl_wheel', color: 0xff0000 },
    { link: 'bl_wheel', color: 0x0000ff },
    { link: 'br_wheel', color: 0xffff00 },
    { link: 'fr_wheel', color: 0x00ff00 }
];
export const wheelForceArrows = new Map();
export const wheelForceTmp = new THREE.Vector3();

// robotRef allows async population from URDFLoader
export const robotRef = { value: null };

const loader = new URDFLoader();
loader.load('/robot.urdf', result => {
    robotRef.value = result;
    scene.add(result);
    wheelForceConfig.forEach(cfg => {
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), wheelForceMinLen, cfg.color);
        arrow.visible = false;
        scene.add(arrow);
        wheelForceArrows.set(cfg.link, arrow);
    });
});
