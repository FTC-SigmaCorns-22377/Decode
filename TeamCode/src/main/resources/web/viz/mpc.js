import * as THREE from 'three';
import { scene } from '../scene.js';

const toggleMpcPredicted = document.getElementById('toggle-mpc-predicted');
const toggleMpcContours = document.getElementById('toggle-mpc-contours');
const toggleMpcHorizon = document.getElementById('toggle-mpc-horizon');

// MPC predicted trajectory line (green-yellow gradient)
let mpcPredictedLine = null;

export function updateMpcPredicted(points) {
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
        const t = i / (points.length - 1);
        colors[i * 3] = 0.2 + 0.8 * t;
        colors[i * 3 + 1] = 1.0;
        colors[i * 3 + 2] = 0.2 * (1 - t);
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    mpcPredictedLine = new THREE.Line(geometry, new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 2 }));
    scene.add(mpcPredictedLine);
}

const contourArrowPool = [];
const contourDotPool = [];
const contourVelPool = [];

export function updateMpcContours(contours) {
    contourArrowPool.forEach(a => { a.visible = false; });
    contourDotPool.forEach(d => { d.visible = false; });
    contourVelPool.forEach(v => { v.visible = false; });

    if (!contours || contours.length === 0 || !toggleMpcContours.checked) return;

    contours.forEach((c, i) => {
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

        let arrow = contourArrowPool[i];
        if (!arrow) {
            arrow = new THREE.ArrowHelper(
                new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.08, 0x00ddff, 0.03, 0.015
            );
            scene.add(arrow);
            contourArrowPool.push(arrow);
        }
        arrow.position.set(c.x, c.y, 0.04);
        arrow.setDirection(new THREE.Vector3(Math.cos(c.theta), Math.sin(c.theta), 0));
        arrow.setLength(0.08, 0.03, 0.015);
        arrow.visible = true;

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
            vel.position.set(c.x, c.y, 0.05);
            vel.setDirection(new THREE.Vector3(c.vx / speed, c.vy / speed, 0));
            const velLen = Math.min(speed * 0.1, 0.15);
            vel.setLength(velLen, velLen * 0.3, velLen * 0.15);
            vel.visible = true;
        }
    });
}

let mpcHorizonMesh = null;

export function updateMpcHorizon(horizon, predicted) {
    if (mpcHorizonMesh) {
        scene.remove(mpcHorizonMesh);
        mpcHorizonMesh.geometry.dispose();
        mpcHorizonMesh.material.dispose();
        mpcHorizonMesh = null;
    }
    if (!horizon || !toggleMpcHorizon.checked || !predicted || predicted.length < 2) return;

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

        const nx = -dy / len;
        const ny = dx / len;
        const dtMs = (knotTimes[i + 1] || 40) - (knotTimes[i] || 0);
        const width = 0.01 + dtMs * 0.0004;
        const tNorm = i / (predicted.length - 1);
        const g = 0.5 + 0.5 * (1 - tNorm);

        ribbonVerts.push(
            p0.x + nx * width, p0.y + ny * width, 0.02,
            p0.x - nx * width, p0.y - ny * width, 0.02,
            p1.x + nx * width, p1.y + ny * width, 0.02,
            p1.x - nx * width, p1.y - ny * width, 0.02,
            p1.x + nx * width, p1.y + ny * width, 0.02,
            p0.x - nx * width, p0.y - ny * width, 0.02
        );
        for (let j = 0; j < 6; j++) ribbonColors.push(0.0, g, 1.0);
    }

    if (ribbonVerts.length === 0) return;

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(ribbonVerts), 3));
    geom.setAttribute('color', new THREE.BufferAttribute(new Float32Array(ribbonColors), 3));
    mpcHorizonMesh = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({
        vertexColors: true, transparent: true, opacity: 0.25,
        side: THREE.DoubleSide, depthWrite: false
    }));
    scene.add(mpcHorizonMesh);
}
