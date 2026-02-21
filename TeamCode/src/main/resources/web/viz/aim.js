import * as THREE from 'three';
import { scene } from '../scene.js';

const toggleAimTurret = document.getElementById('toggle-aim-turret');
const toggleAimGoal = document.getElementById('toggle-aim-goal');
const toggleAimArc = document.getElementById('toggle-aim-arc');
const toggleAimLead = document.getElementById('toggle-aim-lead');

const hudAimLock = document.getElementById('hud-aim-lock');
const hudAimDist = document.getElementById('hud-aim-dist');
const hudAimTurret = document.getElementById('hud-aim-turret');
const hudAimFlywheel = document.getElementById('hud-aim-flywheel');
const hudAimVel = document.getElementById('hud-aim-vel');

// Goal marker (basket position)
const goalMarker = new THREE.Mesh(
    new THREE.CylinderGeometry(0.08, 0.08, 0.02, 24),
    new THREE.MeshBasicMaterial({ color: 0xff4444, transparent: true, opacity: 0.6 })
);
goalMarker.rotation.x = Math.PI / 2;
goalMarker.visible = false;
scene.add(goalMarker);

// Goal ring (pulsing targeting ring) — exported for animation loop
export const goalRingMat = new THREE.MeshBasicMaterial({
    color: 0xff6666, transparent: true, opacity: 0.4, side: THREE.DoubleSide, depthWrite: false
});
export const goalRing = new THREE.Mesh(new THREE.RingGeometry(0.12, 0.14, 32), goalRingMat);
goalRing.visible = false;
scene.add(goalRing);

const goalPillarGeo = new THREE.BufferGeometry();
goalPillarGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array([0, 0, 0, 0, 0, 1.0]), 3));
const goalPillar = new THREE.Line(goalPillarGeo, new THREE.LineBasicMaterial({ color: 0xff4444, transparent: true, opacity: 0.3 }));
goalPillar.visible = false;
scene.add(goalPillar);

// Turret aiming rays
const turretRay = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1.0, 0xff2222, 0.05, 0.025);
turretRay.visible = false;
scene.add(turretRay);

const turretTargetRay = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.8, 0xff8888, 0.04, 0.02);
turretTargetRay.visible = false;
scene.add(turretTargetRay);

// Projectile arc
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
        verts[i * 3] = p.x; verts[i * 3 + 1] = p.y; verts[i * 3 + 2] = p.z;
        const t = i / (arcPoints.length - 1);
        colors[i * 3] = 1.0; colors[i * 3 + 1] = 0.3 + 0.7 * t; colors[i * 3 + 2] = 0.1 * t;
    });
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    projectileArcLine = new THREE.Line(geom, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.8 }));
    scene.add(projectileArcLine);
}

// Lead compensation vector
const leadArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.1, 0xffaa00, 0.03, 0.015);
leadArrow.visible = false;
scene.add(leadArrow);

// Targeting line from robot to goal
let targetingLine = null;

function updateTargetingLine(base, aiming) {
    if (targetingLine) {
        scene.remove(targetingLine);
        targetingLine.geometry.dispose();
        targetingLine.material.dispose();
        targetingLine = null;
    }
    if (!aiming || !toggleAimGoal.checked) return;

    const robotX = base.x, robotY = base.y, robotZ = (base.z || 0) + 0.22;
    const segments = 20;
    const verts = [], colors = [];
    for (let i = 0; i < segments; i++) {
        if (i % 2 === 1) continue;
        const t0 = i / segments, t1 = (i + 1) / segments;
        verts.push(
            robotX + (aiming.goalX - robotX) * t0, robotY + (aiming.goalY - robotY) * t0, robotZ + (0.8 - robotZ) * t0,
            robotX + (aiming.goalX - robotX) * t1, robotY + (aiming.goalY - robotY) * t1, robotZ + (0.8 - robotZ) * t1
        );
        const locked = aiming.hasTarget;
        colors.push(locked ? 0 : 1, locked ? 1 : 0.3, 0.2, 0.8, 0.8, 0.8);
    }
    if (verts.length === 0) return;

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(verts), 3));
    geom.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
    targetingLine = new THREE.LineSegments(geom, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.5 }));
    scene.add(targetingLine);
}

export function updateAimingViz(aiming, base) {
    if (!aiming) {
        goalMarker.visible = false;
        goalRing.visible = false;
        goalPillar.visible = false;
        turretRay.visible = false;
        turretTargetRay.visible = false;
        leadArrow.visible = false;
        return;
    }

    if (toggleAimGoal.checked) {
        goalMarker.position.set(aiming.goalX, aiming.goalY, 0.8);
        goalMarker.visible = true;
        goalRing.position.set(aiming.goalX, aiming.goalY, 0.01);
        goalRingMat.opacity = aiming.hasTarget ? 0.3 + 0.3 * Math.sin(Date.now() * 0.003) : 0.15;
        goalRingMat.color.set(aiming.hasTarget ? 0x00ff44 : 0xff4444);
        goalRing.visible = true;
        goalPillar.position.set(aiming.goalX, aiming.goalY, 0);
        goalPillar.visible = true;
    } else {
        goalMarker.visible = false;
        goalRing.visible = false;
        goalPillar.visible = false;
    }

    if (toggleAimTurret.checked && base) {
        const robotZ = (base.z || 0) + 0.22;
        turretRay.position.set(base.x, base.y, robotZ);
        turretRay.setDirection(new THREE.Vector3(Math.cos(aiming.turretFieldYaw), Math.sin(aiming.turretFieldYaw), 0));
        const rayLen = Math.min(aiming.targetDistance * 0.8, 2.0);
        turretRay.setLength(rayLen, 0.05, 0.025);
        turretRay.setColor(aiming.hasTarget ? (aiming.usingPrediction ? 0xffaa00 : 0x00ff44) : 0xff4444);
        turretRay.visible = true;

        const targetYaw = base.yaw + aiming.turretTargetAngle;
        turretTargetRay.position.set(base.x, base.y, robotZ + 0.01);
        turretTargetRay.setDirection(new THREE.Vector3(Math.cos(targetYaw), Math.sin(targetYaw), 0));
        turretTargetRay.setLength(rayLen * 0.6, 0.04, 0.02);
        turretTargetRay.setColor(0x888888);
        turretTargetRay.visible = true;
    } else {
        turretRay.visible = false;
        turretTargetRay.visible = false;
    }

    updateProjectileArc(aiming.projectileArc);

    if (toggleAimLead.checked && base) {
        const speed = Math.sqrt(aiming.robotVx * aiming.robotVx + aiming.robotVy * aiming.robotVy);
        if (speed > 0.01) {
            leadArrow.position.set(base.x, base.y, (base.z || 0) + 0.25);
            leadArrow.setDirection(new THREE.Vector3(aiming.robotVx / speed, aiming.robotVy / speed, 0));
            leadArrow.setLength(speed * 0.3, 0.03, 0.015);
            leadArrow.setColor(0xffaa00);
            leadArrow.visible = true;
        } else {
            leadArrow.visible = false;
        }
    } else {
        leadArrow.visible = false;
    }

    updateTargetingLine(base, aiming);

    hudAimLock.textContent = aiming.hasTarget ? (aiming.usingPrediction ? 'PREDICT' : 'LOCKED') : 'NO TARGET';
    hudAimLock.className = 'hud-value' + (aiming.hasTarget ? (aiming.usingPrediction ? ' warning' : ' active') : ' error');
    hudAimDist.textContent = `${aiming.targetDistance.toFixed(2)}m`;
    hudAimTurret.textContent = `${(aiming.turretActualAngle * 180 / Math.PI).toFixed(1)}° / ${(aiming.turretTargetAngle * 180 / Math.PI).toFixed(1)}°`;
    const fwPct = aiming.flywheelTarget > 0 ? ((aiming.flywheelOmega / aiming.flywheelTarget) * 100).toFixed(0) : '0';
    hudAimFlywheel.textContent = `${aiming.flywheelOmega.toFixed(0)} rad/s (${fwPct}%)`;
    hudAimFlywheel.className = 'hud-value' + (aiming.flywheelReady ? ' active' : ' warning');
    hudAimVel.textContent = `${Math.sqrt(aiming.robotVx * aiming.robotVx + aiming.robotVy * aiming.robotVy).toFixed(2)} m/s`;
}
