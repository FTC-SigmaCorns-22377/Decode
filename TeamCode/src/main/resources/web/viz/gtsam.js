import * as THREE from 'three';
import { scene } from '../scene.js';

const toggleGtsamFused = document.getElementById('toggle-gtsam-fused');
const toggleGtsamLandmarks = document.getElementById('toggle-gtsam-landmarks');
const toggleGtsamVision = document.getElementById('toggle-gtsam-vision');
const toggleGtsamUncertainty = document.getElementById('toggle-gtsam-uncertainty');
const toggleGtsamProjections = document.getElementById('toggle-gtsam-projections');
const toggleGtsamOdo = document.getElementById('toggle-gtsam-odo');
const toggleGtsamTrue = document.getElementById('toggle-gtsam-true');
const toggleCameraView = document.getElementById('toggle-camera-view');
const uncertaintyScaleSlider = document.getElementById('uncertainty-scale');

const hudGtsamPose = document.getElementById('hud-gtsam-pose');
const hudGtsamUncertainty = document.getElementById('hud-gtsam-uncertainty');
const hudGtsamVision = document.getElementById('hud-gtsam-vision');
const hudGtsamTags = document.getElementById('hud-gtsam-tags');
const hudGtsamMode = document.getElementById('hud-gtsam-mode');
const hudGtsamFusedErr = document.getElementById('hud-gtsam-fused-err');
const hudGtsamOdoErr = document.getElementById('hud-gtsam-odo-err');

const cameraViewContainer = document.getElementById('camera-view-container');
const cameraViewCanvas = document.getElementById('camera-view-canvas');
const cameraViewCtx = cameraViewCanvas.getContext('2d');

// Fused pose marker
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
        verts[i * 3] = p.x; verts[i * 3 + 1] = p.y; verts[i * 3 + 2] = 0.005;
        const t = i / (fusedTrail.length - 1);
        colors[i * 3] = 0; colors[i * 3 + 1] = t; colors[i * 3 + 2] = 0.5 * t;
    });
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    fusedTrailLine = new THREE.Line(geom, new THREE.LineBasicMaterial({ vertexColors: true }));
    scene.add(fusedTrailLine);
}

// Odometry marker
const odoMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.025, 12, 12),
    new THREE.MeshBasicMaterial({ color: 0xff6600 })
);
odoMarker.visible = false;
scene.add(odoMarker);

const odoHeading = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.12, 0xff6600, 0.035, 0.018
);
odoHeading.visible = false;
scene.add(odoHeading);

const ODO_TRAIL_MAX = 300;
const odoTrail = [];
let odoTrailLine = null;

function updateOdoTrail(x, y) {
    odoTrail.push({ x, y });
    if (odoTrail.length > ODO_TRAIL_MAX) odoTrail.shift();

    if (odoTrailLine) {
        scene.remove(odoTrailLine);
        odoTrailLine.geometry.dispose();
        odoTrailLine.material.dispose();
    }
    if (odoTrail.length < 2 || !toggleGtsamOdo.checked) return;

    const verts = new Float32Array(odoTrail.length * 3);
    const colors = new Float32Array(odoTrail.length * 3);
    odoTrail.forEach((p, i) => {
        verts[i * 3] = p.x; verts[i * 3 + 1] = p.y; verts[i * 3 + 2] = 0.004;
        const t = i / (odoTrail.length - 1);
        colors[i * 3] = t; colors[i * 3 + 1] = 0.4 * t; colors[i * 3 + 2] = 0;
    });
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    odoTrailLine = new THREE.Line(geom, new THREE.LineBasicMaterial({ vertexColors: true }));
    scene.add(odoTrailLine);
}

// True position marker
const trueMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.025, 12, 12),
    new THREE.MeshBasicMaterial({ color: 0xffff00 })
);
trueMarker.visible = false;
scene.add(trueMarker);

const trueHeading = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.12, 0xffff00, 0.035, 0.018
);
trueHeading.visible = false;
scene.add(trueHeading);

const TRUE_TRAIL_MAX = 300;
const trueTrail = [];
let trueTrailLine = null;

function updateTrueTrail(x, y) {
    trueTrail.push({ x, y });
    if (trueTrail.length > TRUE_TRAIL_MAX) trueTrail.shift();

    if (trueTrailLine) {
        scene.remove(trueTrailLine);
        trueTrailLine.geometry.dispose();
        trueTrailLine.material.dispose();
    }
    if (trueTrail.length < 2 || !toggleGtsamTrue.checked) return;

    const verts = new Float32Array(trueTrail.length * 3);
    const colors = new Float32Array(trueTrail.length * 3);
    trueTrail.forEach((p, i) => {
        verts[i * 3] = p.x; verts[i * 3 + 1] = p.y; verts[i * 3 + 2] = 0.003;
        const t = i / (trueTrail.length - 1);
        colors[i * 3] = t; colors[i * 3 + 1] = t; colors[i * 3 + 2] = 0;
    });
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    trueTrailLine = new THREE.Line(geom, new THREE.LineBasicMaterial({ vertexColors: true }));
    scene.add(trueTrailLine);
}

// Covariance ellipse constants
const COV_ELLIPSE_SEGMENTS = 64;
const COV_ELLIPSE_SIGMA = 2.0;

// Odometry covariance ellipse
const odoCovEllipseGeo = new THREE.BufferGeometry();
odoCovEllipseGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array((COV_ELLIPSE_SEGMENTS + 1) * 3), 3));
const odoCovEllipseMat = new THREE.LineBasicMaterial({ color: 0xff6600, transparent: true, opacity: 0.5, depthWrite: false });
const odoCovEllipse = new THREE.LineLoop(odoCovEllipseGeo, odoCovEllipseMat);
odoCovEllipse.visible = false;
scene.add(odoCovEllipse);

const odoCovFillGeo = new THREE.BufferGeometry();
const odoCovFillMat = new THREE.MeshBasicMaterial({ color: 0xff6600, transparent: true, opacity: 0.1, side: THREE.DoubleSide, depthWrite: false });
const odoCovFill = new THREE.Mesh(odoCovFillGeo, odoCovFillMat);
odoCovFill.visible = false;
scene.add(odoCovFill);

function updateOdoCovEllipse(cx, cy, odoCovXX, odoCovYY, scaleFactor) {
    scaleFactor = scaleFactor || 1;
    const sxx = odoCovXX * scaleFactor, syy = odoCovYY * scaleFactor;
    if (sxx <= 0 || syy <= 0) { odoCovEllipse.visible = false; odoCovFill.visible = false; return; }
    const sx = COV_ELLIPSE_SIGMA * Math.sqrt(sxx);
    const sy = COV_ELLIPSE_SIGMA * Math.sqrt(syy);

    const positions = odoCovEllipseGeo.attributes.position.array;
    for (let i = 0; i <= COV_ELLIPSE_SEGMENTS; i++) {
        const t = (i / COV_ELLIPSE_SEGMENTS) * 2 * Math.PI;
        positions[i * 3] = cx + sx * Math.cos(t);
        positions[i * 3 + 1] = cy + sy * Math.sin(t);
        positions[i * 3 + 2] = 0.008;
    }
    odoCovEllipseGeo.attributes.position.needsUpdate = true;

    const nVerts = COV_ELLIPSE_SEGMENTS + 2;
    const fillPositions = new Float32Array(nVerts * 3);
    const fillIndices = [];
    fillPositions[0] = cx; fillPositions[1] = cy; fillPositions[2] = 0.008;
    for (let i = 0; i <= COV_ELLIPSE_SEGMENTS; i++) {
        const t = (i / COV_ELLIPSE_SEGMENTS) * 2 * Math.PI;
        const vi = (i + 1) * 3;
        fillPositions[vi] = cx + sx * Math.cos(t);
        fillPositions[vi + 1] = cy + sy * Math.sin(t);
        fillPositions[vi + 2] = 0.008;
        if (i > 0) fillIndices.push(0, i, i + 1);
    }
    odoCovFillGeo.setAttribute('position', new THREE.BufferAttribute(fillPositions, 3));
    odoCovFillGeo.setIndex(fillIndices);
    odoCovFillGeo.computeVertexNormals();
    odoCovEllipse.visible = true;
    odoCovFill.visible = true;
}

// GTSAM covariance ellipse
const covEllipseGeo = new THREE.BufferGeometry();
covEllipseGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array((COV_ELLIPSE_SEGMENTS + 1) * 3), 3));
const covEllipseMat = new THREE.LineBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.6, depthWrite: false });
const covEllipse = new THREE.LineLoop(covEllipseGeo, covEllipseMat);
covEllipse.visible = false;
scene.add(covEllipse);

const covEllipseFillGeo = new THREE.BufferGeometry();
const covEllipseFillMat = new THREE.MeshBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.15, side: THREE.DoubleSide, depthWrite: false });
const covEllipseFill = new THREE.Mesh(covEllipseFillGeo, covEllipseFillMat);
covEllipseFill.visible = false;
scene.add(covEllipseFill);

function updateCovEllipse(cx, cy, covXX, covXY, covYX, covYY, scaleFactor) {
    scaleFactor = scaleFactor || 1;
    const a = covXX * scaleFactor, b = ((covXY + covYX) / 2) * scaleFactor, d = covYY * scaleFactor;
    const trace = a + d;
    const det = a * d - b * b;
    const disc = Math.sqrt(Math.max(0, trace * trace / 4 - det));
    const lambda1 = trace / 2 + disc;
    const lambda2 = trace / 2 - disc;

    if (lambda1 <= 0 || lambda2 <= 0) { covEllipse.visible = false; covEllipseFill.visible = false; return; }

    const sx = COV_ELLIPSE_SIGMA * Math.sqrt(lambda1);
    const sy = COV_ELLIPSE_SIGMA * Math.sqrt(lambda2);
    const angle = (Math.abs(b) < 1e-12) ? (a >= d ? 0 : Math.PI / 2) : Math.atan2(lambda1 - a, b);
    const cosA = Math.cos(angle), sinA = Math.sin(angle);

    const positions = covEllipseGeo.attributes.position.array;
    for (let i = 0; i <= COV_ELLIPSE_SEGMENTS; i++) {
        const t = (i / COV_ELLIPSE_SEGMENTS) * 2 * Math.PI;
        const ex = sx * Math.cos(t), ey = sy * Math.sin(t);
        positions[i * 3] = cx + cosA * ex - sinA * ey;
        positions[i * 3 + 1] = cy + sinA * ex + cosA * ey;
        positions[i * 3 + 2] = 0.01;
    }
    covEllipseGeo.attributes.position.needsUpdate = true;

    const nVerts = COV_ELLIPSE_SEGMENTS + 2;
    const fillPositions = new Float32Array(nVerts * 3);
    const fillIndices = [];
    fillPositions[0] = cx; fillPositions[1] = cy; fillPositions[2] = 0.01;
    for (let i = 0; i <= COV_ELLIPSE_SEGMENTS; i++) {
        const t = (i / COV_ELLIPSE_SEGMENTS) * 2 * Math.PI;
        const ex = sx * Math.cos(t), ey = sy * Math.sin(t);
        const vi = (i + 1) * 3;
        fillPositions[vi] = cx + cosA * ex - sinA * ey;
        fillPositions[vi + 1] = cy + sinA * ex + cosA * ey;
        fillPositions[vi + 2] = 0.01;
        if (i > 0) fillIndices.push(0, i, i + 1);
    }
    covEllipseFillGeo.setAttribute('position', new THREE.BufferAttribute(fillPositions, 3));
    covEllipseFillGeo.setIndex(fillIndices);
    covEllipseFillGeo.computeVertexNormals();

    const totalVar = Math.sqrt(covXX + covYY);
    const u = Math.min(totalVar / 0.3, 1.0);
    covEllipseMat.color.setRGB(Math.min(u * 2, 1), Math.min((1 - u) * 2, 1), 0.1);
    covEllipseFillMat.color.setRGB(Math.min(u * 2, 1), Math.min((1 - u) * 2, 1), 0.1);
    covEllipseMat.opacity = 0.4 + u * 0.4;
    covEllipseFillMat.opacity = 0.08 + u * 0.15;
    covEllipse.visible = true;
    covEllipseFill.visible = true;
}

// Landmark markers
const landmarkGroup = new THREE.Group();
landmarkGroup.name = 'landmarks';
scene.add(landmarkGroup);
let landmarksCreated = false;

function createLandmarkMarkers(landmarks) {
    while (landmarkGroup.children.length > 0) {
        const child = landmarkGroup.children[0];
        landmarkGroup.remove(child);
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
    }
    landmarks.forEach(lm => {
        const tagGeo = new THREE.PlaneGeometry(0.165, 0.165);
        const tagMesh = new THREE.Mesh(tagGeo, new THREE.MeshBasicMaterial({
            color: 0xff8800, transparent: true, opacity: 0.6, side: THREE.DoubleSide, depthWrite: false
        }));
        tagMesh.position.set(lm.x, lm.y, lm.z);
        // Aerospace convention: roll = X-rotation, pitch = Y-rotation, yaw = Z-rotation.
        // THREE.js Euler with order 'ZYX' applies Rz(z) * Ry(y) * Rx(x),
        // so set(x=roll, y=pitch, z=yaw).
        tagMesh.rotation.order = 'ZYX';
        tagMesh.rotation.set(lm.roll || 0, lm.pitch || 0, lm.yaw || 0);
        tagMesh.userData = { tagId: lm.tagId };
        landmarkGroup.add(tagMesh);

        const border = new THREE.LineSegments(new THREE.EdgesGeometry(tagGeo), new THREE.LineBasicMaterial({ color: 0xffaa00 }));
        border.position.copy(tagMesh.position);
        border.rotation.copy(tagMesh.rotation);
        landmarkGroup.add(border);

        const pillarGeo = new THREE.BufferGeometry();
        pillarGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array([lm.x, lm.y, 0, lm.x, lm.y, lm.z]), 3));
        landmarkGroup.add(new THREE.Line(pillarGeo, new THREE.LineBasicMaterial({ color: 0x664400, transparent: true, opacity: 0.3 })));
    });
    landmarksCreated = true;
}

// Vision rays
const visionRayGroup = new THREE.Group();
visionRayGroup.name = 'visionRays';
scene.add(visionRayGroup);

function updateVisionRays(gtsam, base) {
    while (visionRayGroup.children.length > 0) {
        const child = visionRayGroup.children[0];
        visionRayGroup.remove(child);
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
    }
    if (!gtsam || !toggleGtsamVision.checked || !gtsam.detectedTags || gtsam.detectedTags.length === 0) return;

    const robotX = base.x, robotY = base.y, robotZ = (base.z || 0) + 0.22;
    gtsam.detectedTags.forEach(tagId => {
        const lm = (gtsam.landmarks || []).find(l => l.tagId === tagId);
        if (!lm) return;

        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array([robotX, robotY, robotZ, lm.x, lm.y, lm.z]), 3));
        geom.setAttribute('color', new THREE.BufferAttribute(new Float32Array([0, 1, 0.5, 1, 0.5, 0]), 3));
        visionRayGroup.add(new THREE.Line(geom, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.7 })));

        const ring = new THREE.Mesh(new THREE.RingGeometry(0.1, 0.12, 16), new THREE.MeshBasicMaterial({
            color: 0x00ff44, transparent: true, opacity: 0.6, side: THREE.DoubleSide, depthWrite: false
        }));
        ring.position.set(lm.x, lm.y, lm.z);
        ring.rotation.order = 'ZYX';
        ring.rotation.set(lm.roll || 0, lm.pitch || 0, lm.yaw || 0);
        visionRayGroup.add(ring);
    });
}

// Tag projection visualization
const tagProjectionGroup = new THREE.Group();
tagProjectionGroup.name = 'tagProjections';
scene.add(tagProjectionGroup);

export function updateTagProjections(simVision) {
    while (tagProjectionGroup.children.length > 0) {
        const child = tagProjectionGroup.children[0];
        tagProjectionGroup.remove(child);
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
    }
    if (!simVision || !toggleGtsamProjections.checked || !simVision.detections || simVision.detections.length === 0) return;

    const camX = simVision.cameraX, camY = simVision.cameraY, camZ = simVision.cameraZ;
    const camMarker = new THREE.Mesh(new THREE.SphereGeometry(0.015, 8, 8), new THREE.MeshBasicMaterial({ color: 0x00ddff }));
    camMarker.position.set(camX, camY, camZ);
    tagProjectionGroup.add(camMarker);

    simVision.detections.forEach(det => {
        const corners = det.corners;
        if (!corners || corners.length < 4) return;

        corners.forEach(corner => {
            const dot = new THREE.Mesh(new THREE.SphereGeometry(0.01, 6, 6), new THREE.MeshBasicMaterial({ color: 0x00ffaa }));
            dot.position.set(corner.x, corner.y, corner.z);
            tagProjectionGroup.add(dot);

            const lineGeo = new THREE.BufferGeometry();
            lineGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array([camX, camY, camZ, corner.x, corner.y, corner.z]), 3));
            lineGeo.setAttribute('color', new THREE.BufferAttribute(new Float32Array([0.0, 0.9, 1.0, 0.0, 1.0, 0.6]), 3));
            tagProjectionGroup.add(new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.5 })));
        });

        const outlineVerts = new Float32Array(5 * 3);
        const outlineColors = new Float32Array(5 * 3);
        for (let i = 0; i < 5; i++) {
            const c = corners[i % 4];
            outlineVerts[i * 3] = c.x; outlineVerts[i * 3 + 1] = c.y; outlineVerts[i * 3 + 2] = c.z;
            outlineColors[i * 3] = 0.0; outlineColors[i * 3 + 1] = 1.0; outlineColors[i * 3 + 2] = 0.5;
        }
        const outlineGeo = new THREE.BufferGeometry();
        outlineGeo.setAttribute('position', new THREE.BufferAttribute(outlineVerts, 3));
        outlineGeo.setAttribute('color', new THREE.BufferAttribute(outlineColors, 3));
        tagProjectionGroup.add(new THREE.Line(outlineGeo, new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 2 })));
    });
}

// Camera view (2D canvas overlay)
const TAG_COLORS = ['#00ff88', '#ff8800', '#00ddff', '#ff44ff'];

export function updateCameraView(simVision) {
    if (!toggleCameraView.checked) { cameraViewContainer.style.display = 'none'; return; }
    cameraViewContainer.style.display = 'block';

    const imgW = (simVision && simVision.imageWidth) || 640;
    const imgH = (simVision && simVision.imageHeight) || 480;
    const displayScale = 0.5;
    const cw = imgW * displayScale, ch = imgH * displayScale;
    if (cameraViewCanvas.width !== cw || cameraViewCanvas.height !== ch) {
        cameraViewCanvas.width = cw;
        cameraViewCanvas.height = ch;
    }

    const ctx = cameraViewCtx;
    ctx.fillStyle = '#111';
    ctx.fillRect(0, 0, cw, ch);
    ctx.strokeStyle = 'rgba(255,255,255,0.15)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cw / 2, 0); ctx.lineTo(cw / 2, ch);
    ctx.moveTo(0, ch / 2); ctx.lineTo(cw, ch / 2);
    ctx.stroke();

    if (!simVision || !simVision.detections || simVision.detections.length === 0) {
        ctx.fillStyle = '#555';
        ctx.font = '12px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('NO TAGS DETECTED', cw / 2, ch / 2);
        return;
    }

    simVision.detections.forEach((det, di) => {
        const pc = det.pixelCorners;
        if (!pc || pc.length < 4) return;
        const color = TAG_COLORS[di % TAG_COLORS.length];
        const r = parseInt(color.slice(1, 3), 16);
        const g = parseInt(color.slice(3, 5), 16);
        const b = parseInt(color.slice(5, 7), 16);

        ctx.fillStyle = `rgba(${r},${g},${b},0.15)`;
        ctx.beginPath();
        ctx.moveTo(pc[0].u * displayScale, pc[0].v * displayScale);
        for (let i = 1; i < 4; i++) ctx.lineTo(pc[i].u * displayScale, pc[i].v * displayScale);
        ctx.closePath();
        ctx.fill();

        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(pc[0].u * displayScale, pc[0].v * displayScale);
        for (let i = 1; i < 4; i++) ctx.lineTo(pc[i].u * displayScale, pc[i].v * displayScale);
        ctx.closePath();
        ctx.stroke();

        pc.forEach((p, ci) => {
            ctx.fillStyle = ci === 0 ? '#ff0000' : color;
            ctx.beginPath();
            ctx.arc(p.u * displayScale, p.v * displayScale, 3, 0, Math.PI * 2);
            ctx.fill();
        });

        ctx.fillStyle = color;
        ctx.font = 'bold 11px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`ID:${det.tagId}`,
            Math.min(pc[0].u, pc[1].u, pc[2].u, pc[3].u) * displayScale,
            Math.min(pc[0].v, pc[1].v, pc[2].v, pc[3].v) * displayScale - 4
        );
    });

    ctx.fillStyle = '#00ddff';
    ctx.font = '10px monospace';
    ctx.textAlign = 'right';
    ctx.fillText(`${simVision.detections.length} tag(s) | ${imgW}x${imgH}`, cw - 4, ch - 4);
}

export function updateGtsamViz(gtsam, base) {
    if (!gtsam) {
        gtsamFusedMarker.visible = false;
        gtsamFusedHeading.visible = false;
        covEllipse.visible = false;
        covEllipseFill.visible = false;
        odoMarker.visible = false;
        odoHeading.visible = false;
        odoCovEllipse.visible = false;
        odoCovFill.visible = false;
        trueMarker.visible = false;
        trueHeading.visible = false;
        return;
    }

    const scale = parseFloat(uncertaintyScaleSlider.value) || 1;

    if (toggleGtsamFused.checked) {
        gtsamFusedMarker.position.set(gtsam.fusedX, gtsam.fusedY, 0.025);
        gtsamFusedMarker.visible = true;
        gtsamFusedHeading.position.set(gtsam.fusedX, gtsam.fusedY, 0.025);
        gtsamFusedHeading.setDirection(new THREE.Vector3(Math.cos(gtsam.fusedTheta), Math.sin(gtsam.fusedTheta), 0));
        gtsamFusedHeading.setLength(0.15, 0.04, 0.02);
        gtsamFusedHeading.visible = true;
        updateFusedTrail(gtsam.fusedX, gtsam.fusedY);
    } else {
        gtsamFusedMarker.visible = false;
        gtsamFusedHeading.visible = false;
    }

    if (toggleGtsamOdo.checked) {
        odoMarker.position.set(gtsam.odoX || 0, gtsam.odoY || 0, 0.02);
        odoMarker.visible = true;
        odoHeading.position.set(gtsam.odoX || 0, gtsam.odoY || 0, 0.02);
        odoHeading.setDirection(new THREE.Vector3(Math.cos(gtsam.odoTheta || 0), Math.sin(gtsam.odoTheta || 0), 0));
        odoHeading.setLength(0.12, 0.035, 0.018);
        odoHeading.visible = true;
        updateOdoTrail(gtsam.odoX || 0, gtsam.odoY || 0);

        const hasOdoCov = (gtsam.odoCovXX || 0) + (gtsam.odoCovYY || 0) > 1e-10;
        if (toggleGtsamUncertainty.checked && hasOdoCov) {
            updateOdoCovEllipse(gtsam.odoX || 0, gtsam.odoY || 0, gtsam.odoCovXX, gtsam.odoCovYY, scale);
        } else {
            odoCovEllipse.visible = false;
            odoCovFill.visible = false;
        }
    } else {
        odoMarker.visible = false;
        odoHeading.visible = false;
        odoCovEllipse.visible = false;
        odoCovFill.visible = false;
    }

    const hasCov = (gtsam.covXX || 0) + (gtsam.covYY || 0) > 1e-10;
    if (toggleGtsamUncertainty.checked && hasCov) {
        updateCovEllipse(gtsam.fusedX, gtsam.fusedY, gtsam.covXX, gtsam.covXY || 0, gtsam.covYX || 0, gtsam.covYY, scale);
    } else {
        covEllipse.visible = false;
        covEllipseFill.visible = false;
    }

    if (toggleGtsamLandmarks.checked) {
        if (!landmarksCreated && gtsam.landmarks && gtsam.landmarks.length > 0) {
            createLandmarkMarkers(gtsam.landmarks);
        }
        landmarkGroup.visible = true;
    } else {
        landmarkGroup.visible = false;
    }

    // True position marker
    if (toggleGtsamTrue.checked) {
        const tx = gtsam.trueX || 0, ty = gtsam.trueY || 0, tth = gtsam.trueTheta || 0;
        trueMarker.position.set(tx, ty, 0.02);
        trueMarker.visible = true;
        trueHeading.position.set(tx, ty, 0.02);
        trueHeading.setDirection(new THREE.Vector3(Math.cos(tth), Math.sin(tth), 0));
        trueHeading.setLength(0.12, 0.035, 0.018);
        trueHeading.visible = true;
        updateTrueTrail(tx, ty);
    } else {
        trueMarker.visible = false;
        trueHeading.visible = false;
    }

    updateVisionRays(gtsam, base);

    hudGtsamPose.textContent = `(${gtsam.fusedX.toFixed(3)}, ${gtsam.fusedY.toFixed(3)}, ${(gtsam.fusedTheta * 180 / Math.PI).toFixed(1)}°)`;
    const posStdDev = Math.sqrt((gtsam.covXX || 0) + (gtsam.covYY || 0));
    hudGtsamUncertainty.textContent = (posStdDev * 100).toFixed(1) + 'cm';
    hudGtsamUncertainty.className = 'hud-value' + (posStdDev < 0.03 ? ' active' : posStdDev < 0.1 ? ' warning' : ' error');
    hudGtsamVision.textContent = gtsam.hasVision ? 'TRACKING' : 'NO VISION';
    hudGtsamVision.className = 'hud-value' + (gtsam.hasVision ? ' active' : ' warning');
    hudGtsamTags.textContent = (gtsam.detectedTags || []).join(', ') || 'none';
    hudGtsamMode.textContent = gtsam.usingPrediction ? 'PREDICTION' : (gtsam.initialized ? 'FUSED' : 'INIT...');
    hudGtsamMode.className = 'hud-value' + (gtsam.usingPrediction ? ' warning' : (gtsam.initialized ? ' active' : ''));

    // Compute errors vs true position
    const hasTruePos = (gtsam.trueX != null) && (gtsam.trueY != null);
    if (hasTruePos) {
        const fusedErrDist = Math.sqrt((gtsam.fusedX - gtsam.trueX) ** 2 + (gtsam.fusedY - gtsam.trueY) ** 2);
        const fusedErrTheta = Math.abs(normalizeAngle(gtsam.fusedTheta - gtsam.trueTheta)) * 180 / Math.PI;
        hudGtsamFusedErr.textContent = `${(fusedErrDist * 100).toFixed(1)}cm / ${fusedErrTheta.toFixed(1)}°`;
        hudGtsamFusedErr.className = 'hud-value' + (fusedErrDist < 0.03 ? ' active' : fusedErrDist < 0.1 ? ' warning' : ' error');

        const odoErrDist = Math.sqrt(((gtsam.odoX || 0) - gtsam.trueX) ** 2 + ((gtsam.odoY || 0) - gtsam.trueY) ** 2);
        const odoErrTheta = Math.abs(normalizeAngle((gtsam.odoTheta || 0) - gtsam.trueTheta)) * 180 / Math.PI;
        hudGtsamOdoErr.textContent = `${(odoErrDist * 100).toFixed(1)}cm / ${odoErrTheta.toFixed(1)}°`;
        hudGtsamOdoErr.className = 'hud-value' + (odoErrDist < 0.03 ? ' active' : odoErrDist < 0.1 ? ' warning' : ' error');
    }
}

function normalizeAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
}
