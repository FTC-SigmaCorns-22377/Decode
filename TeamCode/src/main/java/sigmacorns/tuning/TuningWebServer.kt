package sigmacorns.tuning

import com.google.gson.Gson
import fi.iki.elonen.NanoHTTPD

/**
 * Web server for shot tuning interface.
 * Provides interactive graphs for distance-speed curve editing and real-time velocity monitoring.
 */
class TuningWebServer(
    port: Int = 8082,
    private val stateProvider: () -> TuningState,
    private val velocityHistoryProvider: () -> List<VelocitySnapshot>,
    private val onShoot: () -> Unit,
    private val onInertiaChange: (Double) -> Unit,
    private val dataStore: ShotDataStore,
    private val tuner: AdaptiveTuner
) : NanoHTTPD(port) {

    private val gson = Gson()

    data class TuningState(
        val hasTarget: Boolean,
        val distance: Double,
        val currentFlywheelSpeed: Double,
        val targetFlywheelSpeed: Double,
        val turretAligned: Boolean,
        val readyToShoot: Boolean,
        val inertia: Double,
        val spindexerState: String
    )

    data class VelocitySnapshot(
        val time: Double,   // seconds since start
        val target: Double, // rad/s
        val actual: Double  // rad/s
    )

    override fun serve(session: IHTTPSession): Response {
        val uri = session.uri
        val method = session.method

        return try {
            when {
                uri == "/" || uri == "/index.html" -> serveMainPage()
                uri == "/api/status" -> serveStatus()
                uri == "/api/points" && method == Method.GET -> servePoints()
                uri == "/api/points" && method == Method.POST -> handleAddPoint(session)
                uri.startsWith("/api/points/") && method == Method.PUT -> handleUpdatePoint(session, uri)
                uri.startsWith("/api/points/") && method == Method.DELETE -> handleDeletePoint(uri)
                uri == "/api/shoot" && method == Method.POST -> handleShoot()
                uri == "/api/velocity-history" -> serveVelocityHistory()
                uri == "/api/config" && method == Method.POST -> handleConfigUpdate(session)
                uri == "/api/interpolated" -> serveInterpolatedCurve()
                else -> newFixedLengthResponse(Response.Status.NOT_FOUND, MIME_PLAINTEXT, "Not Found")
            }
        } catch (e: Exception) {
            e.printStackTrace()
            newFixedLengthResponse(Response.Status.INTERNAL_ERROR, MIME_PLAINTEXT, "Error: ${e.message}")
        }
    }

    private fun serveMainPage(): Response {
        return newFixedLengthResponse(Response.Status.OK, "text/html", HTML_PAGE)
    }

    private fun serveStatus(): Response {
        return newFixedLengthResponse(Response.Status.OK, "application/json", gson.toJson(stateProvider()))
    }

    private fun servePoints(): Response {
        return newFixedLengthResponse(Response.Status.OK, "application/json", dataStore.exportPointsJson())
    }

    private fun handleAddPoint(session: IHTTPSession): Response {
        val body = readBody(session)
        val point = gson.fromJson(body, SpeedPoint::class.java)
        tuner.addPoint(point.distance, point.speed)
        dataStore.save()
        return jsonOk("""{"success":true}""")
    }

    private fun handleUpdatePoint(session: IHTTPSession, uri: String): Response {
        val index = uri.removePrefix("/api/points/").toIntOrNull()
            ?: return jsonError("Invalid index")
        val body = readBody(session)
        val point = gson.fromJson(body, SpeedPoint::class.java)
        tuner.updatePoint(index, point.distance, point.speed)
        dataStore.save()
        return jsonOk("""{"success":true}""")
    }

    private fun handleDeletePoint(uri: String): Response {
        val index = uri.removePrefix("/api/points/").toIntOrNull()
            ?: return jsonError("Invalid index")
        tuner.removePoint(index)
        dataStore.save()
        return jsonOk("""{"success":true}""")
    }

    private fun handleShoot(): Response {
        onShoot()
        return jsonOk("""{"success":true}""")
    }

    private fun serveVelocityHistory(): Response {
        return newFixedLengthResponse(
            Response.Status.OK, "application/json",
            gson.toJson(velocityHistoryProvider())
        )
    }

    private fun handleConfigUpdate(session: IHTTPSession): Response {
        val body = readBody(session)
        val config = gson.fromJson(body, ConfigUpdate::class.java)
        if (config.inertia != null) {
            onInertiaChange(config.inertia)
        }
        return jsonOk("""{"success":true}""")
    }

    private fun serveInterpolatedCurve(): Response {
        val points = tuner.getPointsSorted()
        if (points.size < 2) {
            return newFixedLengthResponse(Response.Status.OK, "application/json", "[]")
        }
        val minDist = points.first().distance - 0.5
        val maxDist = points.last().distance + 0.5
        val step = (maxDist - minDist) / 50
        val curve = (0..50).mapNotNull { i ->
            val d = minDist + i * step
            tuner.getRecommendedSpeed(d)?.let { SpeedPoint(d, it) }
        }
        return newFixedLengthResponse(Response.Status.OK, "application/json", gson.toJson(curve))
    }

    private fun readBody(session: IHTTPSession): String {
        val contentLength = session.headers["content-length"]?.toIntOrNull() ?: 0
        val body = ByteArray(contentLength)
        session.inputStream.read(body, 0, contentLength)
        return String(body)
    }

    private fun jsonOk(json: String): Response {
        return newFixedLengthResponse(Response.Status.OK, "application/json", json)
    }

    private fun jsonError(msg: String): Response {
        return newFixedLengthResponse(Response.Status.BAD_REQUEST, "application/json", """{"error":"$msg"}""")
    }

    private data class ConfigUpdate(val inertia: Double?)

    companion object {
        private val HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Shot Tuning</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            max-width: 700px;
            margin: 0 auto;
            padding: 12px;
            background: #1a1a2e;
            color: #eee;
        }
        h1 { text-align: center; font-size: 22px; margin-bottom: 12px; }
        h3 { margin: 14px 0 6px 0; font-size: 13px; color: #888; text-transform: uppercase; }
        .panel {
            background: #16213e;
            padding: 10px;
            border-radius: 8px;
            margin-bottom: 10px;
        }
        .row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin: 4px 0;
            font-size: 13px;
        }
        .val { font-family: monospace; font-weight: bold; }
        .indicator {
            width: 8px; height: 8px; border-radius: 50%;
            display: inline-block; margin-right: 5px;
        }
        .on { background: #2ecc71; box-shadow: 0 0 6px #2ecc71; }
        .off { background: #e74c3c; }
        canvas {
            width: 100%;
            border-radius: 8px;
            background: #0f1629;
            cursor: crosshair;
        }
        .shoot-btn {
            width: 100%;
            padding: 16px;
            font-size: 18px;
            font-weight: bold;
            background: linear-gradient(135deg, #f39c12, #e74c3c);
            border: none;
            border-radius: 10px;
            cursor: pointer;
            color: white;
            margin: 10px 0;
        }
        .shoot-btn:active { transform: scale(0.98); }
        .shoot-btn:disabled { background: #444; cursor: not-allowed; opacity: 0.5; }
        .config-row {
            display: flex;
            align-items: center;
            gap: 10px;
            margin: 6px 0;
            font-size: 13px;
        }
        .config-row label { min-width: 80px; }
        .config-row input[type=range] { flex: 1; }
        .config-row .config-val { font-family: monospace; min-width: 60px; text-align: right; }
        .point-list { font-size: 12px; }
        .point-item {
            display: flex;
            align-items: center;
            gap: 6px;
            padding: 4px 0;
            border-bottom: 1px solid #2a2a4a;
        }
        .point-item input {
            width: 70px;
            background: #0f1629;
            border: 1px solid #333;
            border-radius: 4px;
            color: #eee;
            padding: 3px 5px;
            font-family: monospace;
            font-size: 12px;
        }
        .point-item button {
            background: #c0392b;
            border: none;
            color: white;
            padding: 3px 8px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 11px;
        }
        .add-btn {
            background: #27ae60;
            border: none;
            color: white;
            padding: 6px 14px;
            border-radius: 6px;
            cursor: pointer;
            font-size: 13px;
            margin-top: 6px;
        }
        .speed-display {
            text-align: center;
            padding: 8px;
        }
        .speed-value { font-size: 40px; font-weight: bold; font-family: monospace; color: #f39c12; }
        .speed-label { font-size: 11px; color: #888; margin-top: 2px; }
        .vel-legend { font-size: 11px; text-align: center; margin-top: 4px; }
        .vel-legend span { margin: 0 8px; }
        .target-color { color: #f39c12; }
        .actual-color { color: #2ecc71; }
    </style>
</head>
<body>
    <h1>Shot Tuning</h1>

    <div class="panel">
        <div class="row">
            <span><span id="tgt-ind" class="indicator off"></span>Target</span>
            <span class="val" id="tgt-status">--</span>
        </div>
        <div class="row">
            <span>Distance</span>
            <span class="val"><span id="dist">--</span> m</span>
        </div>
        <div class="row">
            <span>Spindexer</span>
            <span class="val" id="spindexer-state">--</span>
        </div>
        <div class="row">
            <span><span id="rdy-ind" class="indicator off"></span>Ready</span>
            <span class="val" id="rdy-status">--</span>
        </div>
    </div>

    <div class="panel speed-display">
        <div class="speed-value"><span id="target-speed">--</span></div>
        <div class="speed-label">TARGET SPEED (rad/s)</div>
    </div>

    <button class="shoot-btn" id="shoot-btn" onclick="shoot()" disabled>SHOOT</button>

    <h3>Flywheel Velocity</h3>
    <div class="panel" style="padding:0">
        <canvas id="vel-canvas" height="160"></canvas>
    </div>
    <div class="vel-legend">
        <span class="target-color">--- Target</span>
        <span class="actual-color">--- Actual</span>
    </div>

    <h3>Distance vs Speed Curve</h3>
    <div class="panel" style="padding:0">
        <canvas id="curve-canvas" height="200"></canvas>
    </div>
    <p style="font-size:11px;color:#666;text-align:center;margin-top:4px">Click on the graph to add a point. Drag points to move them.</p>

    <h3>Data Points</h3>
    <div class="panel">
        <div id="point-list" class="point-list"></div>
        <button class="add-btn" onclick="addPointPrompt()">+ Add Point</button>
    </div>

    <h3>Configuration</h3>
    <div class="panel">
        <div class="config-row">
            <label>Inertia</label>
            <input type="range" id="inertia-slider" min="0.01" max="1.0" step="0.005" value="0.1">
            <span class="config-val" id="inertia-val">0.100</span>
            <span style="font-size:11px;color:#666">kg*m^2</span>
        </div>
    </div>

    <script>
        // State
        let points = [];
        let velHistory = [];
        let curveData = [];
        let status = {};
        let dragIndex = -1;
        let isDragging = false;

        // Canvas refs
        const velCanvas = document.getElementById('vel-canvas');
        const curveCanvas = document.getElementById('curve-canvas');
        const velCtx = velCanvas.getContext('2d');
        const curveCtx = curveCanvas.getContext('2d');

        function resizeCanvas(canvas) {
            const rect = canvas.getBoundingClientRect();
            canvas.width = rect.width * devicePixelRatio;
            canvas.height = rect.height * devicePixelRatio;
            canvas.getContext('2d').scale(devicePixelRatio, devicePixelRatio);
        }
        resizeCanvas(velCanvas);
        resizeCanvas(curveCanvas);

        // --- Status polling ---
        function updateStatus() {
            fetch('/api/status').then(r=>r.json()).then(d => {
                status = d;
                document.getElementById('tgt-ind').className = 'indicator ' + (d.hasTarget?'on':'off');
                document.getElementById('tgt-status').textContent = d.hasTarget?'Locked':'None';
                document.getElementById('dist').textContent = d.distance.toFixed(2);
                document.getElementById('rdy-ind').className = 'indicator ' + (d.readyToShoot?'on':'off');
                document.getElementById('rdy-status').textContent = d.readyToShoot?'Yes':'No';
                document.getElementById('target-speed').textContent = d.targetFlywheelSpeed.toFixed(0);
                document.getElementById('shoot-btn').disabled = !d.readyToShoot;
                document.getElementById('spindexer-state').textContent = d.spindexerState;

                // Update inertia slider if not being dragged
                if (!inertiaSliderActive) {
                    document.getElementById('inertia-slider').value = d.inertia;
                    document.getElementById('inertia-val').textContent = d.inertia.toFixed(3);
                }
            }).catch(()=>{});
        }

        function shoot() {
            fetch('/api/shoot', {method:'POST'}).catch(()=>{});
        }

        // --- Velocity history ---
        function updateVelHistory() {
            fetch('/api/velocity-history').then(r=>r.json()).then(d => {
                velHistory = d;
                drawVelocityGraph();
            }).catch(()=>{});
        }

        function drawVelocityGraph() {
            const ctx = velCtx;
            const W = velCanvas.getBoundingClientRect().width;
            const H = velCanvas.getBoundingClientRect().height;
            const pad = {l:45, r:10, t:10, b:25};

            ctx.clearRect(0,0,W,H);
            if (velHistory.length < 2) {
                ctx.fillStyle = '#666';
                ctx.font = '12px sans-serif';
                ctx.textAlign = 'center';
                ctx.fillText('Waiting for data...', W/2, H/2);
                return;
            }

            const times = velHistory.map(v=>v.time);
            const tMin = times[0], tMax = times[times.length-1];
            const allVals = velHistory.flatMap(v=>[v.target,v.actual]);
            let vMin = Math.min(0, ...allVals);
            let vMax = Math.max(100, ...allVals) * 1.1;

            function tx(t) { return pad.l + (t-tMin)/(tMax-tMin) * (W-pad.l-pad.r); }
            function ty(v) { return pad.t + (1-(v-vMin)/(vMax-vMin)) * (H-pad.t-pad.b); }

            // Grid
            ctx.strokeStyle = '#2a2a4a';
            ctx.lineWidth = 0.5;
            for (let v = 0; v <= vMax; v += 100) {
                const y = ty(v);
                ctx.beginPath(); ctx.moveTo(pad.l,y); ctx.lineTo(W-pad.r,y); ctx.stroke();
                ctx.fillStyle = '#666'; ctx.font = '10px monospace'; ctx.textAlign = 'right';
                ctx.fillText(v.toFixed(0), pad.l-4, y+3);
            }

            // Target line
            ctx.strokeStyle = '#f39c12';
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            velHistory.forEach((v,i) => {
                const x = tx(v.time), y = ty(v.target);
                i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
            });
            ctx.stroke();

            // Actual line
            ctx.strokeStyle = '#2ecc71';
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            velHistory.forEach((v,i) => {
                const x = tx(v.time), y = ty(v.actual);
                i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
            });
            ctx.stroke();
        }

        // --- Curve graph ---
        function updatePoints() {
            fetch('/api/points').then(r=>r.json()).then(d => {
                points = d;
                renderPointList();
                return fetch('/api/interpolated');
            }).then(r=>r.json()).then(d => {
                curveData = d;
                drawCurveGraph();
            }).catch(()=>{});
        }

        const CURVE_PAD = {l:50, r:15, t:15, b:30};
        let curveXRange = [0, 5];
        let curveYRange = [0, 650];

        function computeCurveRanges() {
            if (points.length > 0) {
                const dists = points.map(p=>p.distance);
                const speeds = points.map(p=>p.speed);
                const dMin = Math.min(...dists) - 0.5;
                const dMax = Math.max(...dists) + 0.5;
                const sMin = Math.min(0, Math.min(...speeds) - 50);
                const sMax = Math.max(...speeds) + 80;
                curveXRange = [Math.max(0, dMin), dMax];
                curveYRange = [sMin, sMax];
            }
        }

        function curveTx(d) {
            const W = curveCanvas.getBoundingClientRect().width;
            return CURVE_PAD.l + (d-curveXRange[0])/(curveXRange[1]-curveXRange[0]) * (W-CURVE_PAD.l-CURVE_PAD.r);
        }
        function curveTy(s) {
            const H = curveCanvas.getBoundingClientRect().height;
            return CURVE_PAD.t + (1-(s-curveYRange[0])/(curveYRange[1]-curveYRange[0])) * (H-CURVE_PAD.t-CURVE_PAD.b);
        }
        function curveInvX(px) {
            const W = curveCanvas.getBoundingClientRect().width;
            return curveXRange[0] + (px-CURVE_PAD.l)/(W-CURVE_PAD.l-CURVE_PAD.r) * (curveXRange[1]-curveXRange[0]);
        }
        function curveInvY(py) {
            const H = curveCanvas.getBoundingClientRect().height;
            return curveYRange[0] + (1-(py-CURVE_PAD.t)/(H-CURVE_PAD.t-CURVE_PAD.b)) * (curveYRange[1]-curveYRange[0]);
        }

        function drawCurveGraph() {
            const ctx = curveCtx;
            const W = curveCanvas.getBoundingClientRect().width;
            const H = curveCanvas.getBoundingClientRect().height;
            computeCurveRanges();

            ctx.clearRect(0,0,W,H);

            // Grid
            ctx.strokeStyle = '#2a2a4a';
            ctx.lineWidth = 0.5;
            ctx.fillStyle = '#666';
            ctx.font = '10px monospace';

            // Y grid (speed)
            const yStep = Math.max(50, Math.round((curveYRange[1]-curveYRange[0])/6/50)*50);
            for (let s = 0; s <= curveYRange[1]; s += yStep) {
                const y = curveTy(s);
                if (y > CURVE_PAD.t && y < H-CURVE_PAD.b) {
                    ctx.beginPath(); ctx.moveTo(CURVE_PAD.l,y); ctx.lineTo(W-CURVE_PAD.r,y); ctx.stroke();
                    ctx.textAlign = 'right';
                    ctx.fillText(s.toFixed(0), CURVE_PAD.l-4, y+3);
                }
            }

            // X grid (distance)
            const xStep = Math.max(0.5, Math.round((curveXRange[1]-curveXRange[0])/6*2)/2);
            for (let d = Math.ceil(curveXRange[0]/xStep)*xStep; d <= curveXRange[1]; d += xStep) {
                const x = curveTx(d);
                ctx.beginPath(); ctx.moveTo(x,CURVE_PAD.t); ctx.lineTo(x,H-CURVE_PAD.b); ctx.stroke();
                ctx.textAlign = 'center';
                ctx.fillText(d.toFixed(1)+'m', x, H-CURVE_PAD.b+14);
            }

            // Axis labels
            ctx.fillStyle = '#888';
            ctx.font = '11px sans-serif';
            ctx.save();
            ctx.translate(12, H/2);
            ctx.rotate(-Math.PI/2);
            ctx.textAlign = 'center';
            ctx.fillText('Speed (rad/s)', 0, 0);
            ctx.restore();

            // Interpolated curve
            if (curveData.length > 1) {
                ctx.strokeStyle = '#3498db';
                ctx.lineWidth = 2;
                ctx.setLineDash([4,3]);
                ctx.beginPath();
                curveData.forEach((p,i) => {
                    const x = curveTx(p.distance), y = curveTy(p.speed);
                    i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
                });
                ctx.stroke();
                ctx.setLineDash([]);
            }

            // Data points
            points.forEach((p,i) => {
                const x = curveTx(p.distance), y = curveTy(p.speed);
                ctx.fillStyle = (dragIndex===i && isDragging) ? '#e74c3c' : '#f39c12';
                ctx.beginPath();
                ctx.arc(x, y, 6, 0, Math.PI*2);
                ctx.fill();
                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 1.5;
                ctx.stroke();

                // Label
                ctx.fillStyle = '#ccc';
                ctx.font = '10px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(p.distance.toFixed(2)+'m', x, y-12);
                ctx.fillText(p.speed.toFixed(0), x, y+18);
            });

            // Current distance indicator
            if (status.distance > 0) {
                const x = curveTx(status.distance);
                ctx.strokeStyle = '#e74c3c88';
                ctx.lineWidth = 1;
                ctx.setLineDash([3,3]);
                ctx.beginPath(); ctx.moveTo(x,CURVE_PAD.t); ctx.lineTo(x,H-CURVE_PAD.b); ctx.stroke();
                ctx.setLineDash([]);

                if (status.targetFlywheelSpeed > 0) {
                    const y = curveTy(status.targetFlywheelSpeed);
                    ctx.fillStyle = '#e74c3c';
                    ctx.beginPath();
                    ctx.arc(x, y, 4, 0, Math.PI*2);
                    ctx.fill();
                }
            }
        }

        // --- Curve canvas interaction ---
        function getCurveCanvasPos(e) {
            const rect = curveCanvas.getBoundingClientRect();
            const touch = e.touches ? e.touches[0] : e;
            return { x: touch.clientX - rect.left, y: touch.clientY - rect.top };
        }

        function findNearestPoint(pos) {
            let minDist = 20; // pixel threshold
            let idx = -1;
            points.forEach((p,i) => {
                const px = curveTx(p.distance), py = curveTy(p.speed);
                const d = Math.hypot(pos.x-px, pos.y-py);
                if (d < minDist) { minDist = d; idx = i; }
            });
            return idx;
        }

        curveCanvas.addEventListener('mousedown', e => {
            const pos = getCurveCanvasPos(e);
            dragIndex = findNearestPoint(pos);
            if (dragIndex >= 0) {
                isDragging = true;
            }
        });

        curveCanvas.addEventListener('mousemove', e => {
            if (!isDragging || dragIndex < 0) return;
            const pos = getCurveCanvasPos(e);
            const dist = Math.max(0, curveInvX(pos.x));
            const speed = Math.max(50, Math.min(628, curveInvY(pos.y)));
            points[dragIndex] = {distance: dist, speed: speed};
            drawCurveGraph();
        });

        curveCanvas.addEventListener('mouseup', e => {
            if (isDragging && dragIndex >= 0) {
                const p = points[dragIndex];
                fetch('/api/points/' + dragIndex, {
                    method: 'PUT',
                    headers: {'Content-Type':'application/json'},
                    body: JSON.stringify(p)
                }).then(() => updatePoints());
            } else if (!isDragging) {
                // Click to add point
                const pos = getCurveCanvasPos(e);
                if (pos.x > CURVE_PAD.l && pos.y > CURVE_PAD.t &&
                    pos.x < curveCanvas.getBoundingClientRect().width - CURVE_PAD.r &&
                    pos.y < curveCanvas.getBoundingClientRect().height - CURVE_PAD.b) {
                    const dist = Math.max(0, curveInvX(pos.x));
                    const speed = Math.max(50, Math.min(628, curveInvY(pos.y)));
                    fetch('/api/points', {
                        method: 'POST',
                        headers: {'Content-Type':'application/json'},
                        body: JSON.stringify({distance: dist, speed: speed})
                    }).then(() => updatePoints());
                }
            }
            isDragging = false;
            dragIndex = -1;
        });

        // Touch support
        curveCanvas.addEventListener('touchstart', e => {
            e.preventDefault();
            const pos = getCurveCanvasPos(e);
            dragIndex = findNearestPoint(pos);
            if (dragIndex >= 0) isDragging = true;
        }, {passive:false});

        curveCanvas.addEventListener('touchmove', e => {
            e.preventDefault();
            if (!isDragging || dragIndex < 0) return;
            const touch = e.touches[0];
            const rect = curveCanvas.getBoundingClientRect();
            const pos = {x: touch.clientX - rect.left, y: touch.clientY - rect.top};
            const dist = Math.max(0, curveInvX(pos.x));
            const speed = Math.max(50, Math.min(628, curveInvY(pos.y)));
            points[dragIndex] = {distance: dist, speed: speed};
            drawCurveGraph();
        }, {passive:false});

        curveCanvas.addEventListener('touchend', e => {
            if (isDragging && dragIndex >= 0) {
                const p = points[dragIndex];
                fetch('/api/points/' + dragIndex, {
                    method: 'PUT',
                    headers: {'Content-Type':'application/json'},
                    body: JSON.stringify(p)
                }).then(() => updatePoints());
            }
            isDragging = false;
            dragIndex = -1;
        });

        // --- Point list ---
        function renderPointList() {
            const container = document.getElementById('point-list');
            if (points.length === 0) {
                container.innerHTML = '<div style="color:#666;text-align:center;padding:8px">No points yet. Add 2 points to enable interpolation.</div>';
                return;
            }
            container.innerHTML = points.map((p,i) =>
                '<div class="point-item">' +
                '<span style="color:#888;min-width:18px">#'+(i+1)+'</span>' +
                '<input type="number" step="0.1" value="'+p.distance.toFixed(2)+'" onchange="editPoint('+i+',this.value,null)" placeholder="dist">' +
                '<span style="color:#666">m</span>' +
                '<input type="number" step="10" value="'+p.speed.toFixed(0)+'" onchange="editPoint('+i+',null,this.value)" placeholder="speed">' +
                '<span style="color:#666">rad/s</span>' +
                '<button onclick="deletePoint('+i+')">X</button>' +
                '</div>'
            ).join('');
        }

        function editPoint(idx, dist, speed) {
            const p = points[idx];
            const newDist = dist !== null ? parseFloat(dist) : p.distance;
            const newSpeed = speed !== null ? parseFloat(speed) : p.speed;
            fetch('/api/points/'+idx, {
                method:'PUT',
                headers:{'Content-Type':'application/json'},
                body: JSON.stringify({distance:newDist, speed:newSpeed})
            }).then(() => updatePoints());
        }

        function deletePoint(idx) {
            fetch('/api/points/'+idx, {method:'DELETE'}).then(() => updatePoints());
        }

        function addPointPrompt() {
            const dist = prompt('Distance (meters):', status.distance ? status.distance.toFixed(2) : '1.0');
            if (!dist) return;
            const speed = prompt('Speed (rad/s):', '300');
            if (!speed) return;
            fetch('/api/points', {
                method:'POST',
                headers:{'Content-Type':'application/json'},
                body: JSON.stringify({distance:parseFloat(dist), speed:parseFloat(speed)})
            }).then(() => updatePoints());
        }

        // --- Inertia config ---
        let inertiaSliderActive = false;
        const inertiaSlider = document.getElementById('inertia-slider');
        inertiaSlider.addEventListener('input', () => {
            inertiaSliderActive = true;
            document.getElementById('inertia-val').textContent = parseFloat(inertiaSlider.value).toFixed(3);
        });
        inertiaSlider.addEventListener('change', () => {
            const val = parseFloat(inertiaSlider.value);
            fetch('/api/config', {
                method:'POST',
                headers:{'Content-Type':'application/json'},
                body: JSON.stringify({inertia: val})
            });
            setTimeout(() => { inertiaSliderActive = false; }, 500);
        });

        // --- Polling ---
        setInterval(updateStatus, 200);
        setInterval(updateVelHistory, 300);
        setInterval(updatePoints, 2000);

        // Initial load
        updateStatus();
        updateVelHistory();
        updatePoints();
    </script>
</body>
</html>
        """.trimIndent()
    }
}
