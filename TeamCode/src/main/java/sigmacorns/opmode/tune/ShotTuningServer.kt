package sigmacorns.opmode.tune

import com.google.gson.Gson
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.PrintWriter
import java.net.ServerSocket
import java.net.Socket

/**
 * Lightweight HTTP server for the shot tuning web interface.
 * Runs on the robot controller's WiFi Direct IP (typically 192.168.43.1:8080).
 * The driver station laptop can access it at that address.
 */
class ShotTuningServer(
    private val port: Int = 8080,
    private val handler: RequestHandler
) {
    interface RequestHandler {
        fun getState(): ShotTuningState
        fun shoot()
        fun setDistance(distance: Double)
        fun setFlywheelTarget(speed: Double)
        fun setHoodAngle(angle: Double)
        fun savePoint()
        fun deletePoint(index: Int)
        fun getPoints(): String
        fun saveToFile()
    }

    data class ShotTuningState(
        val flywheelTarget: Double,
        val flywheelActual: Double,
        val flywheelRPM: Double,
        val flywheelPower: Double,
        val hoodAngle: Double,
        val hoodServo: Double,
        val distance: Double,
        val isShooting: Boolean,
        val ballCount: Int
    )

    private var serverSocket: ServerSocket? = null
    private var thread: Thread? = null
    @Volatile var running = false
        private set

    private val gson = Gson()

    fun start() {
        running = true
        thread = Thread {
            try {
                serverSocket = ServerSocket(port)
                serverSocket!!.soTimeout = 500
                while (running) {
                    try {
                        val client = serverSocket!!.accept()
                        handleClient(client)
                    } catch (_: java.net.SocketTimeoutException) {
                        // Check running flag
                    }
                }
            } catch (e: Exception) {
                if (running) e.printStackTrace()
            } finally {
                serverSocket?.close()
            }
        }
        thread!!.isDaemon = true
        thread!!.start()
    }

    fun stop() {
        running = false
        serverSocket?.close()
        thread?.join(2000)
    }

    private fun handleClient(client: Socket) {
        try {
            val reader = BufferedReader(InputStreamReader(client.getInputStream()))
            val writer = PrintWriter(client.getOutputStream(), true)

            val requestLine = reader.readLine() ?: return
            val parts = requestLine.split(" ")
            if (parts.size < 2) return
            val method = parts[0]
            val path = parts[1]

            // Read headers
            var contentLength = 0
            var line = reader.readLine()
            while (line != null && line.isNotEmpty()) {
                if (line.startsWith("Content-Length:", ignoreCase = true)) {
                    contentLength = line.substringAfter(":").trim().toIntOrNull() ?: 0
                }
                line = reader.readLine()
            }

            // Read body
            val body = if (contentLength > 0) {
                val buf = CharArray(contentLength)
                reader.read(buf, 0, contentLength)
                String(buf)
            } else ""

            val (status, contentType, responseBody) = route(method, path, body)

            writer.print("HTTP/1.1 $status\r\n")
            writer.print("Content-Type: $contentType\r\n")
            writer.print("Access-Control-Allow-Origin: *\r\n")
            writer.print("Access-Control-Allow-Methods: GET, POST, DELETE, OPTIONS\r\n")
            writer.print("Access-Control-Allow-Headers: Content-Type\r\n")
            writer.print("Connection: close\r\n")
            writer.print("Content-Length: ${responseBody.toByteArray().size}\r\n")
            writer.print("\r\n")
            writer.print(responseBody)
            writer.flush()
        } catch (e: Exception) {
            e.printStackTrace()
        } finally {
            client.close()
        }
    }

    private fun route(method: String, path: String, body: String): Triple<String, String, String> {
        if (method == "OPTIONS") {
            return Triple("200 OK", "text/plain", "")
        }

        return when {
            path == "/" && method == "GET" -> Triple("200 OK", "text/html", WEB_UI_HTML)
            path == "/api/state" && method == "GET" -> {
                val state = handler.getState()
                Triple("200 OK", "application/json", gson.toJson(state))
            }
            path == "/api/shoot" && method == "POST" -> {
                handler.shoot()
                Triple("200 OK", "application/json", """{"ok":true}""")
            }
            path == "/api/distance" && method == "POST" -> {
                val map = gson.fromJson(body, Map::class.java)
                handler.setDistance((map["distance"] as? Number)?.toDouble() ?: 1.0)
                Triple("200 OK", "application/json", """{"ok":true}""")
            }
            path == "/api/flywheel" && method == "POST" -> {
                val map = gson.fromJson(body, Map::class.java)
                handler.setFlywheelTarget((map["speed"] as? Number)?.toDouble() ?: 400.0)
                Triple("200 OK", "application/json", """{"ok":true}""")
            }
            path == "/api/hood" && method == "POST" -> {
                val map = gson.fromJson(body, Map::class.java)
                handler.setHoodAngle((map["angle"] as? Number)?.toDouble() ?: 45.0)
                Triple("200 OK", "application/json", """{"ok":true}""")
            }
            path == "/api/save-point" && method == "POST" -> {
                handler.savePoint()
                Triple("200 OK", "application/json", """{"ok":true}""")
            }
            path.startsWith("/api/point/") && method == "DELETE" -> {
                val idx = path.substringAfterLast("/").toIntOrNull() ?: -1
                if (idx >= 0) handler.deletePoint(idx)
                Triple("200 OK", "application/json", """{"ok":true}""")
            }
            path == "/api/points" && method == "GET" -> {
                Triple("200 OK", "application/json", handler.getPoints())
            }
            path == "/api/save-file" && method == "POST" -> {
                handler.saveToFile()
                Triple("200 OK", "application/json", """{"ok":true}""")
            }
            else -> Triple("404 Not Found", "text/plain", "Not Found")
        }
    }
}

/** The entire web UI as a single HTML string with embedded CSS and JS. */
val WEB_UI_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Shot Tuning</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; background:#1a1a2e; color:#eee; padding:16px; }
h1 { color:#e94560; margin-bottom:12px; font-size:1.4em; }
h2 { color:#0f3460; background:#e94560; padding:6px 12px; border-radius:4px; margin:12px 0 8px; font-size:1em; }
.grid { display:grid; grid-template-columns:1fr 1fr; gap:16px; }
.card { background:#16213e; border-radius:8px; padding:16px; }
.full { grid-column: 1/-1; }
.row { display:flex; align-items:center; gap:8px; margin:6px 0; }
label { min-width:120px; font-size:0.9em; color:#aaa; }
input[type=range] { flex:1; }
input[type=number] { width:80px; background:#0f3460; border:1px solid #e94560; color:#eee; padding:4px 8px; border-radius:4px; }
.val { min-width:60px; text-align:right; font-weight:bold; color:#e94560; }
button { background:#e94560; color:#fff; border:none; padding:10px 20px; border-radius:6px; cursor:pointer; font-size:1em; font-weight:bold; }
button:hover { background:#c73a52; }
button:active { transform:scale(0.97); }
button.secondary { background:#0f3460; }
button.danger { background:#a02040; }
.stat { display:inline-block; background:#0f3460; padding:4px 10px; border-radius:4px; margin:2px; font-size:0.85em; }
.stat b { color:#e94560; }
canvas { width:100%!important; height:250px!important; }
table { width:100%; border-collapse:collapse; font-size:0.85em; margin-top:8px; }
th { background:#0f3460; padding:6px; text-align:left; }
td { padding:6px; border-bottom:1px solid #1a1a2e; }
.status { font-size:0.8em; color:#888; margin-top:4px; }
</style>
</head>
<body>
<h1>Shot Tuning Interface</h1>

<div class="grid">
  <div class="card">
    <h2>Controls</h2>
    <div class="row">
      <label>Distance (m):</label>
      <input type="range" id="distSlider" min="0.5" max="5" step="0.05" value="2.0" oninput="setDist(this.value)">
      <span class="val" id="distVal">2.00</span>
    </div>
    <div class="row">
      <label>Flywheel (rad/s):</label>
      <input type="range" id="fwSlider" min="50" max="628" step="5" value="400" oninput="setFW(this.value)">
      <span class="val" id="fwVal">400</span>
    </div>
    <div class="row">
      <label>Hood Angle (°):</label>
      <input type="range" id="hoodSlider" min="15" max="70" step="0.5" value="45" oninput="setHood(this.value)">
      <span class="val" id="hoodVal">45.0</span>
    </div>
    <div class="row" style="margin-top:12px;">
      <button onclick="shoot()" id="shootBtn">SHOOT</button>
      <button class="secondary" onclick="savePoint()">Save Data Point</button>
      <button class="secondary" onclick="saveFile()">Save to File</button>
    </div>
  </div>

  <div class="card">
    <h2>Live Status</h2>
    <div id="stats">
      <span class="stat">Flywheel: <b id="s_fw">0</b> rad/s</span>
      <span class="stat">Target: <b id="s_fwt">0</b> rad/s</span>
      <span class="stat">RPM: <b id="s_rpm">0</b></span>
      <span class="stat">Power: <b id="s_pwr">0</b></span>
      <span class="stat">Hood: <b id="s_hood">0</b>°</span>
      <span class="stat">Servo: <b id="s_servo">0</b></span>
      <span class="stat">Distance: <b id="s_dist">0</b> m</span>
      <span class="stat">Balls: <b id="s_balls">0</b></span>
    </div>
    <div class="status" id="statusLine">Connecting...</div>
  </div>

  <div class="card">
    <h2>Flywheel Power (Real-Time)</h2>
    <canvas id="powerChart"></canvas>
  </div>

  <div class="card">
    <h2>Interpolation Curve</h2>
    <canvas id="interpChart"></canvas>
  </div>

  <div class="card full">
    <h2>Saved Data Points</h2>
    <table>
      <thead><tr><th>#</th><th>Distance (m)</th><th>Speed (rad/s)</th><th>Hood (°)</th><th></th></tr></thead>
      <tbody id="pointsTable"></tbody>
    </table>
  </div>
</div>

<script>
const BASE = window.location.origin;
const MAX_HISTORY = 200;
let powerHistory = [];
let targetHistory = [];
let timeLabels = [];
let tick = 0;

// Charts
const powerCtx = document.getElementById('powerChart').getContext('2d');
const powerChart = new Chart(powerCtx, {
  type: 'line',
  data: {
    labels: timeLabels,
    datasets: [
      { label: 'Actual Power', data: powerHistory, borderColor: '#e94560', borderWidth: 2, pointRadius: 0, fill: false },
      { label: 'Target', data: targetHistory, borderColor: '#e94560', borderWidth: 1, borderDash: [5,5], pointRadius: 0, fill: false }
    ]
  },
  options: { animation: false, scales: { x: { display: false }, y: { min: -1, max: 1, ticks: { color: '#888' }, grid: { color: '#1a1a2e' } } }, plugins: { legend: { labels: { color: '#aaa' } } } }
});

const interpCtx = document.getElementById('interpChart').getContext('2d');
const interpChart = new Chart(interpCtx, {
  type: 'scatter',
  data: {
    datasets: [
      { label: 'Data Points', data: [], backgroundColor: '#e94560', pointRadius: 6 },
      { label: 'Interpolation', data: [], borderColor: '#0f3460', borderWidth: 2, pointRadius: 0, showLine: true, fill: false }
    ]
  },
  options: { animation: false, scales: { x: { title: { display: true, text: 'Distance (m)', color: '#aaa' }, ticks: { color: '#888' }, grid: { color: '#1a1a2e' } }, y: { title: { display: true, text: 'Speed (rad/s)', color: '#aaa' }, ticks: { color: '#888' }, grid: { color: '#1a1a2e' } } }, plugins: { legend: { labels: { color: '#aaa' } } } }
});

function api(path, opts) {
  return fetch(BASE + path, opts).then(r => r.json()).catch(() => null);
}

function setDist(v) { document.getElementById('distVal').textContent = parseFloat(v).toFixed(2); api('/api/distance', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({distance:parseFloat(v)}) }); }
function setFW(v) { document.getElementById('fwVal').textContent = parseFloat(v).toFixed(0); api('/api/flywheel', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({speed:parseFloat(v)}) }); }
function setHood(v) { document.getElementById('hoodVal').textContent = parseFloat(v).toFixed(1); api('/api/hood', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({angle:parseFloat(v)}) }); }
function shoot() { api('/api/shoot', { method:'POST' }); }
function savePoint() { api('/api/save-point', { method:'POST' }).then(() => loadPoints()); }
function saveFile() { api('/api/save-file', { method:'POST' }); }
function deletePoint(i) { api('/api/point/' + i, { method:'DELETE' }).then(() => loadPoints()); }

async function loadPoints() {
  const pts = await api('/api/points');
  if (!pts) return;
  const tb = document.getElementById('pointsTable');
  tb.innerHTML = pts.map((p, i) =>
    '<tr><td>' + i + '</td><td>' + p.distance.toFixed(2) + '</td><td>' + p.speed.toFixed(0) + '</td><td>' + p.hoodAngle.toFixed(1) + '</td><td><button class="danger" onclick="deletePoint(' + i + ')">X</button></td></tr>'
  ).join('');

  // Update interpolation chart
  const scatter = pts.map(p => ({ x: p.distance, y: p.speed }));
  interpChart.data.datasets[0].data = scatter;

  // Build interpolation line
  if (pts.length >= 2) {
    const sorted = [...pts].sort((a, b) => a.distance - b.distance);
    const minD = Math.max(0.3, sorted[0].distance - 0.5);
    const maxD = sorted[sorted.length - 1].distance + 0.5;
    const line = [];
    for (let d = minD; d <= maxD; d += 0.05) {
      const lower = sorted.filter(p => p.distance <= d).pop();
      const upper = sorted.find(p => p.distance > d);
      let speed;
      if (lower && upper) {
        const t = (d - lower.distance) / (upper.distance - lower.distance);
        speed = lower.speed + t * (upper.speed - lower.speed);
      } else if (lower) {
        const l = sorted[sorted.length - 1], sl = sorted[sorted.length - 2];
        speed = l.speed + (l.speed - sl.speed) / (l.distance - sl.distance) * (d - l.distance);
      } else if (upper) {
        const f = sorted[0], s = sorted[1];
        speed = f.speed + (s.speed - f.speed) / (s.distance - f.distance) * (d - f.distance);
      }
      if (speed !== undefined) line.push({ x: d, y: Math.max(50, Math.min(628, speed)) });
    }
    interpChart.data.datasets[1].data = line;
  }
  interpChart.update();
}

async function poll() {
  try {
    const s = await api('/api/state');
    if (!s) { document.getElementById('statusLine').textContent = 'Disconnected'; return; }
    document.getElementById('statusLine').textContent = 'Connected';
    document.getElementById('s_fw').textContent = s.flywheelActual.toFixed(0);
    document.getElementById('s_fwt').textContent = s.flywheelTarget.toFixed(0);
    document.getElementById('s_rpm').textContent = s.flywheelRPM.toFixed(0);
    document.getElementById('s_pwr').textContent = s.flywheelPower.toFixed(3);
    document.getElementById('s_hood').textContent = s.hoodAngle.toFixed(1);
    document.getElementById('s_servo').textContent = s.hoodServo.toFixed(3);
    document.getElementById('s_dist').textContent = s.distance.toFixed(2);
    document.getElementById('s_balls').textContent = s.ballCount;

    // Update power chart
    tick++;
    timeLabels.push(tick);
    powerHistory.push(s.flywheelPower);
    const normalizedTarget = s.flywheelTarget > 0 ? 1.0 : 0.0;
    targetHistory.push(normalizedTarget);
    if (timeLabels.length > MAX_HISTORY) { timeLabels.shift(); powerHistory.shift(); targetHistory.shift(); }
    powerChart.update();
  } catch(e) {
    document.getElementById('statusLine').textContent = 'Error: ' + e.message;
  }
}

setInterval(poll, 100);
loadPoints();
</script>
</body>
</html>
""".trimIndent()
