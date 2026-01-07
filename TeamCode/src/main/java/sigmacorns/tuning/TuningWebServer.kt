package sigmacorns.tuning

import com.google.gson.Gson
import fi.iki.elonen.NanoHTTPD
import sigmacorns.tuning.model.FeedbackType

/**
 * Web server for the shot tuning interface.
 * Provides a simple HTML UI and REST API endpoints.
 */
class TuningWebServer(
    port: Int = 8082,
    private val stateProvider: () -> TuningState,
    private val onFeedback: (FeedbackType, Int) -> Unit,
    private val onShoot: () -> Unit,
    private val dataStore: ShotDataStore
) : NanoHTTPD(port) {

    private val gson = Gson()

    /**
     * Current state of the tuning system.
     */
    data class TuningState(
        val hasTarget: Boolean,
        val distance: Double,
        val currentFlywheelSpeed: Double,
        val targetFlywheelSpeed: Double,
        val turretAligned: Boolean,
        val readyToShoot: Boolean,
        val tuningInfo: String
    )

    override fun serve(session: IHTTPSession): Response {
        val uri = session.uri
        val method = session.method

        return try {
            when {
                uri == "/" || uri == "/index.html" -> serveMainPage()
                uri == "/api/status" -> serveStatus()
                uri == "/api/feedback" && method == Method.POST -> handleFeedback(session)
                uri == "/api/shoot" && method == Method.POST -> handleShoot()
                uri == "/api/trials" -> serveTrials(session)
                uri == "/api/lookup" -> serveLookupTable()
                uri == "/api/export" -> exportCSV()
                else -> newFixedLengthResponse(
                    Response.Status.NOT_FOUND,
                    MIME_PLAINTEXT,
                    "Not Found"
                )
            }
        } catch (e: Exception) {
            e.printStackTrace()
            newFixedLengthResponse(
                Response.Status.INTERNAL_ERROR,
                MIME_PLAINTEXT,
                "Error: ${e.message}"
            )
        }
    }

    private fun serveMainPage(): Response {
        return newFixedLengthResponse(Response.Status.OK, "text/html", HTML_PAGE)
    }

    private fun serveStatus(): Response {
        val state = stateProvider()
        val json = gson.toJson(state)
        return newFixedLengthResponse(Response.Status.OK, "application/json", json)
    }

    private fun handleFeedback(session: IHTTPSession): Response {
        val contentLength = session.headers["content-length"]?.toIntOrNull() ?: 0
        val body = ByteArray(contentLength)
        session.inputStream.read(body, 0, contentLength)
        val jsonBody = String(body)

        val request = gson.fromJson(jsonBody, FeedbackRequest::class.java)
        val feedbackType = FeedbackType.valueOf(request.type)

        onFeedback(feedbackType, request.intensity)

        return newFixedLengthResponse(
            Response.Status.OK,
            "application/json",
            """{"success": true}"""
        )
    }

    private fun handleShoot(): Response {
        onShoot()
        return newFixedLengthResponse(
            Response.Status.OK,
            "application/json",
            """{"success": true}"""
        )
    }

    private fun serveTrials(session: IHTTPSession): Response {
        val params = session.parms
        val distanceStr = params["distance"]
        val distance = distanceStr?.toDoubleOrNull()

        val json = dataStore.exportTrialsJson(distance)
        return newFixedLengthResponse(Response.Status.OK, "application/json", json)
    }

    private fun serveLookupTable(): Response {
        val json = dataStore.exportLookupTableJson()
        return newFixedLengthResponse(Response.Status.OK, "application/json", json)
    }

    private fun exportCSV(): Response {
        val entries = dataStore.getLookupTableSorted()
        val csv = buildString {
            appendLine("distance_min,distance_max,optimal_speed,confirmed_trials")
            entries.forEach { entry ->
                appendLine("${entry.distanceMin},${entry.distanceMax},${entry.optimalSpeed},${entry.confirmedTrials}")
            }
        }

        val response = newFixedLengthResponse(Response.Status.OK, "text/csv", csv)
        response.addHeader("Content-Disposition", "attachment; filename=shot_tuning_lookup.csv")
        return response
    }

    private data class FeedbackRequest(
        val type: String,
        val intensity: Int
    )

    companion object {
        private val HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Shot Tuning</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            max-width: 600px;
            margin: 0 auto;
            padding: 16px;
            background: #1a1a2e;
            color: #eee;
        }
        h1 {
            text-align: center;
            margin: 0 0 16px 0;
            font-size: 24px;
        }
        h3 {
            margin: 16px 0 8px 0;
            font-size: 14px;
            color: #888;
            text-transform: uppercase;
        }
        .status-panel {
            background: #16213e;
            padding: 12px;
            border-radius: 8px;
            margin-bottom: 16px;
        }
        .status-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin: 6px 0;
            font-size: 14px;
        }
        .status-value {
            font-family: monospace;
            font-weight: bold;
        }
        .feedback-buttons {
            display: grid;
            grid-template-columns: repeat(5, 1fr);
            gap: 8px;
            margin: 12px 0;
        }
        .feedback-btn {
            padding: 16px 8px;
            border: none;
            border-radius: 8px;
            font-size: 12px;
            font-weight: bold;
            cursor: pointer;
            color: white;
            transition: transform 0.1s, opacity 0.1s;
        }
        .feedback-btn:active { transform: scale(0.95); }
        .feedback-btn:disabled { opacity: 0.5; cursor: not-allowed; }
        .btn-big-under { background: #c0392b; }
        .btn-slight-under { background: #e67e22; }
        .btn-good { background: #27ae60; font-size: 14px; }
        .btn-slight-over { background: #2980b9; }
        .btn-big-over { background: #8e44ad; }
        .shoot-btn {
            width: 100%;
            padding: 20px;
            font-size: 20px;
            font-weight: bold;
            background: linear-gradient(135deg, #f39c12, #e74c3c);
            border: none;
            border-radius: 12px;
            cursor: pointer;
            color: white;
            margin: 16px 0;
            transition: transform 0.1s, opacity 0.1s;
        }
        .shoot-btn:active { transform: scale(0.98); }
        .shoot-btn:disabled {
            background: #444;
            cursor: not-allowed;
            opacity: 0.6;
        }
        .indicator {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            display: inline-block;
            margin-right: 6px;
        }
        .indicator.active { background: #2ecc71; box-shadow: 0 0 8px #2ecc71; }
        .indicator.inactive { background: #e74c3c; }
        table {
            width: 100%;
            border-collapse: collapse;
            font-size: 13px;
            background: #16213e;
            border-radius: 8px;
            overflow: hidden;
        }
        th {
            background: #0f1629;
            padding: 8px;
            text-align: left;
            font-weight: normal;
            color: #888;
        }
        td {
            padding: 8px;
            border-bottom: 1px solid #2a2a4a;
        }
        .history-panel {
            max-height: 180px;
            overflow-y: auto;
            background: #16213e;
            border-radius: 8px;
        }
        .speed-display {
            text-align: center;
            padding: 12px;
            background: #16213e;
            border-radius: 8px;
            margin-bottom: 16px;
        }
        .speed-value {
            font-size: 48px;
            font-weight: bold;
            font-family: monospace;
            color: #f39c12;
        }
        .speed-label {
            font-size: 12px;
            color: #888;
            margin-top: 4px;
        }
        .tuning-info {
            font-size: 11px;
            color: #666;
            text-align: center;
            margin-top: 4px;
        }
        .feedback-good { color: #2ecc71; }
        .feedback-under { color: #e67e22; }
        .feedback-over { color: #3498db; }
        .export-btn {
            display: block;
            width: 100%;
            padding: 10px;
            margin-top: 12px;
            background: #16213e;
            border: 1px solid #333;
            border-radius: 8px;
            color: #888;
            text-align: center;
            text-decoration: none;
            font-size: 13px;
        }
    </style>
</head>
<body>
    <h1>Shot Tuning</h1>

    <div class="status-panel">
        <div class="status-row">
            <span><span id="target-indicator" class="indicator inactive"></span>Target</span>
            <span class="status-value" id="target-status">--</span>
        </div>
        <div class="status-row">
            <span>Distance</span>
            <span class="status-value"><span id="distance">--</span> m</span>
        </div>
        <div class="status-row">
            <span><span id="ready-indicator" class="indicator inactive"></span>Ready</span>
            <span class="status-value" id="ready-status">--</span>
        </div>
    </div>

    <div class="speed-display">
        <div class="speed-value"><span id="target-speed">--</span></div>
        <div class="speed-label">TARGET SPEED (rad/s)</div>
        <div class="tuning-info" id="tuning-info"></div>
    </div>

    <button class="shoot-btn" id="shoot-btn" onclick="shoot()" disabled>
        SHOOT
    </button>

    <h3>Feedback</h3>
    <div class="feedback-buttons">
        <button class="feedback-btn btn-big-under" onclick="feedback('UNDERSHOOT', 2)">
            BIG<br>UNDER
        </button>
        <button class="feedback-btn btn-slight-under" onclick="feedback('UNDERSHOOT', 1)">
            SLIGHT<br>UNDER
        </button>
        <button class="feedback-btn btn-good" onclick="feedback('GOOD', 0)">
            GOOD
        </button>
        <button class="feedback-btn btn-slight-over" onclick="feedback('OVERSHOOT', 1)">
            SLIGHT<br>OVER
        </button>
        <button class="feedback-btn btn-big-over" onclick="feedback('OVERSHOOT', 2)">
            BIG<br>OVER
        </button>
    </div>

    <h3>Recent Trials</h3>
    <div class="history-panel">
        <table id="history-table">
            <thead><tr><th>Speed</th><th>Result</th></tr></thead>
            <tbody></tbody>
        </table>
    </div>

    <h3>Lookup Table</h3>
    <table id="lookup-table">
        <thead><tr><th>Distance (m)</th><th>Speed (rad/s)</th><th>Trials</th></tr></thead>
        <tbody></tbody>
    </table>

    <a href="/api/export" class="export-btn">Export CSV</a>

    <script>
        let lastDistance = 0;

        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('target-indicator').className =
                        'indicator ' + (data.hasTarget ? 'active' : 'inactive');
                    document.getElementById('target-status').textContent =
                        data.hasTarget ? 'Locked' : 'None';
                    document.getElementById('distance').textContent =
                        data.distance.toFixed(2);
                    document.getElementById('target-speed').textContent =
                        data.targetFlywheelSpeed.toFixed(0);
                    document.getElementById('ready-indicator').className =
                        'indicator ' + (data.readyToShoot ? 'active' : 'inactive');
                    document.getElementById('ready-status').textContent =
                        data.readyToShoot ? 'Yes' : 'No';
                    document.getElementById('shoot-btn').disabled = !data.readyToShoot;
                    document.getElementById('tuning-info').textContent = data.tuningInfo || '';

                    // Update trials if distance changed significantly
                    if (Math.abs(data.distance - lastDistance) > 0.1) {
                        lastDistance = data.distance;
                        updateHistory();
                    }
                })
                .catch(e => console.error('Status error:', e));
        }

        function feedback(type, intensity) {
            // Disable buttons briefly
            document.querySelectorAll('.feedback-btn').forEach(b => b.disabled = true);

            fetch('/api/feedback', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({type: type, intensity: intensity})
            })
            .then(() => {
                updateHistory();
                updateLookup();
                // Re-enable buttons
                setTimeout(() => {
                    document.querySelectorAll('.feedback-btn').forEach(b => b.disabled = false);
                }, 300);
            })
            .catch(e => {
                console.error('Feedback error:', e);
                document.querySelectorAll('.feedback-btn').forEach(b => b.disabled = false);
            });
        }

        function shoot() {
            fetch('/api/shoot', {method: 'POST'})
                .catch(e => console.error('Shoot error:', e));
        }

        function updateHistory() {
            fetch('/api/trials?distance=' + lastDistance)
                .then(r => r.json())
                .then(trials => {
                    const tbody = document.querySelector('#history-table tbody');
                    if (trials.length === 0) {
                        tbody.innerHTML = '<tr><td colspan="2" style="text-align:center;color:#666">No trials yet</td></tr>';
                        return;
                    }
                    tbody.innerHTML = trials.slice(0, 10).map(t => {
                        let resultClass = 'feedback-good';
                        let resultText = 'Good';
                        if (t.feedbackType === 'UNDERSHOOT') {
                            resultClass = 'feedback-under';
                            resultText = t.feedbackIntensity >= 2 ? 'Big Under' : 'Slight Under';
                        } else if (t.feedbackType === 'OVERSHOOT') {
                            resultClass = 'feedback-over';
                            resultText = t.feedbackIntensity >= 2 ? 'Big Over' : 'Slight Over';
                        }
                        return '<tr><td>' + t.flywheelSpeed.toFixed(0) + '</td>' +
                               '<td class="' + resultClass + '">' + resultText + '</td></tr>';
                    }).join('');
                })
                .catch(e => console.error('History error:', e));
        }

        function updateLookup() {
            fetch('/api/lookup')
                .then(r => r.json())
                .then(entries => {
                    const tbody = document.querySelector('#lookup-table tbody');
                    if (entries.length === 0) {
                        tbody.innerHTML = '<tr><td colspan="3" style="text-align:center;color:#666">No data yet</td></tr>';
                        return;
                    }
                    tbody.innerHTML = entries.map(e =>
                        '<tr><td>' + e.distanceMin.toFixed(2) + ' - ' + e.distanceMax.toFixed(2) + '</td>' +
                        '<td>' + e.optimalSpeed.toFixed(0) + '</td>' +
                        '<td>' + e.confirmedTrials + '</td></tr>'
                    ).join('');
                })
                .catch(e => console.error('Lookup error:', e));
        }

        // Poll status every 200ms
        setInterval(updateStatus, 200);

        // Initial load
        updateStatus();
        updateHistory();
        updateLookup();
    </script>
</body>
</html>
        """.trimIndent()
    }
}
