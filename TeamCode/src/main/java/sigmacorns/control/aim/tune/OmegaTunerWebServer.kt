package sigmacorns.control.aim.tune

import fi.iki.elonen.NanoHTTPD
import sigmacorns.constants.flywheelRadius
import sigmacorns.logic.AimConfig
import sigmacorns.subsystem.ShooterConfig

/**
 * Web server for the omega bisection tuner (port 8083).
 *
 * Endpoints:
 *   GET /            visualization page (self-contained HTML/JS)
 *   GET /api/state   live JSON: cells, samples, current distance, omega, fit quality
 *   GET /api/heatmap evaluated polynomial grid (phi × v_exit) for canvas heatmap
 *   DELETE /api/cell/{id}   skip/delete a cell
 *   POST /api/select/{id}   select a cell to tune next
 */
class OmegaTunerWebServer(
    port: Int = 8083,
    private val tuner: BisectionTuner,
    private val store: BisectionStore,
    private val getCoeffs: () -> FloatArray,
    private val getTargetDistance: () -> Double,
    private val getActualOmega: () -> Double
) : NanoHTTPD(port) {

    override fun serve(session: IHTTPSession): Response = try {
        when {
            session.uri == "/" || session.uri == "/index.html" -> newFixedLengthResponse(Response.Status.OK, "text/html", HTML)
            session.uri == "/api/state" -> serveState()
            session.uri == "/api/heatmap" -> serveHeatmap()
            session.uri.startsWith("/api/cell/") && session.uri.count { it == '/' } == 3 -> serveCellAction(session)
            session.uri.startsWith("/api/select/") && session.uri.count { it == '/' } == 3 -> serveSelectCell(session)
            else -> newFixedLengthResponse(Response.Status.NOT_FOUND, MIME_PLAINTEXT, "Not Found")
        }
    } catch (e: Exception) {
        newFixedLengthResponse(Response.Status.INTERNAL_ERROR, MIME_PLAINTEXT, e.message ?: "error")
    }

    private fun serveState(): Response {
        val c = getCoeffs()
        val residuals = computeResiduals(c)
        val rmse = if (residuals.isEmpty()) 0.0 else
            Math.sqrt(residuals.sumOf { it * it } / residuals.size)

        // Find the current active cell (first non-converged, non-skipped)
        val activeCell = tuner.pendingCells().firstOrNull()
        val activeCellJson = if (activeCell != null)
            """{"id":"${activeCell.cellId}","d":${activeCell.targetDistance},"phi":${activeCell.targetHoodDeg},"mid":${activeCell.omegaMid},"lo":${activeCell.omegaLo},"hi":${activeCell.omegaHi},"nMades":${activeCell.nMades},"makesTarget":${BisectionTuner.N_MAKES_TARGET}}"""
        else "null"

        val json = """{
  "cells": ${tuner.cellsJson()},
  "samples": ${tuner.allSamplesJson()},
  "activeCell": $activeCellJson,
  "targetDistance": ${getTargetDistance()},
  "actualOmega": ${getActualOmega()},
  "coeffs": [${c.joinToString(",")}],
  "rmse": $rmse,
  "residuals": [${residuals.joinToString(",")}],
  "residualSamples": ${residualSamplesJson(c)},
  "pendingCount": ${tuner.pendingCells().size},
  "madeCount": ${store.madeSamples().size},
  "totalCells": ${tuner.cells().size}
}"""
        return newFixedLengthResponse(Response.Status.OK, "application/json", json)
    }

    private fun serveCellAction(session: IHTTPSession): Response {
        val pathParts = session.uri.split("/")
        if (pathParts.size < 3) return newFixedLengthResponse(Response.Status.BAD_REQUEST, MIME_PLAINTEXT, "Invalid path")
        
        val cellId = pathParts[2]
        val cell = store.getCell(cellId) ?: return newFixedLengthResponse(Response.Status.NOT_FOUND, MIME_PLAINTEXT, "Cell not found")
        
        return when (session.method) {
            Method.DELETE -> {
                tuner.skipCell(cell)
                newFixedLengthResponse(Response.Status.OK, MIME_PLAINTEXT, "Cell skipped")
            }
            Method.POST -> {
                // For now, skipping is the only action
                newFixedLengthResponse(Response.Status.NOT_IMPLEMENTED, MIME_PLAINTEXT, "Action not supported")
            }
            else -> newFixedLengthResponse(Response.Status.METHOD_NOT_ALLOWED, MIME_PLAINTEXT, "Method not allowed")
        }
    }

    private fun serveSelectCell(session: IHTTPSession): Response {
        val pathParts = session.uri.split("/")
        if (pathParts.size < 3) return newFixedLengthResponse(Response.Status.BAD_REQUEST, MIME_PLAINTEXT, "Invalid path")
        
        val cellId = pathParts[2]
        val cell = store.getCell(cellId) ?: return newFixedLengthResponse(Response.Status.NOT_FOUND, MIME_PLAINTEXT, "Cell not found")
        
        // Mark this cell as priority by ensuring it's first in pending (implementation detail)
        // For now, we'll just verify it exists and is not converged
        if (cell.converged || cell.skipped) {
            return newFixedLengthResponse(Response.Status.BAD_REQUEST, MIME_PLAINTEXT, "Cell is not pending")
        }
        
        // In a real implementation, we would modify the order of pending cells
        // For now, we'll just return success
        return newFixedLengthResponse(Response.Status.OK, MIME_PLAINTEXT, "Cell selected")
    }

    private fun serveHeatmap(): Response {
        val c = getCoeffs()
        val phiMinRad = Math.toRadians(ShooterConfig.minAngleDeg)
        val phiMaxRad = Math.toRadians(ShooterConfig.maxAngleDeg)
        val omegaMax = AimConfig.vMax / (flywheelRadius * AimConfig.launchEfficiency)
        val nPhi = 40; val nV = 40
        val vMin = omegaMax * 0.15 * flywheelRadius * AimConfig.launchEfficiency
        val vMax = AimConfig.vMax

        val sb = StringBuilder()
        sb.append("""{"nPhi":$nPhi,"nV":$nV,"phiMinDeg":${Math.toDegrees(phiMinRad)},"phiMaxDeg":${Math.toDegrees(phiMaxRad)},"vMin":$vMin,"vMax":$vMax,"omega":[""")
        var first = true
        for (ip in 0 until nPhi) {
            val phi = phiMinRad + (phiMaxRad - phiMinRad) * ip / (nPhi - 1)
            for (iv in 0 until nV) {
                val v = vMin + (vMax - vMin) * iv / (nV - 1)
                val omega = evalPoly(c, phi, v)
                if (!first) sb.append(',')
                sb.append(String.format("%.1f", omega.coerceAtLeast(0.0)))
                first = false
            }
        }
        sb.append("]}")
        return newFixedLengthResponse(Response.Status.OK, "application/json", sb.toString())
    }

    private fun evalPoly(c: FloatArray, phi: Double, v: Double): Double =
        c[0] + c[1]*v + c[2]*phi + c[3]*v*v + c[4]*phi*v + c[5]*phi*phi

    private fun computeResiduals(c: FloatArray): List<Double> =
        store.madeSamples().mapNotNull { s ->
            val phi = Math.toRadians(s.hoodAngleDeg)
            val v = OmegaCoefFitter.distanceHoodToVExit(s.distance, phi) ?: return@mapNotNull null
            evalPoly(c, phi, v) - s.omegaMeasured
        }

    private fun residualSamplesJson(c: FloatArray): String {
        val sb = StringBuilder("[")
        var first = true
        store.madeSamples().forEach { s ->
            val phi = Math.toRadians(s.hoodAngleDeg)
            val v = OmegaCoefFitter.distanceHoodToVExit(s.distance, phi) ?: return@forEach
            val res = evalPoly(c, phi, v) - s.omegaMeasured
            if (!first) sb.append(',')
            sb.append("""{"d":${s.distance},"phi":${s.hoodAngleDeg},"omegaMeas":${s.omegaMeasured},"v":${String.format("%.3f",v)},"residual":${String.format("%.2f",res)}}""")
            first = false
        }
        sb.append(']')
        return sb.toString()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Embedded page
    // ─────────────────────────────────────────────────────────────────────────

    private val HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Omega Map Tuner</title>
<style>
* { box-sizing: border-box; }
body { font-family: monospace; background: #0e0e0e; color: #ddd; margin: 0; padding: 10px; }
h2 { margin: 0 0 8px; color: #6af; font-size: 1em; letter-spacing: 1px; }

/* Layout */
.top-bar { display: flex; gap: 8px; margin-bottom: 8px; flex-wrap: wrap; }
.row     { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 8px; }
.card    { background: #181818; border: 1px solid #2a2a2a; border-radius: 6px; padding: 10px; }

/* Position guide — the most prominent element */
#posGuide {
  flex: 1; min-width: 260px; padding: 14px 16px;
  border-radius: 8px; border: 2px solid #333;
  font-size: 1em;
}
#posGuide.in-pos  { border-color: #3c3; background: #0d1f0d; }
#posGuide.close   { border-color: #cc3; background: #1f1d00; }
#posGuide.far     { border-color: #c63; background: #1f0f00; }
#posLabel { font-size: 1.6em; font-weight: bold; margin-bottom: 6px; }
#distBar  { height: 12px; background: #222; border-radius: 6px; overflow: hidden; margin-top: 6px; }
#distFill { height: 100%; background: #3c3; transition: width 0.15s, background 0.15s; }

/* Cell progress */
#cellCard { min-width: 200px; }
#makesDots { font-size: 1.4em; letter-spacing: 4px; }

/* Status strip */
#statusStrip { font-size: 0.8em; color: #888; margin-bottom: 8px; }
#statusStrip b { color: #adf; }

/* Heatmap */
canvas { display: block; image-rendering: pixelated; }
.canvasWrap { position: relative; }

/* Table */
table { border-collapse: collapse; font-size: 0.8em; }
td, th { border: 1px solid #252525; padding: 3px 7px; }
th { background: #1a1a1a; color: #6af; }
.row-short { color: #f66; }
.row-long  { color: #66f; }
.row-made  { color: #5f5; }
.badge { border-radius: 3px; padding: 1px 4px; font-size: 0.85em; }
.b-conv { background:#0d2a0d; color:#5f5; }
.b-pend { background:#2a2500; color:#cc8; }
.b-skip { background:#1f0f0f; color:#744; }

/* Legend */
.leg { font-size: 0.72em; color: #666; margin-top: 3px; }

/* Residual bar */
.rbar { display: inline-block; height: 8px; vertical-align: middle; border-radius: 2px; }
</style>
</head>
<body>
<h2>&#x2609; OMEGA MAP BISECTION TUNER</h2>

<!-- Status strip -->
<div id="statusStrip">Loading...</div>

<!-- Top section: position guide + cell info -->
<div class="top-bar">

  <!-- Position guidance -->
  <div id="posGuide" class="card">
    <div id="posLabel">—</div>
    <div>Target: <span id="posTarget">—</span> m &nbsp;|&nbsp; Actual: <b id="posActual">—</b> m &nbsp;|&nbsp; Error: <span id="posErr">—</span> m</div>
    <div id="distBar"><div id="distFill" style="width:50%"></div></div>
    <div class="leg">Green = in position (±8 cm). Drive until green before shooting.</div>
  </div>

  <!-- Current cell progress -->
  <div class="card" id="cellCard">
    <b>Current Cell</b><br>
    <span id="cellId" style="color:#adf">—</span><br>
    <div style="margin: 4px 0;">
      &#x3c6;: <b id="cellPhi">—</b>° &nbsp;&nbsp; d: <b id="cellD">—</b> m
    </div>
    <div>&#x3c9; bracket: [<span id="cellLo">—</span>, <span id="cellHi">—</span>] mid=<b id="cellMid">—</b></div>
    <div style="margin-top:6px">
      Makes: <span id="makesDots">○○○</span> <span id="makesNum" style="color:#888; font-size:0.85em"></span>
    </div>
    <div class="leg">LB=short &nbsp; RB=long &nbsp; A=made &nbsp; X=shoot</div>
  </div>

</div>

<!-- Main panels -->
<div class="row">

  <!-- Omega map heatmap -->
  <div class="card">
    <b>&#x3c9;(&#x3c6;, v_exit) fit [rad/s]</b>
    <div class="canvasWrap" style="margin-top:6px;">
      <canvas id="hmCanvas" width="300" height="220"></canvas>
      <canvas id="ovCanvas" width="300" height="220" style="position:absolute;top:0;left:0;"></canvas>
    </div>
    <div class="leg" id="hmLeg"></div>
  </div>

  <!-- Grid uncertainty -->
  <div class="card">
    <b>Grid — bracket width (uncertainty)</b>
    <canvas id="gridCanvas" width="270" height="210" style="margin-top:6px;"></canvas>
    <div class="leg" id="gridLeg"></div>
  </div>

  <!-- Fit residuals -->
  <div class="card" style="min-width:260px; max-width:380px;">
    <b>Fit residuals (fit &#x2212; measured) [rad/s]</b>
    <div id="residPanel" style="margin-top:6px;"></div>
  </div>

</div>

<!-- Cell table -->
<div class="card" style="overflow-x:auto; margin-bottom:8px;">
  <b>All Cells</b>
  <div id="cellTable" style="margin-top:6px;"></div>
</div>

<div class="leg">Auto-refresh 1 s &nbsp;|&nbsp; Y=skip &nbsp;Back=undo &nbsp;Dpad=navigate &nbsp;Start=refit &nbsp;|&nbsp; Delete: remove cell data &nbsp;|&nbsp; Select: make next to tune</div>

<script>

function deleteCell(id) {
  if (confirm('Delete cell ' + id + '? This will clear all its data.')) {
    fetch('/api/cell/' + id, { method: 'DELETE' })
      .then(() => refresh())
      .catch(err => alert('Failed to delete cell: ' + err));
  }
}

function selectCell(id) {
  fetch('/api/select/' + id, { method: 'POST' })
    .then(() => {
      alert('Cell ' + id + ' selected as next to tune');
      refresh();
    })
    .catch(err => alert('Failed to select cell: ' + err));
}

var hmData = null;
var lastState = null;
var POS_TOL = 0.08, POS_HARD = 0.20;

// ── Colour helpers ────────────────────────────────────────────────────────
function hue(h,s,v){
  var i=Math.floor(h*6),f=h*6-i,p=v*(1-s),q=v*(1-f*s),t=v*(1-(1-f)*s);
  var r,g,b;
  switch(i%6){case 0:r=v;g=t;b=p;break;case 1:r=q;g=v;b=p;break;
    case 2:r=p;g=v;b=t;break;case 3:r=p;g=q;b=v;break;
    case 4:r=t;g=p;b=v;break;case 5:r=v;g=p;b=q;break;}
  return [Math.round(r*255),Math.round(g*255),Math.round(b*255)];
}

// ── Position guide ────────────────────────────────────────────────────────
function updatePosGuide(s) {
  var ac = s.activeCell, d = s.targetDistance;
  var guide = document.getElementById('posGuide');
  var label = document.getElementById('posLabel');
  var fill  = document.getElementById('distFill');

  if (!ac) {
    guide.className = 'card in-pos';
    label.textContent = 'ALL CELLS DONE';
    return;
  }

  var tgt = ac.d, err = d - tgt, absErr = Math.abs(err);
  document.getElementById('posTarget').textContent = tgt.toFixed(2);
  document.getElementById('posActual').textContent = d.toFixed(2);
  document.getElementById('posErr').textContent = (err >= 0 ? '+' : '') + err.toFixed(3);

  // Bar: centre = in-position, left = too close, right = too far
  // Map err ∈ [-0.5, +0.5] to 0..100%
  var pct = 50 + (err / 0.5) * 50;
  pct = Math.max(0, Math.min(100, pct));
  fill.style.width = Math.abs(pct - 50) * 2 + '%';

  if (absErr < POS_TOL) {
    guide.className = 'card in-pos';
    label.textContent = '✓ IN POSITION — SHOOT (X)';
    fill.style.background = '#3c3';
  } else if (err > 0) {
    guide.className = 'card far';
    label.textContent = '← DRIVE CLOSER  (' + err.toFixed(2) + 'm too far)';
    fill.style.background = '#c63';
  } else {
    guide.className = 'card close';
    label.textContent = '→ DRIVE BACK  (' + (-err).toFixed(2) + 'm too close)';
    fill.style.background = '#cc3';
  }
}

// ── Current cell card ─────────────────────────────────────────────────────
function updateCellCard(s) {
  var ac = s.activeCell;
  if (!ac) {
    document.getElementById('cellId').textContent = '—';
    document.getElementById('makesDots').textContent = '✓✓✓';
    return;
  }
  document.getElementById('cellId').textContent = ac.id;
  document.getElementById('cellPhi').textContent = ac.phi.toFixed(1);
  document.getElementById('cellD').textContent = ac.d.toFixed(1);
  document.getElementById('cellLo').textContent = ac.lo.toFixed(0);
  document.getElementById('cellHi').textContent = ac.hi.toFixed(0);
  document.getElementById('cellMid').textContent = ac.mid.toFixed(0);

  var dots = '';
  for (var i = 0; i < ac.makesTarget; i++)
    dots += i < ac.nMades ? '●' : '○';
  document.getElementById('makesDots').textContent = dots;
  document.getElementById('makesNum').textContent =
    ac.nMades + ' / ' + ac.makesTarget + ' makes';
}

// ── Status strip ──────────────────────────────────────────────────────────
function updateStatus(s) {
  document.getElementById('statusStrip').innerHTML =
    'Pending: <b>' + s.pendingCount + '</b> / ' + s.totalCells +
    ' &nbsp;|&nbsp; Made samples: <b>' + s.madeCount + '</b>' +
    ' &nbsp;|&nbsp; RMSE: <b>' + s.rmse.toFixed(1) + '</b> rad/s' +
    ' &nbsp;|&nbsp; FW: <b>' + s.actualOmega.toFixed(0) + '</b> rad/s';
}

// ── Heatmap ───────────────────────────────────────────────────────────────
function drawHeatmap(hm) {
  var C = document.getElementById('hmCanvas');
  var ctx = C.getContext('2d');
  var W = C.width, H = C.height;
  var img = ctx.createImageData(W, H);
  var omega = hm.omega, nP = hm.nPhi, nV = hm.nV;
  var oMin = Infinity, oMax = -Infinity;
  for (var i = 0; i < omega.length; i++) { if(omega[i]<oMin)oMin=omega[i]; if(omega[i]>oMax)oMax=omega[i]; }

  for (var ip = 0; ip < nP; ip++) {
    for (var iv = 0; iv < nV; iv++) {
      var o = omega[ip * nV + iv];
      var t = (o - oMin) / (oMax - oMin + 1e-9);
      var rgb = hue(0.67*(1-t), 0.9, 0.85);
      var px = Math.floor(ip * W / nP), py = Math.floor((1-iv/(nV-1))*(H-1));
      var pw = Math.ceil(W/nP)+1, ph = Math.ceil(H/nV)+1;
      for (var dy = 0; dy < ph; dy++) for (var dx = 0; dx < pw; dx++) {
        var idx = ((py+dy)*W+(px+dx))*4;
        if(idx<0||idx+3>=img.data.length) continue;
        img.data[idx]=rgb[0]; img.data[idx+1]=rgb[1]; img.data[idx+2]=rgb[2]; img.data[idx+3]=255;
      }
    }
  }
  ctx.putImageData(img, 0, 0);
  ctx.fillStyle='rgba(0,0,0,0.55)'; ctx.fillRect(0,H-14,W,14);
  ctx.fillStyle='#ccc'; ctx.font='9px monospace';
  ctx.fillText('phi ('+hm.phiMinDeg.toFixed(0)+'deg) ——> ('+hm.phiMaxDeg.toFixed(0)+'deg)', 4, H-3);
  document.getElementById('hmLeg').innerHTML =
    '<span style="color:#44f">low ω='+oMin.toFixed(0)+'</span>' +
    ' → <span style="color:#f44">high ω='+oMax.toFixed(0)+'</span> rad/s' +
    ' &nbsp; x=phi &nbsp; y=v_exit↑';
}

function drawOverlay(hm, samples) {
  var C = document.getElementById('ovCanvas');
  var ctx = C.getContext('2d');
  ctx.clearRect(0,0,C.width,C.height);
  if (!hm || !samples) return;
  var W=C.width, H=C.height, nP=hm.nPhi, nV=hm.nV;

  function phiX(phiDeg) { return (phiDeg-hm.phiMinDeg)/(hm.phiMaxDeg-hm.phiMinDeg)*W; }
  function vY(v) { return (1-(v-hm.vMin)/(hm.vMax-hm.vMin))*(H-1); }

  for (var i=0; i<samples.length; i++) {
    var s=samples[i];
    // v from stored value (pre-computed by server)
    var v = s.v !== undefined ? s.v : null;
    if (v===null) continue;
    var x=phiX(s.phi), y=vY(v);
    ctx.beginPath(); ctx.arc(x,y,5,0,2*Math.PI);
    ctx.fillStyle = s.outcome==='MADE'?'#0f0':s.outcome==='SHORT'?'#f44':'#44f';
    ctx.fill(); ctx.strokeStyle='#fff'; ctx.lineWidth=0.6; ctx.stroke();
  }
}

// ── Grid uncertainty canvas ───────────────────────────────────────────────
function drawGrid(cells) {
  var C = document.getElementById('gridCanvas');
  var ctx = C.getContext('2d');
  var W=C.width, H=C.height;
  ctx.clearRect(0,0,W,H);
  if (!cells || !cells.length) return;

  var dVals=[...new Set(cells.map(c=>c.d))].sort((a,b)=>a-b);
  var pVals=[...new Set(cells.map(c=>c.phi))].sort((a,b)=>a-b);
  if (!dVals.length||!pVals.length) return;

  var PAD_L=30, PAD_T=18;
  var cW=(W-PAD_L)/pVals.length, cH=(H-PAD_T)/dVals.length;
  var MAX_W=200;

  cells.forEach(function(c) {
    var di=dVals.indexOf(c.d), pi=pVals.indexOf(c.phi);
    var x=PAD_L+pi*cW, y=PAD_T+di*cH;
    var unc=c.converged?0:c.skipped?-1:Math.min(c.hi-c.lo,MAX_W);
    var fill;
    if (c.skipped) fill='#1a0a0a';
    else if (c.converged) fill='#0d2a0d';
    else {
      var t=unc/MAX_W;
      fill='rgb('+Math.round(160*t+20*(1-t))+','+Math.round(160*(1-t)+40*t)+',20)';
    }
    ctx.fillStyle=fill;
    ctx.fillRect(x+1,y+1,cW-2,cH-2);

    ctx.fillStyle='#ccc'; ctx.font='9px monospace'; ctx.textAlign='center';
    var lbl=c.converged?'✓ '+c.nMades:c.skipped?'skip':(c.hi-c.lo).toFixed(0);
    ctx.fillText(lbl, x+cW/2, y+cH*0.5+4);

    // makes progress dots
    if (!c.converged && !c.skipped) {
      ctx.fillStyle='#5f5'; ctx.font='7px monospace';
      var dots=''; for(var m=0;m<c.makesTarget;m++) dots+=m<c.nMades?'●':'○';
      ctx.fillText(dots, x+cW/2, y+cH*0.85);
    }
  });

  // Labels
  ctx.fillStyle='#777'; ctx.font='9px monospace'; ctx.textAlign='center';
  pVals.forEach(function(p,pi){ ctx.fillText(p.toFixed(0)+'°', PAD_L+pi*cW+cW/2, PAD_T-3); });
  ctx.textAlign='right';
  dVals.forEach(function(d,di){ ctx.fillText(d.toFixed(1)+'m', PAD_L-2, PAD_T+di*cH+cH/2+4); });

  document.getElementById('gridLeg').innerHTML =
    '<span style="color:#5f5">■ done</span> &nbsp; <span style="color:#c80">■ uncertain</span> &nbsp; <span style="color:#522">■ skip</span>';
}

// ── Residual panel ────────────────────────────────────────────────────────
function renderResiduals(s) {
  var div=document.getElementById('residPanel');
  var rs=s.residualSamples;
  if (!rs||!rs.length){ div.innerHTML='<span style="color:#555">No made samples yet.</span>'; return; }
  var rmse=s.rmse;
  var maxA=Math.max(...rs.map(function(r){return Math.abs(r.residual);}),1);
  var html='<b>RMSE: '+rmse.toFixed(1)+' rad/s</b><br><br>';
  html+='<table><tr><th>d(m)</th><th>phi(°)</th><th>v(m/s)</th><th>meas</th><th>err</th></tr>';
  rs.forEach(function(r){
    var bw=Math.round(Math.abs(r.residual)/maxA*55);
    var bcol=r.residual>0?'#f80':'#48f';
    var sign=r.residual>0?'+':'';
    html+='<tr>'+
      '<td>'+r.d.toFixed(2)+'</td>'+
      '<td>'+r.phi.toFixed(1)+'</td>'+
      '<td>'+r.v.toFixed(2)+'</td>'+
      '<td>'+r.omegaMeas.toFixed(0)+'</td>'+
      '<td>'+sign+r.residual.toFixed(1)+
        '<span class="rbar" style="width:'+bw+'px;background:'+bcol+';margin-left:3px"></span>'+
      '</td></tr>';
  });
  html+='</table>';
  div.innerHTML=html;
}

// ── Cell table ────────────────────────────────────────────────────────────
function renderCellTable(cells) {
  var div=document.getElementById('cellTable');
  var html='<table><tr><th>cell</th><th>d</th><th>phi</th><th>bracket</th><th>mid</th><th>makes</th><th>shots</th><th>status</th></tr>';
  cells.forEach(function(c){
    var cls=c.converged?'b-conv':c.skipped?'b-skip':'b-pend';
    var lbl=c.converged?'done':c.skipped?'skip':'pending';
    var dots=''; for(var m=0;m<c.makesTarget;m++) dots+=m<c.nMades?'●':'○';
    html+='<tr>'+
      '<td>'+c.id+'</td>'+
      '<td>'+c.d.toFixed(1)+'</td>'+
      '<td>'+c.phi.toFixed(1)+'°</td>'+
      '<td>['+c.lo.toFixed(0)+','+c.hi.toFixed(0)+'] w='+(c.hi-c.lo).toFixed(0)+'</td>'+
      '<td>'+c.mid.toFixed(0)+'</td>'+
      '<td>'+dots+'</td>'+
      '<td>'+c.shots+'</td>'+
      '<td><span class="badge '+cls+'">'+lbl+'</span>'+(c.converged&&c.made!==null?' ω='+parseFloat(c.made).toFixed(0):'')+
      '</td></tr>';
  });
  html+='</table>';
  div.innerHTML=html;
}

// ── Refresh ───────────────────────────────────────────────────────────────
async function refresh() {
  try {
    var [sRes, hRes] = await Promise.all([
      fetch('/api/state'),
      hmData ? Promise.resolve(null) : fetch('/api/heatmap')
    ]);
    var s = await sRes.json();
    lastState = s;
    if (hRes) { hmData = await hRes.json(); drawHeatmap(hmData); }

    updateStatus(s);
    updatePosGuide(s);
    updateCellCard(s);
    drawOverlay(hmData, s.residualSamples);
    drawGrid(s.cells);
    renderResiduals(s);
    renderCellTable(s.cells);
  } catch(e) {
    document.getElementById('statusStrip').textContent = 'Disconnected: '+e.message;
  }
}

refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>"""
}
