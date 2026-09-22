// Control page served by the ESP32 at http://192.168.4.1 (see main.cpp for the WebSocket messages).
#pragma once
#include <Arduino.h>

static const char WEB_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Hoverboard</title>
<style>
  :root { --bg:#14171c; --panel:#1f242c; --line:#343b46; --text:#e8ecf1; --dim:#8b95a3;
          --accent:#3aa0ff; --ok:#3ecf7a; --bad:#ff5a5a; }
  * { box-sizing:border-box; -webkit-user-select:none; user-select:none; touch-action:none; }
  body { margin:0; background:var(--bg); color:var(--text); font:15px system-ui, sans-serif;
         display:flex; flex-direction:column; align-items:center; min-height:100vh; padding:12px 16px; }
  .bar { width:100%; max-width:480px; display:flex; justify-content:space-between; align-items:center;
         gap:8px; margin-bottom:10px; }
  .dot { display:inline-block; width:10px; height:10px; border-radius:50%; background:var(--bad);
         margin-right:6px; vertical-align:middle; }
  .dot.on { background:var(--ok); }
  .tabs { display:flex; gap:6px; }
  .tabs button { background:var(--panel); color:var(--dim); border:1px solid var(--line);
                 border-radius:8px; padding:8px 14px; font-size:15px; }
  .tabs button.sel { color:var(--text); border-color:var(--accent); }
  .area { width:100%; max-width:480px; height:min(62vh, 440px); display:flex;
          justify-content:center; align-items:center; gap:24px; }
  #pad { width:min(86vw, 380px); height:min(86vw, 380px); border-radius:50%; background:var(--panel);
         border:1px solid var(--line); position:relative; }
  .track { width:110px; height:100%; border-radius:18px; background:var(--panel);
           border:1px solid var(--line); position:relative; }
  .knob { position:absolute; width:84px; height:84px; border-radius:50%; background:var(--accent);
          left:50%; top:50%; transform:translate(-50%,-50%); opacity:.9; pointer-events:none; }
  .track .knob { width:90px; height:56px; border-radius:14px; }
  .axis { position:absolute; background:var(--line); pointer-events:none; }
  .wheels { width:100%; max-width:480px; display:grid; grid-template-columns:1fr 1fr; gap:8px; margin:10px 0; }
  .wheel { background:var(--panel); border:1px solid var(--line); border-radius:10px; padding:8px 10px; }
  .wheel b { font-size:22px; }
  .dim { color:var(--dim); font-size:13px; }
  .row { width:100%; max-width:480px; display:flex; align-items:center; gap:10px; }
  input[type=range] { flex:1; touch-action:auto; }
  #stop { width:100%; max-width:480px; margin-top:10px; padding:16px; font-size:20px; font-weight:700;
          color:#fff; background:#c62828; border:none; border-radius:12px; }
  .hidden { display:none !important; }
</style>
</head>
<body>
  <div class="bar">
    <div><span id="link" class="dot"></span><span id="linktext">connecting</span></div>
    <div class="tabs"><button id="tabJ" class="sel">Joystick</button><button id="tabT">Tank</button></div>
  </div>

  <div id="joy" class="area">
    <div id="pad">
      <div class="axis" style="left:50%;top:8%;bottom:8%;width:1px"></div>
      <div class="axis" style="top:50%;left:8%;right:8%;height:1px"></div>
      <div class="knob" id="padKnob"></div>
    </div>
  </div>
  <div id="tank" class="area hidden">
    <div class="track" id="trackL"><div class="axis" style="top:50%;left:10%;right:10%;height:1px"></div><div class="knob" id="knobL"></div></div>
    <div class="track" id="trackR"><div class="axis" style="top:50%;left:10%;right:10%;height:1px"></div><div class="knob" id="knobR"></div></div>
  </div>

  <div class="wheels">
    <div class="wheel"><span class="dot" id="okL"></span>Left<br><b id="rpmL">--</b> rpm
      <div class="dim">cmd <span id="cmdL">0</span> &middot; iq <span id="iqL">0</span> A</div></div>
    <div class="wheel"><span class="dot" id="okR"></span>Right<br><b id="rpmR">--</b> rpm
      <div class="dim">cmd <span id="cmdR">0</span> &middot; iq <span id="iqR">0</span> A</div></div>
  </div>
  <div class="row"><span>Max</span><input id="max" type="range" min="20" max="300" step="10" value="100">
    <span><b id="maxv">100</b> rpm</span></div>
  <div class="row dim" style="margin-top:6px"><span>Battery <b id="volt">--</b> V</span></div>
  <button id="stop">STOP</button>

<script>
  const $ = id => document.getElementById(id);
  let ws = null, mode = 'j', joy = {x:0, y:0}, tank = {l:0, r:0};

  function connect() {
    ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onopen = () => { setLink(true); ws.send('m ' + $('max').value); };
    ws.onclose = () => { setLink(false); setTimeout(connect, 1000); };
    ws.onmessage = e => { try { show(JSON.parse(e.data)); } catch (err) {} };
  }
  function setLink(on) { $('link').classList.toggle('on', on); $('linktext').textContent = on ? 'connected' : 'reconnecting'; }
  function send(msg) { if (ws && ws.readyState === 1) ws.send(msg); }

  // Input is re-sent every 100 ms; the ESP32 stops the wheels when it hears nothing for 300 ms.
  setInterval(() => {
    if (mode === 'j') send('j ' + Math.round(joy.x * 1000) + ' ' + Math.round(joy.y * 1000));
    else send('t ' + Math.round(tank.l * 1000) + ' ' + Math.round(tank.r * 1000));
  }, 100);

  function show(d) {
    for (const [k, w] of [['L', d.L], ['R', d.R]]) {
      $('ok' + k).classList.toggle('on', w.ok);
      $('rpm' + k).textContent = w.ok ? w.rpm.toFixed(0) : '--';
      $('cmd' + k).textContent = w.cmd;
      $('iq' + k).textContent = w.iq.toFixed(2);
    }
    const v = d.L.ok ? d.L.v : (d.R.ok ? d.R.v : null);
    $('volt').textContent = v === null ? '--' : v.toFixed(1);
  }

  // Joystick: pointer inside the pad, springs back to the centre on release.
  function stick(el, knob, onMove, vertical) {
    let id = null;
    const move = e => {
      const r = el.getBoundingClientRect();
      let x = (e.clientX - r.left) / r.width * 2 - 1, y = 1 - (e.clientY - r.top) / r.height * 2;
      if (vertical) { x = 0; y = Math.max(-1, Math.min(1, y)); }
      else { const m = Math.hypot(x, y); if (m > 1) { x /= m; y /= m; } }
      knob.style.left = (50 + x * 50 * (vertical ? 0 : 0.78)) + '%';
      knob.style.top = (50 - y * 50 * (vertical ? 0.8 : 0.78)) + '%';
      onMove(x, y);
    };
    const end = e => {
      if (e.pointerId !== id) return;
      id = null; knob.style.left = '50%'; knob.style.top = '50%'; onMove(0, 0);
    };
    el.addEventListener('pointerdown', e => { id = e.pointerId; el.setPointerCapture(id); move(e); });
    el.addEventListener('pointermove', e => { if (e.pointerId === id) move(e); });
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
  }
  stick($('pad'), $('padKnob'), (x, y) => { joy.x = x; joy.y = y; }, false);
  stick($('trackL'), $('knobL'), (x, y) => { tank.l = y; }, true);
  stick($('trackR'), $('knobR'), (x, y) => { tank.r = y; }, true);

  function setMode(m) {
    mode = m; joy = {x:0, y:0}; tank = {l:0, r:0};
    $('joy').classList.toggle('hidden', m !== 'j'); $('tank').classList.toggle('hidden', m !== 't');
    $('tabJ').classList.toggle('sel', m === 'j'); $('tabT').classList.toggle('sel', m === 't');
  }
  $('tabJ').onclick = () => setMode('j');
  $('tabT').onclick = () => setMode('t');
  $('max').oninput = () => { $('maxv').textContent = $('max').value; send('m ' + $('max').value); };
  $('stop').onclick = () => { joy = {x:0, y:0}; tank = {l:0, r:0}; send('s'); };
  document.addEventListener('visibilitychange', () => { if (document.hidden) { joy = {x:0, y:0}; tank = {l:0, r:0}; send('s'); } });
  connect();
</script>
</body>
</html>
)HTML";
