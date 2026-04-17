/**
 * @file dashboard.h
 * @brief Single-file HTML dashboard embedded as a C string literal.
 *
 * The HTML/CSS/JS implements the Wi-Fi CSI Human Radar UI:
 *   - Rotating polar radar sweep
 *   - CSI amplitude waveform
 *   - Status / Variance / Subcarrier HUD
 *   - Event log
 *   - Scan speed + threshold sliders (sends config back via WS)
 *   - Recalibrate button
 */
#pragma once

static const char DASHBOARD_HTML[] =
"<!DOCTYPE html>"
"<html lang=\"en\">"
"<head>"
"<meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>Wi-Fi CSI Human Radar</title>"
"<style>"
"@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@400;700;900&display=swap');"
":root{"
"  --bg:#0a0e1a;"
"  --panel:#0d1424;"
"  --border:#1a2a4a;"
"  --cyan:#00d4ff;"
"  --cyan-dim:#006880;"
"  --red:#ff2d55;"
"  --red-dim:#4a0018;"
"  --green:#00ff9d;"
"  --yellow:#ffe600;"
"  --text:#c8d8f0;"
"  --text-dim:#4a6080;"
"  --font-mono:'Share Tech Mono',monospace;"
"  --font-hud:'Orbitron',sans-serif;"
"}"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{background:var(--bg);color:var(--text);font-family:var(--font-mono);"
"     min-height:100vh;display:flex;flex-direction:column;overflow-x:hidden}"

/* ── top bar ── */
"#topbar{display:flex;align-items:center;justify-content:space-between;"
"  padding:12px 24px;border-bottom:1px solid var(--border);"
"  background:rgba(10,14,26,0.95);backdrop-filter:blur(8px);"
"  position:sticky;top:0;z-index:100}"
"#topbar h1{font-family:var(--font-hud);font-size:1.1rem;font-weight:700;"
"  letter-spacing:0.15em;color:var(--cyan);text-shadow:0 0 18px var(--cyan)}"
"#hud{display:flex;gap:40px;align-items:center}"
".hud-item{display:flex;flex-direction:column;align-items:flex-end}"
".hud-label{font-size:0.6rem;letter-spacing:0.2em;color:var(--text-dim)}"
".hud-value{font-family:var(--font-hud);font-size:1.1rem;font-weight:700;"
"  letter-spacing:0.08em;color:var(--cyan);text-shadow:0 0 10px var(--cyan)}"
"#status-value{font-size:1rem;letter-spacing:0.12em}"
"#status-value.motion{color:var(--red);text-shadow:0 0 14px var(--red)}"
"#status-value.clear{color:var(--green);text-shadow:0 0 10px var(--green)}"
"#hud-act.breathing{color:var(--green)}"
"#hud-act.moving{color:var(--red)}"
"#hud-dir.toward{color:var(--cyan)}"
"#hud-dir.away{color:var(--yellow)}"

/* ── main split ── */
"#main{display:flex;flex:1;gap:0}"

/* ── radar panel ── */
"#radar-panel{flex:1;display:flex;flex-direction:column;"
"  border-right:1px solid var(--border);position:relative;min-height:420px}"
"#radar-info{padding:10px 16px;font-size:0.7rem;line-height:1.8;"
"  color:var(--cyan);border-bottom:1px solid var(--border);"
"  display:flex;justify-content:space-between}"
"#radar-wrap{flex:1;display:flex;align-items:center;justify-content:center;padding:16px}"
"#radar-canvas{display:block;max-width:100%;max-height:360px}"

/* ── bottom section ── */
"#bottom{display:flex;border-top:1px solid var(--border)}"
"#waveform-panel{"
"  flex:1;padding:14px 18px;border-right:1px solid var(--border)}"
"#eventlog-panel{width:420px;padding:14px 18px}"
".panel-title{font-size:0.65rem;letter-spacing:0.3em;color:var(--cyan);"
"  margin-bottom:10px;border-bottom:1px solid var(--border);padding-bottom:6px}"
"#wave-canvas{display:block;width:100%;height:90px;"
"  background:transparent}"

/* ── event log ── */
"#event-table{width:100%;border-collapse:collapse;font-size:0.72rem}"
"#event-table tr{border-bottom:1px solid rgba(26,42,74,0.5)}"
"#event-table td{padding:5px 8px}"
".evnt-id{color:var(--text-dim);font-weight:bold}"
".evnt-type{color:var(--red);font-weight:bold}"
".evnt-type.clear-ev{color:var(--text-dim)}"
".evnt-var{color:var(--red);text-align:right}"
".evnt-var.clear-ev{color:var(--text-dim)}"

/* ── controls ── */
"#controls{display:flex;align-items:center;gap:32px;padding:14px 24px;"
"  border-top:1px solid var(--border);background:rgba(10,14,26,0.8);flex-wrap:wrap}"
".ctrl-group{display:flex;align-items:center;gap:10px;font-size:0.75rem}"
".ctrl-label{color:var(--text-dim);letter-spacing:0.1em}"
"input[type=range]{-webkit-appearance:none;width:160px;height:3px;"
"  background:var(--border);border-radius:2px;outline:none}"
"input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:14px;height:14px;"
"  border-radius:50%;background:var(--cyan);cursor:pointer;"
"  box-shadow:0 0 6px var(--cyan)}"
".ctrl-value{font-family:var(--font-hud);font-size:0.8rem;color:var(--text);"
"  background:var(--border);padding:2px 8px;border-radius:3px;min-width:40px;text-align:center}"
"#btn-recal{margin-left:auto;font-family:var(--font-hud);font-size:0.7rem;"
"  letter-spacing:0.15em;color:var(--cyan);background:transparent;"
"  border:1px solid var(--cyan);padding:7px 20px;border-radius:4px;"
"  cursor:pointer;transition:all 0.2s}"
"#btn-recal:hover{background:var(--cyan);color:var(--bg);"
"  box-shadow:0 0 18px var(--cyan)}"

/* ── raw data strip ── */
"#raw-strip{padding:4px 18px;font-size:0.6rem;color:var(--text-dim);"
"  border-top:1px solid var(--border);letter-spacing:0.05em;overflow:hidden;"
"  white-space:nowrap;text-overflow:ellipsis}"

/* ── motion overlay ── */
"#motion-overlay{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);"
"  pointer-events:none;display:none;flex-direction:column;align-items:center}"
"#motion-badge{font-family:var(--font-hud);font-size:1.3rem;font-weight:900;"
"  letter-spacing:0.25em;color:var(--red);text-shadow:0 0 30px var(--red);"
"  border:2px solid var(--red);padding:8px 28px;"
"  background:rgba(74,0,24,0.6);animation:pulse 0.6s infinite alternate}"
"@keyframes pulse{from{opacity:0.7;box-shadow:0 0 10px var(--red)}"
"                   to{opacity:1.0;box-shadow:0 0 40px var(--red)}}"
"</style>"
"</head>"
"<body>"

/* ── top bar ── */
"<div id=\"topbar\">"
"  <h1>Wi-Fi CSI Human Radar</h1>"
"  <div id=\"hud\">"
"    <div class=\"hud-item\"><span class=\"hud-label\">STATUS</span>"
"      <span class=\"hud-value\" id=\"status-value\">CONNECTING</span></div>"
"    <div class=\"hud-item\"><span class=\"hud-label\">ACTIVITY</span>"
"      <span class=\"hud-value\" id=\"hud-act\">STILL</span></div>"
"    <div class=\"hud-item\"><span class=\"hud-label\">DIRECTION</span>"
"      <span class=\"hud-value\" id=\"hud-dir\">—</span></div>"
"    <div class=\"hud-item\"><span class=\"hud-label\">VARIANCE</span>"
"      <span class=\"hud-value\" id=\"hud-var\">0.00</span></div>"
"  </div>"
"</div>"

/* ── main ── */
"<div id=\"main\">"
"<div id=\"radar-panel\">"

/*   info strip */
"  <div id=\"radar-info\">"
"    <div><span id=\"info-proc\">CONNECTING...</span><br>"
"         <span id=\"info-addr\">SYSTEM_REMOTE_ADDR: —</span></div>"
"    <div style=\"text-align:right\">"
"      <span>CSI MODE: <b style=\"color:var(--cyan)\">ACTIVE</b></span><br>"
"      <span>SUBCARRIERS: <b id=\"info-sc2\">—</b> ACTIVE</span><br>"
"      <span>VAR SCORE: <b id=\"info-var2\">0.00</b></span>"
"    </div>"
"  </div>"

"  <div id=\"radar-wrap\">"
"    <canvas id=\"radar-canvas\" width=\"420\" height=\"420\"></canvas>"
"    <div id=\"motion-overlay\">"
"      <div id=\"motion-badge\">&#9651; MOTION DETECTED &#9651;</div>"
"    </div>"
"  </div>"
"</div>"
"</div>"

/* ── bottom ── */
"<div id=\"bottom\">"
"<div id=\"waveform-panel\">"
"  <div class=\"panel-title\">CSI AMPLITUDE / SUBCARRIERS</div>"
"  <canvas id=\"wave-canvas\"></canvas>"
"  <div id=\"raw-strip\">RAW_DATA: —</div>"
"</div>"
"<div id=\"eventlog-panel\">"
"  <div class=\"panel-title\">EVENT LOG // RECENT ACTIVITY</div>"
"  <table id=\"event-table\"><tbody id=\"event-tbody\"></tbody></table>"
"</div>"
"</div>"

/* ── controls ── */
"<div id=\"controls\">"
"  <div class=\"ctrl-group\">"
"    <span class=\"ctrl-label\">Scan Speed</span>"
"    <input type=\"range\" id=\"sl-speed\" min=\"0.5\" max=\"5\" step=\"0.1\" value=\"1.5\">"
"    <span class=\"ctrl-value\" id=\"lbl-speed\">1.5</span>"
"  </div>"
"  <div class=\"ctrl-group\">"
"    <span class=\"ctrl-label\">Alert Threshold</span>"
"    <input type=\"range\" id=\"sl-thresh\" min=\"5\" max=\"100\" step=\"1\" value=\"20\">"
"    <span class=\"ctrl-value\" id=\"lbl-thresh\">20</span>"
"  </div>"
"  <button id=\"btn-recal\">Recalibrate</button>"
"</div>"

"<script>"
"/* ── WebSocket connection ── */"
"const WS_URL = 'ws://' + location.hostname + '/ws';"
"let ws, reconnectTimer;"
"let scanAngle = 0;"
"let scanSpeed = 1.5;"
"let alertThreshold = 20;"
"let isMotion = false;"
"let blips = [];        /* [{angle, dist, life}] */"
"let lastAmps = [];"
"let events = [];"
"let eventCount = 0;"

"function connect() {"
"  ws = new WebSocket(WS_URL);"
"  ws.onopen  = () => { clearTimeout(reconnectTimer);"
"    document.getElementById('info-proc').textContent = 'PROCESSING...'; };"
"  ws.onclose = () => { reconnectTimer = setTimeout(connect, 2000); };"
"  ws.onerror = () => ws.close();"
"  ws.onmessage = (e) => handleData(JSON.parse(e.data));"
"}"

"const VECTORS ={NONE:'—',TOWARD:'▲ TOWARD',AWAY:'▼ AWAY'};"
"function handleData(d) {"
"  isMotion = d.status === 'MOTION_DETECTED';"
"  const varFmt = (d.variance || 0).toFixed(2);"
"  const act = d.activity || 'STILL';"
"  const dir = d.direction || 'NONE';"

/*   HUD */
"  const sv = document.getElementById('status-value');"
"  sv.textContent = isMotion ? 'MOTION DETECTED' : (act === 'BREATHING' ? 'BIOMETRIC PULSE' : 'CLEAR');"
"  sv.className   = isMotion ? 'motion' : 'clear';"
"  const ha = document.getElementById('hud-act');"
"  ha.textContent = act; ha.className = 'hud-value ' + act.toLowerCase();"
"  const hd = document.getElementById('hud-dir');"
"  hd.textContent = VECTORS[dir]; hd.className = 'hud-value ' + dir.toLowerCase();"
"  document.getElementById('hud-var').textContent  = varFmt;"
"  document.getElementById('info-addr').textContent= 'SYSTEM_REMOTE_ADDR: ' + (d.remote_addr || '—');"
"  document.getElementById('info-var2').textContent= varFmt;"
"  document.getElementById('info-sc2').textContent = d.subcarriers || 64;"

/*   motion overlay */
"  const overlay = document.getElementById('motion-overlay');"
"  overlay.style.display = isMotion ? 'flex' : 'none';"

/*   amplitudes */
"  if (d.amplitudes && d.amplitudes.length) {"
"    lastAmps = d.amplitudes;"
"    drawWaveform(lastAmps);"
"    document.getElementById('raw-strip').textContent ="
"      'RAW_DATA: [' + lastAmps.slice(0,12).map(v=>v.toFixed(1)).join(', ') + ', ...]';"
"  }"

/*   spawn blips on motion */
"  if (isMotion || act === 'BREATHING') {"
"    const count = isMotion ? 3 : 1;"
"    for (let i=0; i<count; i++) {"
"      blips.push({"
"        angle: Math.random() * Math.PI * 2,"
"        dist:  0.25 + Math.random() * 0.65,"
"        life:  1.0,"
"        color: isMotion ? 'var(--red)' : 'var(--green)'"
"      });"
"    }"
"  }"

/*   event log */
"  if (d.event_count && d.event_count > eventCount) {"
"    eventCount = d.event_count;"
"    events.unshift({id: eventCount, type:'Motion Detected', var: varFmt});"
"    if (events.length > 10) events.pop();"
"    renderEvents();"
"  }"
"}"

/* ── event log ── */
"function renderEvents() {"
"  const tbody = document.getElementById('event-tbody');"
"  tbody.innerHTML = events.slice(0,5).map(ev => "
"    '<tr><td class=\"evnt-id\">EVNT '+String(ev.id).padStart(3,'0')+'</td>'"
"       +'<td class=\"evnt-type\">'+ev.type+'</td>'"
"       +'<td class=\"evnt-var\">'+ev.var+' Var</td></tr>'"
"  ).join('');"
"}"

/* ── radar canvas ── */
"const radarCanvas = document.getElementById('radar-canvas');"
"const radarCtx    = radarCanvas.getContext('2d');"
"const CX = 210, CY = 210, R = 190;"

"function drawRadar(ts) {"
"  const ctx = radarCtx;"
"  const W = radarCanvas.width, H = radarCanvas.height;"
"  ctx.clearRect(0, 0, W, H);"

/*   dark background */
"  ctx.fillStyle = '#0a0e1a';"
"  ctx.beginPath(); ctx.arc(CX,CY,R,0,Math.PI*2); ctx.fill();"

/*   range rings */
"  [0.25,0.5,0.75,1.0].forEach(f => {"
"    ctx.beginPath(); ctx.arc(CX,CY,R*f,0,Math.PI*2);"
"    ctx.strokeStyle='rgba(0,100,140,0.35)'; ctx.lineWidth=1; ctx.stroke();"
"  });"

/*   spoke lines */
"  for (let a=0; a<360; a+=30) {"
"    const rad = a * Math.PI/180;"
"    ctx.beginPath();"
"    ctx.moveTo(CX,CY);"
"    ctx.lineTo(CX+R*Math.sin(rad), CY-R*Math.cos(rad));"
"    ctx.strokeStyle='rgba(0,100,140,0.25)'; ctx.lineWidth=1; ctx.stroke();"
"  }"

/*   angle labels */
"  ctx.fillStyle='rgba(0,180,220,0.5)'; ctx.font='11px Share Tech Mono';"
"  ctx.textAlign='center'; ctx.textBaseline='middle';"
"  ['0°','30°','60°','90°','120°','150°','180°','210°','240°','270°','300°','330°'].forEach((lbl,i)=>{"
"    const a=i*30*Math.PI/180; const dist=R+14;"
"    ctx.fillText(lbl, CX+dist*Math.sin(a), CY-dist*Math.cos(a));"
"  });"

/*   sweep arm movement */
"  scanAngle += (scanSpeed * 0.018);"  /* speed multiplier */
"  if (scanAngle > Math.PI*2) scanAngle -= Math.PI*2;"

/*   sweep gradient fan */
"  const fanEnd   = scanAngle;"
"  const fanStart = scanAngle - 0.55;"  /* ~31° fan */
"  const grad = ctx.createConicalGradient ? null : null;"  /* fallback */
"  ctx.save();"
"  ctx.beginPath();"
"  ctx.moveTo(CX,CY);"
"  ctx.arc(CX,CY,R,fanStart-Math.PI/2,fanEnd-Math.PI/2);"
"  ctx.closePath();"
"  const fanGrad = ctx.createLinearGradient(CX,CY,"
"    CX+R*Math.cos(fanEnd-Math.PI/2), CY+R*Math.sin(fanEnd-Math.PI/2));"
"  fanGrad.addColorStop(0,'rgba(0,212,255,0)');"
"  fanGrad.addColorStop(1,'rgba(0,212,255,0.18)');"
"  ctx.fillStyle = fanGrad;"
"  ctx.fill();"
"  ctx.restore();"

/*   sweep arm lead line */
"  ctx.beginPath();"
"  ctx.moveTo(CX,CY);"
"  ctx.lineTo(CX+R*Math.cos(scanAngle-Math.PI/2), CY+R*Math.sin(scanAngle-Math.PI/2));"
"  ctx.strokeStyle='rgba(0,212,255,0.85)'; ctx.lineWidth=2;"
"  ctx.shadowColor='#00d4ff'; ctx.shadowBlur=10;"
"  ctx.stroke();"
"  ctx.shadowBlur=0;"

/*   blips */
"  blips.forEach(b => {"
"    b.life -= 0.012;"
"    if (b.life < 0) return;"
"    const bx = CX + R*b.dist*Math.cos(b.angle-Math.PI/2);"
"    const by = CY + R*b.dist*Math.sin(b.angle-Math.PI/2);"
"    ctx.fillStyle = b.color.replace('var(--red)','#ff2d55').replace('var(--green)','#00ff9d');"
"    ctx.shadowColor = b.color === 'var(--red)' ? '#ff2d55' : '#00ff9d';"
"    ctx.shadowBlur=8;"
"    ctx.fillRect(bx-4,by-10,8,12);"  /* radar blip bar shape */
"    ctx.shadowBlur=0;"
"  });"
"  blips = blips.filter(b=>b.life>0);"

/*   center dot */
"  ctx.beginPath(); ctx.arc(CX,CY,4,0,Math.PI*2);"
"  ctx.fillStyle='#00d4ff'; ctx.fill();"

"  requestAnimationFrame(drawRadar);"
"}"

/* ── waveform canvas ── */
"const waveCanvas = document.getElementById('wave-canvas');"
"const waveCtx    = waveCanvas.getContext('2d');"

"function drawWaveform(amps) {"
"  const W = waveCanvas.clientWidth || 400;"
"  const H = 90;"
"  waveCanvas.width  = W;"
"  waveCanvas.height = H;"
"  const ctx = waveCtx;"
"  ctx.clearRect(0,0,W,H);"
"  ctx.fillStyle='#0a0e1a'; ctx.fillRect(0,0,W,H);"

/*   find max for normalisation */
"  const maxVal = Math.max(...amps, 1);"
"  const step   = W / Math.max(amps.length-1,1);"

"  ctx.beginPath();"
"  amps.forEach((v,i) => {"
"    const x = i*step;"
"    const y = H - (v/maxVal)*(H*0.82) - H*0.06;"
"    i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);"
"  });"
"  ctx.strokeStyle='#00d4ff'; ctx.lineWidth=1.5;"
"  ctx.shadowColor='#00d4ff'; ctx.shadowBlur=6;"
"  ctx.stroke(); ctx.shadowBlur=0;"
"}"

/* ── controls ── */
"const slSpeed  = document.getElementById('sl-speed');"
"const slThresh = document.getElementById('sl-thresh');"
"slSpeed.addEventListener('input', () => {"
"  scanSpeed = parseFloat(slSpeed.value);"
"  document.getElementById('lbl-speed').textContent = slSpeed.value;"
"});"
"slThresh.addEventListener('input', () => {"
"  alertThreshold = parseInt(slThresh.value);"
"  document.getElementById('lbl-thresh').textContent = slThresh.value;"
"  if (ws && ws.readyState===1)"
"    ws.send(JSON.stringify({cmd:'set_threshold',value:alertThreshold}));"
"});"
"document.getElementById('btn-recal').addEventListener('click', function() {"
"  if (ws && ws.readyState===1) {"
"    const btn = this;"
"    const oldText = btn.innerHTML;"
"    btn.innerHTML = 'PROC...';"
"    btn.disabled = true;"
"    ws.send(JSON.stringify({cmd:'recalibrate'}));"
"    setTimeout(() => {"
"      btn.innerHTML = oldText;"
"      btn.disabled = false;"
"    }, 2000);"
"  }"
"});"

/* ── kick off ── */
"connect();"
"requestAnimationFrame(drawRadar);"
"</script>"
"</body></html>";
