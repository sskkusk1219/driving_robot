'use strict';

// ── Toast ──────────────────────────────────────────────
const toast = document.getElementById('toast');
let toastTimer = null;
function showToast(msg, type = 'ok') {
  toast.textContent = msg;
  toast.className = `show ${type}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { toast.className = ''; }, 3000);
}

// ── Chart (Canvas) ─────────────────────────────────────
const canvas = document.getElementById('speed-chart');
const ctx = canvas.getContext('2d');
const MAX_POINTS = 150;
const refData = [];
const actData = [];

function resizeCanvas() {
  canvas.width = canvas.offsetWidth;
  canvas.height = canvas.offsetHeight;
}
resizeCanvas();
window.addEventListener('resize', resizeCanvas);

function drawChart() {
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  const maxSpeed = 120;
  function toY(v) { return H - (v / maxSpeed) * H * 0.9 - H * 0.05; }
  function toX(i, len) { return (i / (MAX_POINTS - 1)) * W; }

  // grid
  ctx.strokeStyle = '#2d3148';
  ctx.lineWidth = 1;
  for (let v = 0; v <= maxSpeed; v += 20) {
    const y = toY(v);
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    ctx.fillStyle = '#4b5563';
    ctx.fillText(`${v}`, 4, y - 2);
  }

  function drawLine(data, color) {
    if (data.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    data.forEach((v, i) => {
      const x = toX(i, data.length);
      const y = toY(v);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  drawLine(refData, '#ef4444');  // ref: red
  drawLine(actData, '#3b82f6');  // actual: blue
}

function pushData(actual, ref) {
  actData.push(actual);
  if (actData.length > MAX_POINTS) actData.shift();
  refData.push(ref ?? 0);
  if (refData.length > MAX_POINTS) refData.shift();
  drawChart();
}

// ── DOM refs ────────────────────────────────────────────
const stateBadge      = document.getElementById('state-badge');
const actualSpeedEl   = document.getElementById('actual-speed');
const refSpeedEl      = document.getElementById('ref-speed');
const accelProgress   = document.getElementById('accel-progress');
const brakeProgress   = document.getElementById('brake-progress');
const accelLabel      = document.getElementById('accel-label');
const brakeLabel      = document.getElementById('brake-label');

// ── WebSocket ───────────────────────────────────────────
let ws = null;
function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws/realtime`);

  ws.onmessage = (ev) => {
    const d = JSON.parse(ev.data);
    updateUI(d);
  };

  ws.onclose = () => {
    setTimeout(connectWS, 2000);
  };
}
connectWS();

function updateUI(d) {
  // State badge
  stateBadge.textContent = d.robot_state;
  stateBadge.className = d.robot_state;

  // Speed values
  actualSpeedEl.textContent = d.actual_speed_kmh.toFixed(1);
  refSpeedEl.textContent   = d.ref_speed_kmh != null ? d.ref_speed_kmh.toFixed(1) : '—';

  // Gauges
  const accelPct = Math.min(100, Math.max(0, d.accel_opening));
  const brakePct = Math.min(100, Math.max(0, d.brake_opening));
  accelProgress.value = accelPct;
  brakeProgress.value = brakePct;
  accelLabel.textContent = `${accelPct.toFixed(1)}%`;
  brakeLabel.textContent = `${brakePct.toFixed(1)}%`;

  pushData(d.actual_speed_kmh, d.ref_speed_kmh);
}

// ── API helper ──────────────────────────────────────────
async function api(method, path, body = null) {
  try {
    const opts = { method, headers: { 'Content-Type': 'application/json' } };
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(path, opts);
    const json = await res.json().catch(() => ({}));
    if (!res.ok) {
      showToast(`Error ${res.status}: ${json.detail ?? res.statusText}`, 'error');
      return null;
    }
    return json;
  } catch (e) {
    showToast(`Network error: ${e.message}`, 'error');
    return null;
  }
}

// ── Buttons ─────────────────────────────────────────────
document.getElementById('btn-initialize').addEventListener('click', async () => {
  const r = await api('POST', '/api/v1/drive/initialize');
  if (r) showToast('初期化しました', 'ok');
});

document.getElementById('btn-auto-start').addEventListener('click', async () => {
  const modeId = prompt('走行モード ID を入力:');
  if (!modeId) return;
  const r = await api('POST', '/api/v1/drive/start', { mode_id: modeId });
  if (r) showToast(`自動走行を開始: セッション ${r.id}`, 'ok');
});

document.getElementById('btn-stop').addEventListener('click', async () => {
  const r = await api('POST', '/api/v1/drive/stop');
  if (r) showToast('停止しました', 'ok');
});

document.getElementById('btn-manual-start').addEventListener('click', async () => {
  const r = await api('POST', '/api/v1/drive/manual/start');
  if (r) showToast(`手動操作を開始: セッション ${r.id}`, 'ok');
});

document.getElementById('btn-manual-stop').addEventListener('click', async () => {
  const r = await api('POST', '/api/v1/drive/manual/stop');
  if (r) showToast('手動操作を終了しました', 'ok');
});

document.getElementById('btn-emergency').addEventListener('click', async () => {
  if (!confirm('緊急停止しますか？')) return;
  const r = await api('POST', '/api/v1/drive/emergency');
  if (r) showToast('緊急停止しました', 'error');
});

document.getElementById('btn-reset-emergency').addEventListener('click', async () => {
  const r = await api('POST', '/api/v1/drive/reset-emergency');
  if (r) showToast('非常停止をリセットしました', 'ok');
});
