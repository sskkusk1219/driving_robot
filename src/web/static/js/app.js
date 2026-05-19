'use strict';

// ── Toast ──────────────────────────────────────────────
const toast = document.getElementById('toast');
let toastTimer = null;
function showToast(msg, type = 'ok') {
  toast.textContent = msg;
  toast.className = `show ${type}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { toast.className = ''; }, 3500);
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
  const toY = v => H - (v / maxSpeed) * H * 0.9 - H * 0.05;
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
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
    data.forEach((v, i) => {
      const x = (i / (MAX_POINTS - 1)) * W;
      i === 0 ? ctx.moveTo(x, toY(v)) : ctx.lineTo(x, toY(v));
    });
    ctx.stroke();
  }
  drawLine(refData, '#ef4444');
  drawLine(actData, '#3b82f6');
}

function pushData(actual, ref) {
  actData.push(actual);
  if (actData.length > MAX_POINTS) actData.shift();
  refData.push(ref ?? 0);
  if (refData.length > MAX_POINTS) refData.shift();
  drawChart();
}

// ── State ───────────────────────────────────────────────
let profiles = [];
let activeProfileId = null;
let robotState = 'BOOTING';

// ── DOM refs ────────────────────────────────────────────
const stateBadge          = document.getElementById('state-badge');
const activeProfileBadge  = document.getElementById('active-profile-badge');
const actualSpeedEl       = document.getElementById('actual-speed');
const refSpeedEl          = document.getElementById('ref-speed');
const accelProgress       = document.getElementById('accel-progress');
const brakeProgress       = document.getElementById('brake-progress');
const accelLabel          = document.getElementById('accel-label');
const brakeLabel          = document.getElementById('brake-label');
const profileListEl       = document.getElementById('profile-list');
const calibResultCard     = document.getElementById('calib-result-card');
const calibResultBody     = document.getElementById('calib-result-body');

// ── WebSocket ───────────────────────────────────────────
let ws = null;
function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws/realtime`);
  ws.onmessage = ev => {
    const d = JSON.parse(ev.data);
    robotState = d.robot_state;
    updateDashboard(d);
  };
  ws.onclose = () => { setTimeout(connectWS, 2000); };
}
connectWS();

function updateDashboard(d) {
  stateBadge.textContent = d.robot_state;
  stateBadge.className = d.robot_state;
  actualSpeedEl.textContent = d.actual_speed_kmh.toFixed(1);
  refSpeedEl.textContent = d.ref_speed_kmh != null ? d.ref_speed_kmh.toFixed(1) : '—';
  const ap = Math.min(100, Math.max(0, d.accel_opening));
  const bp = Math.min(100, Math.max(0, d.brake_opening));
  accelProgress.value = ap; accelLabel.textContent = `${ap.toFixed(1)}%`;
  brakeProgress.value = bp; brakeLabel.textContent = `${bp.toFixed(1)}%`;
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

// ── Profile management ──────────────────────────────────
async function loadProfiles() {
  const data = await api('GET', '/api/v1/profiles/');
  if (!data) return;
  profiles = data;
  renderProfileList();
}

function renderProfileList() {
  if (profiles.length === 0) {
    profileListEl.innerHTML = '<p class="empty-msg">プロファイルがありません。「＋ 新規作成」で作成してください。</p>';
    return;
  }
  profileListEl.innerHTML = profiles.map(p => {
    const isActive = p.id === activeProfileId;
    const calibHtml = p.calibration && p.calibration.is_valid
      ? `<div class="profile-card-calib ok">✓ キャリブレーション済 (${new Date(p.calibration.calibrated_at).toLocaleDateString('ja-JP')})</div>`
      : `<div class="profile-card-calib none">⚠ キャリブレーション未実施</div>`;
    return `
      <div class="profile-card${isActive ? ' active' : ''}" data-id="${p.id}">
        <div class="profile-card-info">
          <div class="profile-card-name">${escHtml(p.name)}${isActive ? ' ✓ 選択中' : ''}</div>
          <div class="profile-card-meta">最高車速 ${p.max_speed} km/h ／ アクセル ${p.max_accel_opening}% ／ ブレーキ ${p.max_brake_opening}%</div>
          ${calibHtml}
        </div>
        <div class="profile-card-actions">
          ${!isActive ? `<button class="btn-primary btn-sm" onclick="selectProfile('${p.id}')">選択</button>` : ''}
          <button class="btn-secondary btn-sm" onclick="openProfileModal('${p.id}')">編集</button>
          <button class="btn-danger btn-sm" onclick="deleteProfile('${p.id}')">削除</button>
        </div>
      </div>`;
  }).join('');
}

async function selectProfile(id) {
  const r = await api('POST', '/api/v1/drive/select-profile', { profile_id: id });
  if (!r) return;
  activeProfileId = id;
  const p = profiles.find(x => x.id === id);
  activeProfileBadge.textContent = p ? `選択中: ${p.name}` : '選択中';
  renderProfileList();
  showToast(`プロファイル「${p?.name}」を選択しました`, 'ok');
}

async function deleteProfile(id) {
  const p = profiles.find(x => x.id === id);
  if (!confirm(`「${p?.name}」を削除しますか？`)) return;
  const res = await fetch(`/api/v1/profiles/${id}`, { method: 'DELETE' });
  if (res.status === 204 || res.ok) {
    if (activeProfileId === id) {
      activeProfileId = null;
      activeProfileBadge.textContent = 'プロファイル未選択';
    }
    showToast('プロファイルを削除しました', 'ok');
    await loadProfiles();
  } else {
    const json = await res.json().catch(() => ({}));
    showToast(`削除失敗: ${json.detail ?? res.statusText}`, 'error');
  }
}

function escHtml(s) {
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

// ── Profile modal ───────────────────────────────────────
const profileModal   = document.getElementById('profile-modal');
const modalTitle     = document.getElementById('modal-title');
const formProfileId  = document.getElementById('form-profile-id');
const formName       = document.getElementById('form-name');
const formMaxAccel   = document.getElementById('form-max-accel');
const formMaxBrake   = document.getElementById('form-max-brake');
const formMaxSpeed   = document.getElementById('form-max-speed');
const formMaxDecel   = document.getElementById('form-max-decel');
const formKp         = document.getElementById('form-kp');
const formKi         = document.getElementById('form-ki');
const formKd         = document.getElementById('form-kd');
const formDevThresh  = document.getElementById('form-dev-threshold');
const formDevDur     = document.getElementById('form-dev-duration');
const formModelPath  = document.getElementById('form-model-path');

function openProfileModal(profileId = null) {
  const p = profileId ? profiles.find(x => x.id === profileId) : null;
  modalTitle.textContent   = p ? 'プロファイル編集' : 'プロファイル作成';
  formProfileId.value      = p?.id ?? '';
  formName.value           = p?.name ?? '';
  formMaxAccel.value       = p?.max_accel_opening ?? 80;
  formMaxBrake.value       = p?.max_brake_opening ?? 80;
  formMaxSpeed.value       = p?.max_speed ?? 120;
  formMaxDecel.value       = p?.max_decel_g ?? 0.3;
  formKp.value             = p?.pid_gains?.kp ?? 1.0;
  formKi.value             = p?.pid_gains?.ki ?? 0.0;
  formKd.value             = p?.pid_gains?.kd ?? 0.0;
  formDevThresh.value      = p?.stop_config?.deviation_threshold_kmh ?? 2.0;
  formDevDur.value         = p?.stop_config?.deviation_duration_s ?? 4.0;
  formModelPath.value      = p?.model_path ?? '';
  profileModal.classList.remove('hidden');
  formName.focus();
}

function closeProfileModal() {
  profileModal.classList.add('hidden');
}

async function saveProfile() {
  if (!formName.value.trim()) {
    showToast('プロファイル名を入力してください', 'error');
    return;
  }
  const payload = {
    name: formName.value.trim(),
    max_accel_opening: parseFloat(formMaxAccel.value),
    max_brake_opening: parseFloat(formMaxBrake.value),
    max_speed: parseFloat(formMaxSpeed.value),
    max_decel_g: parseFloat(formMaxDecel.value),
    pid_gains: { kp: parseFloat(formKp.value), ki: parseFloat(formKi.value), kd: parseFloat(formKd.value) },
    stop_config: {
      deviation_threshold_kmh: parseFloat(formDevThresh.value),
      deviation_duration_s: parseFloat(formDevDur.value),
    },
    model_path: formModelPath.value.trim() || null,
  };
  const id = formProfileId.value;
  let r;
  if (id) {
    r = await api('PUT', `/api/v1/profiles/${id}`, payload);
  } else {
    r = await api('POST', '/api/v1/profiles/', payload);
  }
  if (!r) return;
  showToast(`プロファイル「${r.name}」を${id ? '更新' : '作成'}しました`, 'ok');
  closeProfileModal();
  await loadProfiles();
}

document.getElementById('btn-new-profile').addEventListener('click', () => openProfileModal());
document.getElementById('btn-modal-cancel').addEventListener('click', closeProfileModal);
document.getElementById('btn-modal-save').addEventListener('click', saveProfile);
profileModal.addEventListener('click', e => { if (e.target === profileModal) closeProfileModal(); });

// ── Drive controls ──────────────────────────────────────
document.getElementById('btn-initialize').addEventListener('click', async () => {
  const r = await api('POST', '/api/v1/drive/initialize');
  if (r) showToast('初期化しました', 'ok');
});

document.getElementById('btn-calibrate').addEventListener('click', async () => {
  if (!activeProfileId) {
    showToast('先にプロファイルを選択してください', 'error');
    return;
  }
  if (!confirm('キャリブレーションを実行しますか？\nアクチュエータが動作します。')) return;
  showToast('キャリブレーション実行中...', 'ok');
  const r = await api('POST', '/api/v1/drive/calibrate');
  if (!r) return;
  if (r.success) {
    showToast('キャリブレーション完了', 'ok');
    renderCalibResult(r.data);
    await loadProfiles();
  } else {
    showToast(`キャリブレーション失敗: ${r.error_message}`, 'error');
    calibResultCard.style.display = 'none';
  }
});

function renderCalibResult(data) {
  if (!data) { calibResultCard.style.display = 'none'; return; }
  const items = [
    ['アクセル ゼロ点', `${data.accel_zero_pos} pulse`],
    ['アクセル フル点', `${data.accel_full_pos} pulse`],
    ['アクセル ストローク', `${data.accel_stroke} pulse`],
    ['ブレーキ ゼロ点', `${data.brake_zero_pos} pulse`],
    ['ブレーキ フル点', `${data.brake_full_pos} pulse`],
    ['ブレーキ ストローク', `${data.brake_stroke} pulse`],
  ];
  calibResultBody.innerHTML = items.map(([label, val]) => `
    <div class="calib-item">
      <div class="calib-item-label">${label}</div>
      <div class="calib-item-value">${val}</div>
    </div>`).join('');
  calibResultCard.style.display = '';
}

document.getElementById('btn-auto-start').addEventListener('click', async () => {
  const modeId = prompt('走行モード ID を入力:');
  if (!modeId) return;
  const r = await api('POST', '/api/v1/drive/start', { mode_id: modeId });
  if (r) showToast(`自動走行を開始: セッション ${r.id}`, 'ok');
});

document.getElementById('btn-learning-start').addEventListener('click', async () => {
  const r = await api('POST', '/api/v1/drive/learning/start');
  if (r) showToast(`学習運転を開始: セッション ${r.id}`, 'ok');
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

// ── Init ────────────────────────────────────────────────
loadProfiles();
