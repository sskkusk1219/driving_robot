// ── localStorage helpers ──────────────────────────────────
function lsGet(key, fallback = null) {
  try { return localStorage.getItem(key) ?? fallback; } catch { return fallback; }
}
function lsSet(key, value) {
  try {
    if (value == null) localStorage.removeItem(key);
    else localStorage.setItem(key, value);
  } catch {}
}

// ── 操作権のためのウィンドウの印 ─────────────────────────────
// client_id: タブごと・読み直しで残る（sessionStorage）。instance_id: 読み込みのたびに新しく作る（メモリのみ）。
// LAN 内の http 配信では crypto.randomUUID が使えないことがあるので、代わりの作り方を持つ
function makeUuid() {
  try {
    if (window.crypto && typeof window.crypto.randomUUID === 'function') return window.crypto.randomUUID();
    if (window.crypto && typeof window.crypto.getRandomValues === 'function') {
      const b = window.crypto.getRandomValues(new Uint8Array(16));
      return Array.from(b, x => x.toString(16).padStart(2, '0')).join('');
    }
  } catch (e) { /* 下の代わりを使う */ }
  return `${Date.now().toString(16)}-${Math.random().toString(16).slice(2)}-${Math.random().toString(16).slice(2)}`;
}
function newClientId() {
  const id = makeUuid();
  try { sessionStorage.setItem('drv_client_id', id); } catch (e) { /* 保存できなくてもこのページの間は使える */ }
  window.__drvClientId = id;
  return id;
}
(function initControlIds() {
  let id = null;
  try { id = sessionStorage.getItem('drv_client_id'); } catch (e) { /* 無視 */ }
  if (id) window.__drvClientId = id; else newClientId();
  window.__drvInstanceId = makeUuid();
})();
// 変更系の API・WebSocket に付けるヘッダ（サーバーが操作権の持ち主か見る）
window.drvControlHeaders = function (extra) {
  return { ...(extra || {}), 'X-Client-Id': window.__drvClientId, 'X-Instance-Id': window.__drvInstanceId };
};
const PING_INTERVAL_MS = 2000; // サーバーの PING_INTERVAL_S と同じ

// ── Global state context ──────────────────────────────────
const AppContext = React.createContext(null);

const REALTIME_BUF_SIZE = 800; // ~80 seconds at 100ms
const INIT_REALTIME = {
  robot_state: 'BOOTING',
  actual_speed_kmh: 0,
  ref_speed_kmh: 0,
  accel_opening: 0,
  brake_opening: 0,
  accel_current_ma: 0,
  brake_current_ma: 0,
  init_steps: [],
};

// ── ブラウザ側のエラーをサーバーのエラーページへ記録する ─────────────────
// 記録は制御に影響させない: 待たず、失敗してもトーストを出さない（出すとまた記録して無限ループになる）
function reportClientError(message, kind, stack) {
  try {
    const text = String(message ?? '').slice(0, 4000);
    if (!text || text.startsWith('ResizeObserver loop')) return;
    fetch('/api/v1/errors/client', {
      method: 'POST',
      headers: window.drvControlHeaders({ 'Content-Type': 'application/json' }),
      body: JSON.stringify({
        message: text,
        kind,
        stack: stack ? String(stack).slice(0, 8000) : null,
        url: String(location.href).slice(0, 500),
      }),
      keepalive: true,
    }).catch(() => {});
  } catch (e) { /* 記録できなくても何もしない */ }
}
window.addEventListener('error', ev => {
  const where = ev.filename ? ` (${ev.filename}:${ev.lineno})` : '';
  reportClientError(`${ev.message || '画面のエラー'}${where}`, 'onerror', ev.error && ev.error.stack);
});
window.addEventListener('unhandledrejection', ev => {
  const r = ev.reason;
  reportClientError(`未処理の Promise の失敗: ${r && r.message ? r.message : String(r)}`, 'unhandledrejection', r && r.stack);
});

// ── Toast system ──────────────────────────────────────────
function showToast(msg, type = 'info') {
  if (type === 'error') reportClientError(msg, 'toast');
  const container = document.getElementById('toast-container');
  const el = document.createElement('div');
  el.className = `toast${type === 'error' ? ' error' : type === 'success' ? ' success' : ''}`;
  el.textContent = msg;
  container.appendChild(el);
  // エラーは「何が起きたか・次にすること」を読む時間を長めに取る
  setTimeout(() => el.remove(), type === 'error' ? 9000 : 3500);
}
window.showToast = showToast;

// ── API client ────────────────────────────────────────────
async function apiFetch(method, path, body = null, isFormData = false) {
  try {
    const opts = { method, headers: window.drvControlHeaders() };
    if (body) {
      if (isFormData) {
        opts.body = body;
      } else {
        opts.headers = window.drvControlHeaders({ 'Content-Type': 'application/json' });
        opts.body = JSON.stringify(body);
      }
    }
    const res = await fetch(path, opts);
    if (res.status === 204) return {};
    const json = await res.json().catch(() => ({}));
    if (res.status === 403) {
      // 閲覧用のウィンドウ（操作権なし）からの操作
      showToast(json.detail || '閲覧用のウィンドウでは操作できません', 'error');
      return null;
    }
    if (!res.ok) {
      // 409/422/423/503 などは「何が起きたか」と「次にすること」の 2 行にする（job-utils.js）
      const e = window.describeApiError(res.status, json.detail, res.statusText);
      showToast(e.next ? `${e.what}\n次にすること: ${e.next}` : e.what, 'error');
      return null;
    }
    return json;
  } catch (e) {
    showToast(`ネットワークエラー: ${e.message}`, 'error');
    return null;
  }
}
window.apiFetch = apiFetch;

// ── Root App ──────────────────────────────────────────────
function App() {
  const { useState, useEffect, useRef, useCallback } = React;

  // ホーム画面は廃止（保存されていた 'home' は初期化へ）。既定の画面は初期化
  const [nav, setNav] = useState(() => { const v = lsGet('drv_nav', 'init'); return v === 'home' ? 'init' : v; });
  const [robotState, setRobotState] = useState('BOOTING');
  const [activeProfileId, setActiveProfileId] = useState(() => lsGet('drv_profile_id', null));
  const [activeProfileName, setActiveProfileName] = useState(() => lsGet('drv_profile_name', null));
  const [activeModeId, setActiveModeId] = useState(() => lsGet('drv_mode_id', null));
  const [activeModeName, setActiveModeName] = useState(() => lsGet('drv_mode_name', null));
  // 'mode'（走行モード）| 'schedule'（タイムスケジュール）。TopBar の mode 表示と
  // 自動運転ページでどちらの開始/停止APIを叩くかを切り替えるために使う。
  const [activeModeKind, setActiveModeKind] = useState(() => lsGet('drv_mode_kind', 'mode'));
  const [upsLoss, setUpsLoss] = useState(false);
  const [upsInstalled, setUpsInstalled] = useState(true); // devices.ups（false なら右上に UPS 表示を出さない）
  const [navLock, setNavLock] = useState(false);
  const [realtimeData, setRealtimeData] = useState(INIT_REALTIME);
  // 操作権: null（未決定）| 'operator' | 'viewer'。chooser は「操作」「閲覧」を選ぶ画面を出すか
  const [controlRole, setControlRoleState] = useState(null);
  const [chooser, setChooser] = useState(false);
  const controlRoleRef = useRef(null);
  const setControlRole = useCallback(role => {
    controlRoleRef.current = role;
    window.__drvViewer = role === 'viewer';
    setControlRoleState(role);
  }, []);
  const realtimeBuf = useRef([]);

  // ── Persist nav / profile / mode to localStorage ─────────
  useEffect(() => { lsSet('drv_nav', nav); }, [nav]);
  useEffect(() => { lsSet('drv_profile_id', activeProfileId); }, [activeProfileId]);
  useEffect(() => { lsSet('drv_profile_name', activeProfileName); }, [activeProfileName]);
  useEffect(() => { lsSet('drv_mode_id', activeModeId); }, [activeModeId]);
  useEffect(() => { lsSet('drv_mode_name', activeModeName); }, [activeModeName]);
  useEffect(() => { lsSet('drv_mode_kind', activeModeKind); }, [activeModeKind]);

  // ── System state screen override ─────────────────────────
  const [systemScreen, setSystemScreen] = useState(null);

  const wsRef = useRef(null);
  const reconnectTimer = useRef(null);

  const connectWS = useCallback(() => {
    if (wsRef.current && wsRef.current.readyState < 2) return;
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const q = `client_id=${encodeURIComponent(window.__drvClientId)}&instance_id=${encodeURIComponent(window.__drvInstanceId)}`;
    const ws = new WebSocket(`${proto}://${location.host}/ws/realtime?${q}`);
    wsRef.current = ws;
    let pingTimer = null;

    // 読み直し（F5）・つなぎ直しでバッファが空のとき、サーバーが覚えている最近の値で埋める（過去の軌跡が消えないように）
    ws.onopen = async () => {
      // 操作権の持ち主は 2 s ごとの ping で生きていると示す（切れた接続の再接続でも毎回張り直す）
      pingTimer = setInterval(() => { try { if (ws.readyState === 1) ws.send('ping'); } catch (e) { /* 無視 */ } }, PING_INTERVAL_MS);
      if (realtimeBuf.current.length > 0) return;
      try {
        const res = await fetch('/api/v1/drive/history');
        if (!res.ok) return;
        const h = await res.json();
        const now = Date.now();
        const past = (h.samples ?? []).map(p => ({ ...p, ts: now - p.age_ms }));
        if (past.length > 0 && realtimeBuf.current.length < past.length) {
          realtimeBuf.current = past.concat(realtimeBuf.current.filter(d => d.ts > past[past.length - 1].ts));
        }
      } catch (e) { /* 取れなければ今からの値だけで描く */ }
    };

    ws.onmessage = ev => {
      let d;
      try { d = JSON.parse(ev.data); } catch { return; }
      // 接続直後の操作権の判定（配信本体ではないのでバッファには入れない）
      if (d.type === 'control') {
        if (d.role === 'operator') {
          setControlRole('operator');
          setChooser(false);
        } else if (d.role === 'duplicate') {
          // タブの複製: 別の印を作り直してつなぎ直し、選ぶ画面を出す
          newClientId();
          setControlRole(null);
          ws.close();
        } else if (controlRoleRef.current === 'operator') {
          // 操作権が切れていた（長いネット断など）。自動では取り直さない
          setControlRole('viewer');
          setChooser(false);
          showToast('操作権が切れたため閲覧に切り替えました。操作するには読み直して「操作」を選んでください', 'info');
        } else if (controlRoleRef.current === null) {
          setChooser(true);
        }
        return;
      }
      const state = d.robot_state;
      setRobotState(state);
      setUpsLoss(d.ups_on_battery ?? false);
      setRealtimeData(d);

      // push to ring buffer
      const buf = realtimeBuf.current;
      buf.push({ ...d, ts: Date.now() });
      if (buf.length > REALTIME_BUF_SIZE) buf.splice(0, buf.length - REALTIME_BUF_SIZE);

      // automatic screen transitions based on robot state
      setSystemScreen(prev => {
        if (state === 'AC_LOSS') return 'ac_loss';
        if (state === 'EMERGENCY') return null; // handled inline
        if (state === 'BOOTING') return 'booting';
        if (state === 'ERROR') return 'error';
        return null;
      });
    };

    ws.onclose = () => {
      clearInterval(pingTimer);
      clearTimeout(reconnectTimer.current);
      reconnectTimer.current = setTimeout(connectWS, 2000);
    };
    ws.onerror = () => ws.close();
  }, []);

  useEffect(() => {
    connectWS();
    apiFetch('GET', '/api/v1/drive/status').then(async d => {
      if (!d) return;
      setRobotState(d.robot_state);
      setUpsInstalled(d.devices?.ups !== false);
      const serverProfileId = d.active_profile_id ?? null;
      if (serverProfileId) {
        setActiveProfileId(serverProfileId);
        if (serverProfileId !== lsGet('drv_profile_id')) {
          const profile = await apiFetch('GET', `/api/v1/profiles/${serverProfileId}`);
          if (profile) {
            setActiveProfileName(profile.name);
          } else {
            setActiveProfileId(null);
            setActiveProfileName(null);
          }
        }
      } else {
        setActiveProfileId(null);
        setActiveProfileName(null);
      }
    });
    return () => {
      if (wsRef.current) wsRef.current.close();
      clearTimeout(reconnectTimer.current);
    };
  }, [connectWS]);

  // 選んでいる走行モード・スケジュール・シーケンスがサーバーにまだ有るか確かめる（段 D）。
  // 選択はブラウザの localStorage だけにあり、別のブラウザで消されることがある。
  // トーストを出さないよう素の fetch を使う。無ければ選択を外し、名前が変わっていれば直す
  const recheckSelection = useCallback(async () => {
    const id = lsGet('drv_mode_id', null);
    if (!id) return;
    const kind = lsGet('drv_mode_kind', 'mode');
    const base = kind === 'schedule' ? 'schedules' : kind === 'sequence' ? 'sequences' : 'modes';
    try {
      const res = await fetch(`/api/v1/${base}/${encodeURIComponent(id)}`);
      if (res.status === 404 || res.status === 400 || res.status === 422) {
        setActiveModeId(null);
        setActiveModeName(null);
      } else if (res.ok) {
        const d = await res.json();
        if (d?.name && d.name !== lsGet('drv_mode_name', null)) setActiveModeName(d.name);
      }
    } catch (e) { /* 通信できないときは判断しない */ }
  }, []);
  useEffect(() => { recheckSelection(); }, [recheckSelection]);

  const ctx = {
    recheckSelection,
    nav, setNav,
    robotState,
    activeProfileId, setActiveProfileId,
    activeProfileName, setActiveProfileName,
    activeModeId, setActiveModeId,
    activeModeName, setActiveModeName,
    activeModeKind, setActiveModeKind,
    upsLoss,
    navLock, setNavLock,
    realtimeData,
    realtimeBuf,
    controlRole,
    showToast,
    apiFetch,
  };

  // ── 操作・閲覧の選択 ─────────────────────────────────────
  const [controlHeld, setControlHeld] = useState(false);
  useEffect(() => {
    if (realtimeData?.control_held !== undefined) setControlHeld(!!realtimeData.control_held);
  }, [realtimeData?.control_held]);
  const refreshControl = useCallback(async () => {
    try {
      const q = `client_id=${encodeURIComponent(window.__drvClientId)}&instance_id=${encodeURIComponent(window.__drvInstanceId)}`;
      const res = await fetch(`/api/v1/control?${q}`);
      if (res.ok) { const s = await res.json(); setControlHeld(!!s.held && !s.mine); }
    } catch (e) { /* 取れなければ配信の値のまま */ }
  }, []);
  useEffect(() => { if (chooser) refreshControl(); }, [chooser]);
  const claimControl = useCallback(async () => {
    try {
      const res = await fetch('/api/v1/control/claim', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ client_id: window.__drvClientId, instance_id: window.__drvInstanceId }),
      });
      if (res.ok) { setControlRole('operator'); setChooser(false); return; }
      const j = await res.json().catch(() => ({}));
      showToast(j.detail || '操作権を取れませんでした', 'info');
    } catch (e) {
      showToast(`ネットワークエラー: ${e.message}`, 'error');
    }
    refreshControl();
  }, []);
  const viewOnly = useCallback(() => { setControlRole('viewer'); setChooser(false); }, []);

  // ── Determine which screen to render ─────────────────────
  function renderScreen() {
    // 非常停止中・エラー中でも、「ログを確認」ではログ画面を見られる（解除・復旧はしない）。上に戻る帯を出す
    const logsOverlay = nav === 'logs' && (robotState === 'EMERGENCY' || systemScreen === 'error');
    if (logsOverlay) {
      const isEmg = robotState === 'EMERGENCY' && systemScreen !== 'error';
      return React.createElement(
        React.Fragment, null,
        React.createElement('div', {
          style: {
            display: 'flex', alignItems: 'center', gap: 12, padding: '6px 12px', marginBottom: 6,
            border: '1.5px solid #943030', background: '#2a1212', borderRadius: 4, flexShrink: 0,
          },
        },
          React.createElement('span', { style: { color: '#e05050', fontWeight: 700, fontSize: 14 } },
            isEmg ? '非常停止中です（解除していません）' : 'エラー状態です（復旧していません）'),
          React.createElement('div', { style: { flex: 1 } }),
          React.createElement(window.Btn, { onClick: () => setNav('init') },
            isEmg ? '非常停止の画面へ戻る' : 'エラーの画面へ戻る')),
        React.createElement(window.LogsScreen));
    }

    // System-level overrides
    if (systemScreen === 'ac_loss') return React.createElement(window.EmergencyResetScreen); // AC 断も非常停止の画面にまとめた（きっかけは 1 行で出す）
    if (systemScreen === 'booting') return React.createElement(window.BootingScreen);
    if (systemScreen === 'error') return React.createElement(window.ErrorScreen);

    // Emergency can overlay on any screen
    if (robotState === 'EMERGENCY') return React.createElement(window.EmergencyResetScreen);

    switch (nav) {
      case 'init':        return React.createElement(window.InitScreen);
      case 'profiles':    return React.createElement(window.ProfilesScreen);
      case 'calibration': return React.createElement(window.CalibrationScreen);
      case 'modes':       return React.createElement(window.ModesScreen);
      case 'learning':    return React.createElement(window.LearningScreen);
      case 'auto':        return React.createElement(window.AutoDriveScreen);
      case 'manual':      return React.createElement(window.ManualScreen);
      case 'schedule':    return React.createElement(window.ScheduleScreen);
      case 'sequence':    return React.createElement(window.SequenceScreen);
      case 'logs':        return React.createElement(window.LogsScreen);
      case 'errors':      return React.createElement(window.ErrorsScreen);
      default:            return React.createElement(window.InitScreen);
    }
  }

  // 閲覧中でも、ログ・エラーの画面（本物の画面のとき）は制限しない。非常停止・エラーの全画面などは制限する
  const logsOverlayNow = nav === 'logs' && (robotState === 'EMERGENCY' || systemScreen === 'error');
  const realScreenNow = logsOverlayNow || (!systemScreen && robotState !== 'EMERGENCY');
  const viewerLock = controlRole === 'viewer' && !((nav === 'logs' || nav === 'errors') && realScreenNow);

  // 操作・閲覧が決まるまで: サーバーの判定が来るまでは何も出さず、来たら選ぶ画面を出す
  if (controlRole === null) {
    return React.createElement(
      AppContext.Provider,
      { value: ctx },
      React.createElement('div', { style: { width: '100%', height: '100%', background: window.PAPER, color: window.INK, display: 'flex', flexDirection: 'column' } },
        chooser ? React.createElement(window.ControlChooserScreen, { held: controlHeld, onOperate: claimControl, onView: viewOnly }) : null));
  }

  return React.createElement(
    AppContext.Provider,
    { value: ctx },
    React.createElement(
      window.Frame,
      {
        state: robotState,
        screen: NAV_LABEL[nav] ?? nav,
        activeNav: nav,
        viewer: controlRole === 'viewer',
        viewerLock,
        upsLoss,
        upsInstalled,
        locked: navLock,
        profileName: activeProfileName,
        // 走っているモード走行・スケジュール・シーケンスがあれば、ブラウザの選択よりそれを出す（段 B）
        modeName: (realtimeData?.sequence?.state === 'running' ? realtimeData.sequence.name : null)
          ?? realtimeData?.job?.mode_name ?? realtimeData?.job?.schedule_name ?? activeModeName,
        // 見出しの種類（mode / スケジュール / シーケンス）。走っているものがあればそれ、無ければ選択の種類
        modeLabel: realtimeData?.sequence?.state === 'running' ? 'シーケンス'
          : realtimeData?.job?.mode_name ? 'mode'
          : realtimeData?.job?.schedule_name ? 'スケジュール'
          : activeModeKind === 'sequence' ? 'シーケンス' : activeModeKind === 'schedule' ? 'スケジュール' : 'mode',
        onNav: key => {
          if (navLock && key !== nav) {
            showToast('実行中は他のページに移動できません', 'error');
            return;
          }
          setNav(key);
        },
        // 非常停止オーバーレイの「初期化画面へ」。スイッチ解除後、まず非常停止を
        // リセット（EMERGENCY → STANDBY）してから初期化画面へ遷移する。
        // リセットしないと robotState が EMERGENCY のままでオーバーレイが消えない。
        onGoInit: async () => {
          const r = await apiFetch('POST', '/api/v1/drive/reset-emergency');
          if (r) {
            showToast('非常停止をリセットしました', 'success');
            setNav('init');
          }
        },
      },
      renderScreen()
    )
  );
}

const NAV_LABEL = {
  init: '初期化',
  profiles: '車両プロファイル',
  modes: '走行モード',
  schedule: 'スケジュールモード',
  sequence: 'シーケンスモード',
  calibration: 'キャリブレーション',
  learning: '学習運転',
  auto: '自動運転',
  manual: '手動運転',
  logs: 'ログ',
  errors: 'エラー',
};

window.AppContext = AppContext;

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(React.createElement(App));
