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
};

// ── Toast system ──────────────────────────────────────────
function showToast(msg, type = 'info') {
  const container = document.getElementById('toast-container');
  const el = document.createElement('div');
  el.className = `toast${type === 'error' ? ' error' : type === 'success' ? ' success' : ''}`;
  el.textContent = msg;
  container.appendChild(el);
  setTimeout(() => el.remove(), 3500);
}
window.showToast = showToast;

// ── API client ────────────────────────────────────────────
async function apiFetch(method, path, body = null, isFormData = false) {
  try {
    const opts = { method };
    if (body) {
      if (isFormData) {
        opts.body = body;
      } else {
        opts.headers = { 'Content-Type': 'application/json' };
        opts.body = JSON.stringify(body);
      }
    }
    const res = await fetch(path, opts);
    if (res.status === 204) return {};
    const json = await res.json().catch(() => ({}));
    if (!res.ok) {
      showToast(`エラー ${res.status}: ${json.detail ?? res.statusText}`, 'error');
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

  const [nav, setNav] = useState('init');
  const [robotState, setRobotState] = useState('BOOTING');
  const [activeProfileId, setActiveProfileId] = useState(null);
  const [activeProfileName, setActiveProfileName] = useState(null);
  const [activeModeId, setActiveModeId] = useState(null);
  const [activeModeName, setActiveModeName] = useState(null);
  const [upsLoss, setUpsLoss] = useState(false);
  const [realtimeData, setRealtimeData] = useState(INIT_REALTIME);
  const realtimeBuf = useRef([]);

  // ── System state screen override ─────────────────────────
  const [systemScreen, setSystemScreen] = useState(null);

  const wsRef = useRef(null);
  const reconnectTimer = useRef(null);

  const connectWS = useCallback(() => {
    if (wsRef.current && wsRef.current.readyState < 2) return;
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws/realtime`);
    wsRef.current = ws;

    ws.onmessage = ev => {
      let d;
      try { d = JSON.parse(ev.data); } catch { return; }
      const state = d.robot_state;
      setRobotState(state);
      setUpsLoss(d.ups_loss ?? false);
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
      clearTimeout(reconnectTimer.current);
      reconnectTimer.current = setTimeout(connectWS, 2000);
    };
    ws.onerror = () => ws.close();
  }, []);

  useEffect(() => {
    connectWS();
    // Poll system state on mount
    apiFetch('GET', '/api/v1/drive/status').then(d => {
      if (!d) return;
      setRobotState(d.robot_state);
      if (d.active_profile_id) setActiveProfileId(d.active_profile_id);
    });
    return () => {
      if (wsRef.current) wsRef.current.close();
      clearTimeout(reconnectTimer.current);
    };
  }, [connectWS]);

  const ctx = {
    nav, setNav,
    robotState,
    activeProfileId, setActiveProfileId,
    activeProfileName, setActiveProfileName,
    activeModeId, setActiveModeId,
    activeModeName, setActiveModeName,
    upsLoss,
    realtimeData,
    realtimeBuf,
    showToast,
    apiFetch,
  };

  // ── Determine which screen to render ─────────────────────
  function renderScreen() {
    // System-level overrides
    if (systemScreen === 'ac_loss') return React.createElement(window.AcPowerLossScreen);
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
      default:            return React.createElement(window.InitScreen);
    }
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
        upsLoss,
        profileName: activeProfileName,
        modeName: activeModeName,
        onNav: key => setNav(key),
        onGoInit: () => setNav('init'),
      },
      renderScreen()
    )
  );
}

const NAV_LABEL = {
  init: '初期化',
  profiles: '車両プロファイル',
  calibration: 'キャリブレーション',
  modes: '走行モード',
  learning: '学習運転',
  auto: '自動走行',
  manual: '手動運転',
  schedule: 'タイムスケジュール',
  sequence: 'シーケンス',
  logs: 'ログ',
};

window.AppContext = AppContext;

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(React.createElement(App));
