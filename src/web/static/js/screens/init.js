// ── Init screen ───────────────────────────────────────────
// 初期化シーケンスのステップ表示。バックエンドの実ハード操作の進捗
// (realtimeData.init_steps) と連動し、各ステップ完了でチェックされる。

const { INK, INK_SOFT, INK_MUTE, PAPER_2, Box, Btn, H2 } = window;

// WebSocket 配信前 (init_steps が空) のフォールバック表示。全て PENDING。
const FALLBACK_INIT_STEPS = [
  { key: 'comm_brake',  label: '通信確認 (ブレーキ)',     status: 'pending' },
  { key: 'comm_accel',  label: '通信確認 (アクセル)',     status: 'pending' },
  { key: 'comm_can',    label: '通信確認 (CAN)',          status: 'pending' },
  { key: 'alarm_reset', label: 'アラームリセット (両軸)', status: 'pending' },
  { key: 'servo_on',    label: 'サーボON (両軸)',         status: 'pending' },
  { key: 'home_return', label: '原点復帰',                status: 'pending' },
];

function InitScreen() {
  const { useState, useContext } = React;
  const { robotState, realtimeData, apiFetch } = useContext(window.AppContext);

  const [loading, setLoading] = useState(false);

  const isInitializing = loading || robotState === 'INITIALIZING';

  const steps = (realtimeData.init_steps && realtimeData.init_steps.length)
    ? realtimeData.init_steps
    : FALLBACK_INIT_STEPS;

  // status → 表示インジケータ (丸印・色・補足ラベル)
  function indicator(status) {
    switch (status) {
      case 'done':    return { mark: '✓', border: '#2a6c2a', bg: '#0e1e0e',    note: null };
      case 'running': return { mark: '⟳', border: INK,       bg: PAPER_2,      note: '実行中...' };
      case 'skipped': return { mark: '⤼', border: INK_MUTE,  bg: '#1e1e18',    note: 'スキップ' };
      case 'error':   return { mark: '✗', border: '#943030', bg: '#200e0e',    note: 'エラー' };
      default:        return { mark: '—', border: INK_MUTE,  bg: 'transparent', note: null };
    }
  }

  async function handleInitialize() {
    setLoading(true);
    const r = await apiFetch('POST', '/api/v1/drive/initialize');
    setLoading(false);
    if (r) window.showToast('初期化を開始しました', 'success');
  }

  return (
    <div style={{
      display: 'flex', flexDirection: 'column', gap: 18, height: '100%',
    }}>

      <Box style={{ padding: 18, flex: 1 }}>
        {steps.map(({ key, label, status }) => {
          const ind = indicator(status);
          const active = status === 'running';
          return (
            <div key={key} style={{
              display: 'flex', alignItems: 'center', gap: 16,
              padding: '18px 8px',
              borderBottom: `1px dashed ${INK_MUTE}`,
            }}>
              <div style={{
                width: 36, height: 36, borderRadius: '50%',
                border: `1.8px solid ${ind.border}`,
                background: ind.bg,
                display: 'flex', alignItems: 'center', justifyContent: 'center',
                fontWeight: 700, fontSize: 18,
              }}>
                {ind.mark}
              </div>
              <div style={{ flex: 1 }}>
                <div style={{ fontSize: 20, fontWeight: active ? 700 : 400 }}>{label}</div>
              </div>
              {ind.note && <div style={{ fontSize: 13, color: INK_SOFT }}>{ind.note}</div>}
            </div>
          );
        })}
      </Box>

      <div style={{ display: 'flex', justifyContent: 'flex-end', gap: 12 }}>
        <Btn primary big onClick={isInitializing ? undefined : handleInitialize} disabled={isInitializing}>
          {isInitializing ? 'READY 待ち...' : '初期化を実行'}
        </Btn>
      </div>
    </div>
  );
}

window.InitScreen = InitScreen;
