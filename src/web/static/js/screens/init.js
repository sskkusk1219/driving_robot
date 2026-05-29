// ── Init screen ───────────────────────────────────────────
// Matches wireframe InitA: initialization sequence steps with status indicators

const { INK, INK_SOFT, INK_MUTE, PAPER_2, Box, Btn, H2 } = window;

function InitScreen() {
  const { useState, useContext, useEffect, useRef } = React;
  const { robotState, apiFetch, setNav } = useContext(window.AppContext);

  const [loading, setLoading] = useState(false);
  const [, forceUpdate] = useState(0);
  const initStartRef = useRef(null);

  const isInitializing = loading || robotState === 'INITIALIZING';

  useEffect(() => {
    if (!isInitializing) return;
    if (!initStartRef.current) initStartRef.current = Date.now();
    const id = setInterval(() => forceUpdate(n => n + 1), 400);
    return () => clearInterval(id);
  }, [isInitializing]);

  const STEPS = [
    { label: '通信確認 (ブレーキ)' },
    { label: '通信確認 (アクセル)' },
    { label: '通信確認 (CAN)' },
    { label: 'アラームリセット (両軸)' },
    { label: 'サーボON (両軸)' },
    { label: '原点復帰' },
  ];

  function stepStatus(i) {
    if (!isInitializing || !initStartRef.current) return null;
    const elapsed = (Date.now() - initStartRef.current) / 1000;
    const doneAt   = i * 1.5;
    const activeAt = doneAt + 1.5;
    if (elapsed >= activeAt) return true;   // ✓
    if (elapsed >= doneAt)   return 'now';  // ⟳
    return null;                             // —
  }

  async function handleInitialize() {
    setLoading(true);
    initStartRef.current = Date.now();
    const r = await apiFetch('POST', '/api/v1/drive/initialize');
    setLoading(false);
    if (r) window.showToast('初期化を開始しました', 'success');
  }

  return (
    <div style={{
      display: 'flex', flexDirection: 'column', gap: 18, height: '100%',
    }}>

      <Box style={{ padding: 18, flex: 1 }}>
        {STEPS.map(({ label, sub }, i) => {
          const st = stepStatus(i);
          return (
            <div key={label} style={{
              display: 'flex', alignItems: 'center', gap: 16,
              padding: '18px 8px',
              borderBottom: `1px dashed ${INK_MUTE}`,
            }}>
              <div style={{
                width: 36, height: 36, borderRadius: '50%',
                border: `1.8px solid ${st === true ? '#3f6b3f' : st === 'now' ? INK : INK_MUTE}`,
                background: st === true ? '#dfeadc' : st === 'now' ? PAPER_2 : 'transparent',
                display: 'flex', alignItems: 'center', justifyContent: 'center',
                fontWeight: 700, fontSize: 18,
              }}>
                {st === true ? '✓' : st === 'now' ? '⟳' : '—'}
              </div>
              <div style={{ flex: 1 }}>
                <div style={{ fontSize: 20, fontWeight: st === 'now' ? 700 : 400 }}>{label}</div>
                {sub && <div style={{ fontSize: 12, color: INK_SOFT, fontFamily: 'monospace' }}>{sub}</div>}
              </div>
              {st === 'now' && <div style={{ fontSize: 13, color: INK_SOFT }}>実行中...</div>}
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
