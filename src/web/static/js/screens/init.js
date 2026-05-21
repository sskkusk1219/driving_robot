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
    { label: '通信確認 (ttyUSB0)',      sub: 'pymodbus connect OK' },
    { label: '通信確認 (ttyUSB1)',      sub: 'pymodbus connect OK' },
    { label: '通信確認 (CAN/Kvaser)',   sub: '/dev/usbcanII0 ready' },
    { label: 'アラームリセット (両軸)',  sub: 'FC05 0x0407 → FF00' },
    { label: 'サーボON (両軸)',         sub: 'FC05 0x0403' },
    { label: '原点復帰',               sub: '前回正常終了 → スキップ予定' },
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
      maxWidth: 760, margin: '0 auto', justifyContent: 'center',
    }}>
      <H2 sub="アラームリセット → サーボON → 原点復帰 (必要時)">
        初期化を実行します
      </H2>

      <Box style={{ padding: 18 }}>
        {STEPS.map(({ label, sub }, i) => {
          const st = stepStatus(i);
          return (
            <div key={label} style={{
              display: 'flex', alignItems: 'center', gap: 12,
              padding: '10px 4px',
              borderBottom: `1px dashed ${INK_MUTE}`,
            }}>
              <div style={{
                width: 26, height: 26, borderRadius: '50%',
                border: `1.4px solid ${st === true ? '#3f6b3f' : st === 'now' ? INK : INK_MUTE}`,
                background: st === true ? '#dfeadc' : st === 'now' ? PAPER_2 : 'transparent',
                display: 'flex', alignItems: 'center', justifyContent: 'center',
                fontWeight: 700, fontSize: 14,
              }}>
                {st === true ? '✓' : st === 'now' ? '⟳' : '—'}
              </div>
              <div style={{ flex: 1 }}>
                <div style={{ fontSize: 16, fontWeight: st === 'now' ? 700 : 400 }}>{label}</div>
                <div style={{ fontSize: 12, color: INK_SOFT, fontFamily: 'monospace' }}>{sub}</div>
              </div>
              {st === 'now' && <div style={{ fontSize: 13, color: INK_SOFT }}>実行中...</div>}
            </div>
          );
        })}
      </Box>

      <div style={{ display: 'flex', justifyContent: 'flex-end', gap: 12 }}>
        <Btn onClick={() => setNav('profiles')} disabled={isInitializing}>
          キャンセル
        </Btn>
        <Btn primary big onClick={isInitializing ? undefined : handleInitialize} disabled={isInitializing}>
          {isInitializing ? 'READY 待ち...' : '初期化を実行'}
        </Btn>
      </div>
    </div>
  );
}

window.InitScreen = InitScreen;
