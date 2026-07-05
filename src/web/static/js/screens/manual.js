// ── Manual drive screen ───────────────────────────────────

function AxisJog({ label, axisId, currentPos, openingPct, maxPct, currentMa, onJog, onHome, enabled = true, active = false }) {
  const { INK, INK_SOFT, PAPER, Box, Btn } = window;
  const { JogKey, DragSlider } = window;
  const dis = !enabled;

  return (
    <Box
      label={active ? `${label}（操作中）` : label}
      style={{
        padding: '10px 12px', display: 'flex', flexDirection: 'column', gap: 8, flex: 1, minHeight: 0,
        ...(active ? { border: '2px solid #c8922a' } : {}),
      }}
    >

      {/* Jog buttons + drag slider */}
      <div style={{ flex: 1, minHeight: 0, display: 'flex', gap: 8 }}>
        {/* Left buttons */}
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: 8 }}>
          <JogKey label="−10"  onClick={() => onJog(axisId, -10)} disabled={dis} />
          <JogKey label="−100" onClick={() => onJog(axisId, -100)} disabled={dis} />
        </div>

        <DragSlider currentPos={currentPos} axisId={axisId} onJog={onJog} disabled={dis} />

        {/* Right buttons */}
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: 8 }}>
          <JogKey label="+10"  onClick={() => onJog(axisId, 10)} disabled={dis} />
          <JogKey label="+100" onClick={() => onJog(axisId, 100)} disabled={dis} />
        </div>
      </div>

      {/* Opening bar */}
      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 10, color: INK_SOFT, marginBottom: 3, fontFamily: 'inherit' }}>
          <span>0 %</span><span>50</span><span>100 % (max {maxPct})</span>
        </div>
        <div style={{ height: 10, border: `1px solid ${HATCH}`, position: 'relative', background: PAPER, borderRadius: 2 }}>
          <div style={{ position: 'absolute', left: 0, top: 0, bottom: 0, width: `${openingPct}%`, background: '#c8922a', opacity: 0.9 }} />
        </div>
      </div>

      {/* Info + home */}
      <div style={{ display: 'flex', gap: 12, fontSize: 12, color: INK_SOFT, alignItems: 'center' }}>
        <span>開度 <b style={{ fontFamily: 'inherit', color: INK }}>{openingPct.toFixed(1)} %</b></span>
        <span>電流 <b style={{ fontFamily: 'inherit', color: INK }}>{currentMa} mA</b></span>
        <div style={{ flex: 1 }} />
        <Btn onClick={() => onHome(axisId)} disabled={dis}>原点へ戻す</Btn>
      </div>
    </Box>
  );
}

function ManualScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, realtimeData, robotState, setNavLock } = useContext(window.AppContext);
  const { Box, Btn, Note, Row } = window;
  const { ConfirmStopPopup } = window;

  const [brk, setBrk] = useState({ currentPos: 0 });
  const [acc, setAcc] = useState({ currentPos: 0 });
  const [activeAxis, setActiveAxis] = useState('accel'); // キーボード操作対象 (Tabで切替)
  const [confirmStop, setConfirmStop] = useState(false);
  const [confirmStart, setConfirmStart] = useState(false);

  const enabled = robotState === 'MANUAL';

  // 手動運転中は他ページへの離脱をロック
  useEffect(() => {
    setNavLock(enabled);
    return () => setNavLock(false);
  }, [enabled]);

  // キーボードショートカット: e/w ±10, d/s ±100, Tab 軸切替
  useEffect(() => {
    if (!enabled) return;
    function onKey(e) {
      const tag = (e.target.tagName || '').toLowerCase();
      if (tag === 'input' || tag === 'textarea') return;
      switch (e.key) {
        case 'e': case 'E': handleJog(activeAxis, 10); break;
        case 'w': case 'W': handleJog(activeAxis, -10); break;
        case 'd': case 'D': handleJog(activeAxis, 100); break;
        case 's': case 'S': handleJog(activeAxis, -100); break;
        case 'Tab':
          e.preventDefault();
          setActiveAxis(a => (a === 'accel' ? 'brake' : 'accel'));
          break;
        default: return;
      }
    }
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [enabled, activeAxis]);

  function setAxisState(axisId, updater) {
    if (axisId === 'brake') setBrk(updater);
    else setAcc(updater);
  }

  async function handleJog(axisId, step) {
    const r = await apiFetch('POST', '/api/v1/drive/manual/jog', { axis: axisId, step });
    if (!r) return;
    setAxisState(axisId, s => ({ ...s, currentPos: r.position ?? s.currentPos }));
  }

  async function handleHome(axisId) {
    const r = await apiFetch('POST', '/api/v1/drive/manual/home', { axis: axisId });
    if (!r) return;
    setAxisState(axisId, s => ({ ...s, currentPos: r.position ?? 0 }));
    window.showToast('原点へ戻しました', 'success');
  }

  async function handleStart() {
    const r = await apiFetch('POST', '/api/v1/drive/manual/start');
    if (r) window.showToast('手動運転を開始しました', 'success');
  }

  async function handleStop() {
    const r = await apiFetch('POST', '/api/v1/drive/manual/stop');
    if (r) {
      window.showToast('手動運転を終了しました', 'success');
      setConfirmStop(false);
    }
  }

  const brkOpeningPct = realtimeData.brake_opening ?? 0;
  const accOpeningPct = realtimeData.accel_opening ?? 0;
  const brkMa = realtimeData.brake_current_ma ?? 0;
  const accMa = realtimeData.accel_current_ma ?? 0;

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 10, flex: 1, minHeight: 0 }}>

      {/* Top: axis panels (fill remaining height) */}
      <div style={{ display: 'flex', gap: 10, flex: 1, minHeight: 0 }}>
        <AxisJog
          label="ブレーキ" axisId="brake"
          currentPos={brk.currentPos}
          openingPct={brkOpeningPct} maxPct={70} currentMa={brkMa}
          onJog={handleJog} onHome={handleHome}
          enabled={enabled} active={enabled && activeAxis === 'brake'}
        />
        <AxisJog
          label="アクセル" axisId="accel"
          currentPos={acc.currentPos}
          openingPct={accOpeningPct} maxPct={80} currentMa={accMa}
          onJog={handleJog} onHome={handleHome}
          enabled={enabled} active={enabled && activeAxis === 'accel'}
        />
      </div>

      {/* Bottom: shortcuts | status + button */}
      <div style={{ display: 'flex', gap: 10 }}>
        <Box label="キーボードショートカット" style={{ padding: '10px 12px', flex: 1 }}>
          <div style={{ fontSize: 12, lineHeight: 1.7, fontFamily: 'inherit' }}>
            <div>e / w  : ±10 pulse</div>
            <div>d / s  : ±100 pulse</div>
            <div>Tab    : 軸切替 (アクセル ↔ ブレーキ)</div>
          </div>
          <Note style={{ marginTop: 6 }}>ジョグ中は最大開度リミットが有効です。電流値を常時監視中。</Note>
        </Box>

        <div style={{ display: 'flex', gap: 10, flex: 1 }}>
          <Box style={{ padding: '10px 14px', fontSize: 13, flex: 1 }}>
            <Row cells={[['実車速',       '1.4fr'], [`${(realtimeData.actual_speed_kmh ?? 0).toFixed(1)} km/h`, '1fr', 'mono']]} />
            <Row cells={[['ブレーキ pos', '1.4fr'], [`${brk.currentPos} pulse`, '1fr', 'mono']]} />
            <Row cells={[['アクセル pos', '1.4fr'], [`${acc.currentPos} pulse`, '1fr', 'mono']]} />
          </Box>
          <div style={{ display: 'flex', flexDirection: 'column', flex: 1 }}>
            {robotState === 'MANUAL' ? (
              <Btn danger big style={{ flex: 1 }} onClick={() => setConfirmStop(true)}>■ 運転終了</Btn>
            ) : (
              <Btn big style={{ flex: 1, borderColor: '#3c8c3c', background: '#0e220e', color: '#68d468' }} onClick={() => setConfirmStart(true)}>開始</Btn>
            )}
          </div>
        </div>
      </div>

      {confirmStart && (
        <ConfirmStopPopup
          message="開始しますか？"
          onYes={() => { handleStart(); setConfirmStart(false); }}
          onNo={() => setConfirmStart(false)}
        />
      )}

      {confirmStop && (
        <ConfirmStopPopup
          message="運転を終了しますか？"
          onYes={handleStop}
          onNo={() => setConfirmStop(false)}
        />
      )}
    </div>
  );
}

window.ManualScreen = ManualScreen;
