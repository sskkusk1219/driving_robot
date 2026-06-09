// ── Manual drive screen ───────────────────────────────────

function AxisJog({ label, axisId, currentPos, openingPct, maxPct, currentMa, onJog, onHome }) {
  const { INK, INK_SOFT, PAPER, Box, Btn } = window;
  const { JogKey, DragSlider } = window;

  return (
    <Box label={label} style={{ padding: '10px 12px', display: 'flex', flexDirection: 'column', gap: 8, flex: 1, minHeight: 0 }}>

      {/* Jog buttons + drag slider */}
      <div style={{ flex: 1, minHeight: 0, display: 'flex', gap: 8 }}>
        {/* Left buttons */}
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: 8 }}>
          <JogKey label="−10" hint="Shift+←" onClick={() => onJog(axisId, -10)} />
          <JogKey label="−1"  hint="←"       onClick={() => onJog(axisId, -1)} />
        </div>

        <DragSlider currentPos={currentPos} axisId={axisId} onJog={onJog} />

        {/* Right buttons */}
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: 8 }}>
          <JogKey label="+1"  hint="→"       onClick={() => onJog(axisId, 1)} />
          <JogKey label="+10" hint="Shift+→" onClick={() => onJog(axisId, 10)} />
        </div>
      </div>

      {/* Opening bar */}
      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 10, color: INK_SOFT, marginBottom: 3, fontFamily: 'monospace' }}>
          <span>0 %</span><span>50</span><span>100 % (max {maxPct})</span>
        </div>
        <div style={{ height: 13, border: `1.4px solid ${INK}`, position: 'relative', background: PAPER }}>
          <div style={{ position: 'absolute', left: 0, top: 0, bottom: 0, width: `${openingPct}%`, background: INK, opacity: 0.82 }} />
        </div>
      </div>

      {/* Info + home */}
      <div style={{ display: 'flex', gap: 12, fontSize: 12, color: INK_SOFT, alignItems: 'center' }}>
        <span>開度 <b style={{ fontFamily: 'monospace', color: INK }}>{openingPct.toFixed(1)} %</b></span>
        <span>電流 <b style={{ fontFamily: 'monospace', color: INK }}>{currentMa} mA</b></span>
        <div style={{ flex: 1 }} />
        <Btn onClick={() => onHome(axisId)}>原点へ戻す</Btn>
      </div>
    </Box>
  );
}

function ManualScreen() {
  const { useState, useContext } = React;
  const { apiFetch, realtimeData, robotState } = useContext(window.AppContext);
  const { Box, Btn, Note, Row } = window;
  const { ConfirmStopPopup } = window;

  const [brk, setBrk] = useState({ currentPos: 0 });
  const [acc, setAcc] = useState({ currentPos: 0 });
  const [confirmStop, setConfirmStop] = useState(false);

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
        />
        <AxisJog
          label="アクセル" axisId="accel"
          currentPos={acc.currentPos}
          openingPct={accOpeningPct} maxPct={80} currentMa={accMa}
          onJog={handleJog} onHome={handleHome}
        />
      </div>

      {/* Bottom: shortcuts | status + button */}
      <div style={{ display: 'flex', gap: 10 }}>
        <Box label="キーボードショートカット" style={{ padding: '10px 12px', flex: 1 }}>
          <div style={{ fontSize: 12, lineHeight: 1.7, fontFamily: 'monospace' }}>
            <div>← / →        : ±1 pulse</div>
            <div>Shift+← / →  : ±10 pulse</div>
            <div>Tab           : 軸切替 (アクセル ↔ ブレーキ)</div>
            <div>Esc           : 手動運転終了</div>
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
              <Btn big style={{ flex: 1, borderColor: '#3f6b3f', background: '#dfeadc', color: '#22421f' }} onClick={handleStart}>▶ 走行開始</Btn>
            )}
          </div>
        </div>
      </div>

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
