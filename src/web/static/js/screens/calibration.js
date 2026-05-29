// ── Calibration screen ────────────────────────────────────

function AxisCal({ label, axisId, currentPos, zero, full, onJog, onSetZero, onSetFull, onHome }) {
  const { INK, INK_SOFT, INK_MUTE, Box, Btn } = window;
  const { JogKey, DragSlider } = window;
  const stroke = (zero !== null && full !== null) ? Math.abs(full - zero) : null;

  return (
    <Box label={label} style={{ padding: '10px 12px', display: 'flex', flexDirection: 'column', gap: 8, flex: 1, minHeight: 0 }}>

      {/* Jog buttons + drag slider */}
      <div style={{ flex: 1, minHeight: 0, display: 'flex', gap: 8 }}>
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: 8 }}>
          <JogKey label="−10" hint="Shift+←" onClick={() => onJog(axisId, -10)} />
          <JogKey label="−1"  hint="←"       onClick={() => onJog(axisId, -1)} />
        </div>

        <DragSlider currentPos={currentPos} axisId={axisId} onJog={onJog} zeroPos={zero} fullPos={full} />

        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: 8 }}>
          <JogKey label="+1"  hint="→"       onClick={() => onJog(axisId, 1)} />
          <JogKey label="+10" hint="Shift+→" onClick={() => onJog(axisId, 10)} />
        </div>
      </div>

      {/* ZERO / FULL buttons */}
      <div style={{ display: 'flex', gap: 8 }}>
        <Btn style={{ flex: 1, borderColor: '#3f6b3f', color: '#22421f', justifyContent: 'center' }}
             onClick={() => onSetZero(axisId)}>
          ZERO 確定 [Z]
        </Btn>
        <Btn style={{ flex: 1, borderColor: '#a23232', color: '#5e1414', justifyContent: 'center' }}
             onClick={() => onSetFull(axisId)}>
          FULL 確定 [F]
        </Btn>
      </div>

      {/* ZERO / FULL / STROKE info + home button */}
      <div style={{ display: 'flex', gap: 18, fontSize: 12, color: INK_SOFT, alignItems: 'center' }}>
        <span>ZERO <b style={{ fontFamily: 'monospace', color: zero !== null ? INK : INK_MUTE }}>{zero ?? '—'}</b></span>
        <span>FULL <b style={{ fontFamily: 'monospace', color: full !== null ? INK : INK_MUTE }}>{full ?? '—'}</b></span>
        <span>STROKE <b style={{ fontFamily: 'monospace', color: stroke !== null ? INK : INK_MUTE }}>{stroke ?? '—'}</b></span>
        <div style={{ flex: 1 }} />
        <Btn onClick={() => onHome(axisId)}>原点へ戻す</Btn>
      </div>
    </Box>
  );
}

function CalibrationScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeProfileId, setNav } = useContext(window.AppContext);
  const { Box, Btn, Note, Row, ValidationPopup } = window;

  const [brk, setBrk] = useState({ currentPos: 0, zero: null, full: null });
  const [acc, setAcc] = useState({ currentPos: 0, zero: null, full: null });
  const [popup, setPopup] = useState(null);

  useEffect(() => {
    if (!activeProfileId) setPopup('no_profile');
  }, []);

  function setAxisState(axisId, updater) {
    if (axisId === 'brake') setBrk(updater);
    else setAcc(updater);
  }

  async function handleJog(axisId, step) {
    const r = await apiFetch('POST', '/api/v1/drive/calib/jog', { axis: axisId, step });
    if (!r) return;
    setAxisState(axisId, s => ({ ...s, currentPos: r.position ?? s.currentPos }));
  }

  async function handleSetZero(axisId) {
    const r = await apiFetch('POST', '/api/v1/drive/calib/set-zero', { axis: axisId });
    if (!r) return;
    setAxisState(axisId, s => ({ ...s, zero: r.position ?? s.currentPos }));
    window.showToast(`${axisId === 'brake' ? 'ブレーキ' : 'アクセル'} ZERO を記録しました`, 'success');
  }

  async function handleSetFull(axisId) {
    const r = await apiFetch('POST', '/api/v1/drive/calib/set-full', { axis: axisId });
    if (!r) return;
    setAxisState(axisId, s => ({ ...s, full: r.position ?? s.currentPos }));
    window.showToast(`${axisId === 'brake' ? 'ブレーキ' : 'アクセル'} FULL を記録しました`, 'success');
  }

  async function handleHome(axisId) {
    const r = await apiFetch('POST', '/api/v1/drive/calib/home', { axis: axisId });
    if (!r) return;
    setAxisState(axisId, s => ({ ...s, currentPos: r.position ?? 0 }));
    window.showToast('原点へ戻しました', 'success');
  }

  async function handleSave() {
    if (!(brk.zero !== null && brk.full !== null && acc.zero !== null && acc.full !== null)) {
      window.showToast('全軸の ZERO / FULL を記録してから保存してください', 'error');
      return;
    }
    const r = await apiFetch('POST', '/api/v1/drive/calib/save');
    if (r) window.showToast('キャリブレーションを保存しました', 'success');
  }

  const canSave = brk.zero !== null && brk.full !== null && acc.zero !== null && acc.full !== null;

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 10, flex: 1, minHeight: 0 }}>

      {popup === 'no_profile' && (
        <ValidationPopup
          message="車両プロファイルを選択してください"
          actionLabel="プロファイルへ"
          onAction={() => setNav('profiles')}
        />
      )}

      {/* Top: axis panels (fill remaining height) */}
      <div style={{ display: 'flex', gap: 10, flex: 1, minHeight: 0 }}>
        <AxisCal
          label="ブレーキ" axisId="brake"
          currentPos={brk.currentPos} zero={brk.zero} full={brk.full}
          onJog={handleJog} onSetZero={handleSetZero} onSetFull={handleSetFull} onHome={handleHome}
        />
        <AxisCal
          label="アクセル" axisId="accel"
          currentPos={acc.currentPos} zero={acc.zero} full={acc.full}
          onJog={handleJog} onSetZero={handleSetZero} onSetFull={handleSetFull} onHome={handleHome}
        />
      </div>

      {/* Bottom: shortcuts | recorded + save button */}
      <div style={{ display: 'flex', gap: 10 }}>
        <Box label="キーボードショートカット" style={{ padding: '10px 12px', flex: 1 }}>
          <div style={{ fontSize: 12, lineHeight: 1.7, fontFamily: 'monospace' }}>
            <div>← / →        : ±1 pulse</div>
            <div>Shift+← / →  : ±10 pulse</div>
            <div>Z             : ZERO 確定</div>
            <div>F             : FULL 確定</div>
            <div>Tab           : 軸切替 (アクセル ↔ ブレーキ)</div>
            <div>Esc           : キャンセル</div>
          </div>
          <Note style={{ marginTop: 6 }}>キャリブレーション中は最大開度設定が無視されます。電流値を常時監視中。</Note>
        </Box>

        <div style={{ display: 'flex', gap: 10, flex: 1 }}>
          <Box label="記録済み" style={{ padding: '10px 14px', fontSize: 13, flex: 1 }}>
            <Row cells={[['', '22px'], ['軸', '1.4fr'], ['ZERO', '1fr'], ['FULL', '1fr']]} header />
            <Row cells={[
              [brk.zero !== null && brk.full !== null ? '✓' : '○', '22px'],
              ['ブレーキ', '1.4fr'],
              [brk.zero !== null ? String(brk.zero) : '—', '1fr'],
              [brk.full !== null ? String(brk.full) : '—', '1fr'],
            ]} />
            <Row cells={[
              [acc.zero !== null && acc.full !== null ? '✓' : '○', '22px'],
              ['アクセル', '1.4fr'],
              [acc.zero !== null ? String(acc.zero) : '—', '1fr'],
              [acc.full !== null ? String(acc.full) : '—', '1fr'],
            ]} />
          </Box>
          <div style={{ display: 'flex', flexDirection: 'column', flex: 1 }}>
            <Btn primary big style={{ flex: 1 }} disabled={!canSave} onClick={handleSave}>
              キャリブレーション保存
            </Btn>
          </div>
        </div>
      </div>

    </div>
  );
}

window.CalibrationScreen = CalibrationScreen;
