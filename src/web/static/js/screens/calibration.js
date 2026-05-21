// ── Calibration screen ────────────────────────────────────
// CalibrationA: step-by-step jog calibration matching wireframe

function AxisCal({ label, axisId, currentPos, zero, full, onJog, onSetZero, onSetFull, onHome }) {
  const { INK, INK_SOFT, INK_MUTE, Box, Btn } = window;
  const { JogKey, PosRuler } = window;
  const stroke = (zero !== null && full !== null) ? Math.abs(full - zero) : null;

  return (
    <Box label={label} style={{ padding: '12px 14px', display: 'flex', flexDirection: 'column', gap: 10 }}>
      <PosRuler
        zeroPos={zero ?? undefined}
        fullPos={full ?? undefined}
        currentPos={currentPos}
        maxPulse={5000}
      />
      <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
        <JogKey label="−10" hint="Shift+←" onClick={() => onJog(axisId, -10)} />
        <JogKey label="−1"  hint="←"       onClick={() => onJog(axisId, -1)} />
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 1 }}>
          <div style={{ fontSize: 12, color: INK_SOFT }}>現在位置</div>
          <div style={{ fontSize: 32, fontFamily: 'monospace', fontWeight: 800, lineHeight: 1 }}>{currentPos}</div>
          <div style={{ fontSize: 11, color: INK_SOFT }}>pulse</div>
        </div>
        <JogKey label="+1"  hint="→"       onClick={() => onJog(axisId, 1)} />
        <JogKey label="+10" hint="Shift+→" onClick={() => onJog(axisId, 10)} />
      </div>
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
      <div style={{ display: 'flex', gap: 18, fontSize: 13, color: INK_SOFT, alignItems: 'center' }}>
        <span>ZERO <b style={{ fontFamily: 'monospace', color: zero !== null ? INK : INK_MUTE }}>{zero ?? '—'}</b></span>
        <span>FULL <b style={{ fontFamily: 'monospace', color: full !== null ? INK : INK_MUTE }}>{full ?? '—'}</b></span>
        <span>STROKE <b style={{ fontFamily: 'monospace', color: stroke !== null ? INK : INK_MUTE }}>{stroke ?? '—'}</b></span>
        <div style={{ flex: 1 }} />
        <Btn onClick={() => onHome(axisId)}>原点へ戻す</Btn>
      </div>
    </Box>
  );
}

const CALIB_STEPS = ['ブレーキ ZERO', 'ブレーキ FULL', 'アクセル ZERO', 'アクセル FULL', '確定・保存'];

function CalibrationScreen() {
  const { useState, useContext } = React;
  const { apiFetch } = useContext(window.AppContext);
  const { INK, INK_SOFT, INK_MUTE, PAPER_2, Box, Btn, Note, Row } = window;

  const [brk, setBrk] = useState({ currentPos: 0, zero: null, full: null });
  const [acc, setAcc] = useState({ currentPos: 0, zero: null, full: null });

  function activeStep() {
    if (brk.zero === null) return 0;
    if (brk.full === null) return 1;
    if (acc.zero === null) return 2;
    if (acc.full === null) return 3;
    return 4;
  }

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
    const pos = r.position;
    setAxisState(axisId, s => ({ ...s, zero: pos ?? s.currentPos }));
    window.showToast(`${axisId === 'brake' ? 'ブレーキ' : 'アクセル'} ZERO を記録しました`, 'success');
  }

  async function handleSetFull(axisId) {
    const r = await apiFetch('POST', '/api/v1/drive/calib/set-full', { axis: axisId });
    if (!r) return;
    const pos = r.position;
    setAxisState(axisId, s => ({ ...s, full: pos ?? s.currentPos }));
    window.showToast(`${axisId === 'brake' ? 'ブレーキ' : 'アクセル'} FULL を記録しました`, 'success');
  }

  async function handleHome(axisId) {
    const r = await apiFetch('POST', '/api/v1/drive/calib/home', { axis: axisId });
    if (!r) return;
    setAxisState(axisId, s => ({ ...s, currentPos: r.position ?? 0 }));
    window.showToast('原点へ戻しました', 'success');
  }

  async function handleSave() {
    if (activeStep() < 4) {
      window.showToast('全軸の ZERO / FULL を記録してから保存してください', 'error');
      return;
    }
    const r = await apiFetch('POST', '/api/v1/drive/calib/save');
    if (r) window.showToast('キャリブレーションを保存しました', 'success');
  }

  const step = activeStep();
  const canSave = step >= 4;

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 10, flex: 1, minHeight: 0 }}>

      {/* Step progress bar */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
        {CALIB_STEPS.map((s, i) => {
          const isDone = i < step;
          const isActive = i === step;
          return (
            <React.Fragment key={s}>
              <div style={{
                display: 'flex', alignItems: 'center', gap: 6, fontSize: 13,
                padding: '4px 12px', borderRadius: 14,
                border: `1.4px solid ${isActive ? INK : INK_MUTE}`,
                background: isActive ? PAPER_2 : 'transparent',
                fontWeight: isActive ? 700 : 400,
                color: isDone ? INK_SOFT : INK,
                whiteSpace: 'nowrap',
              }}>
                {isDone
                  ? <span style={{ color: '#3f6b3f' }}>✓</span>
                  : <span style={{ fontFamily: 'monospace', fontSize: 12 }}>{i + 1}</span>}
                <span>{s}</span>
              </div>
              {i < 4 && <div style={{ flex: 1, height: 1, background: INK_MUTE, minWidth: 8 }} />}
            </React.Fragment>
          );
        })}
      </div>

      {/* 2×2 grid */}
      <div style={{
        display: 'grid',
        gridTemplateColumns: '1fr 1fr',
        gridTemplateRows: '1fr auto',
        gap: 10,
        flex: 1,
        minHeight: 0,
      }}>
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

        {/* Lower left: keyboard shortcuts */}
        <Box label="キーボードショートカット" style={{ padding: 12 }}>
          <div style={{ fontSize: 13, lineHeight: 1.9, fontFamily: 'monospace' }}>
            <div>← / →        : ±1 pulse</div>
            <div>Shift+← / →  : ±10 pulse</div>
            <div>Z             : ZERO 確定</div>
            <div>F             : FULL 確定</div>
            <div>Tab           : 軸切替 (アクセル ↔ ブレーキ)</div>
            <div>Space         : この位置で確定</div>
            <div>Esc           : キャンセル</div>
          </div>
          <Note style={{ marginTop: 8 }}>キャリブレーション中は最大開度設定が無視されます。電流値を常時監視中。</Note>
        </Box>

        {/* Lower right: recorded progress + save button */}
        <div style={{ display: 'flex', gap: 10, alignItems: 'stretch' }}>
          <Box label="記録済み" style={{ padding: '10px 14px', fontSize: 13, flex: 1 }}>
            <Row cells={[['', '22px'], ['軸', '1.4fr'], ['ZERO', '1fr'], ['FULL', '1fr']]} header />
            <Row cells={[
              [brk.zero !== null ? '✓' : (step === 0 ? '●' : ''), '22px'],
              ['ブレーキ', '1.4fr'],
              [brk.zero !== null ? String(brk.zero) : '—', '1fr'],
              [brk.full !== null ? String(brk.full) : '—', '1fr'],
            ]} style={{ background: step <= 1 ? PAPER_2 : 'transparent' }} />
            <Row cells={[
              [acc.zero !== null ? '✓' : (step >= 2 ? '●' : ''), '22px'],
              ['アクセル', '1.4fr'],
              [acc.zero !== null ? String(acc.zero) : '—', '1fr'],
              [acc.full !== null ? String(acc.full) : '—', '1fr'],
            ]} style={{ background: step >= 2 ? PAPER_2 : 'transparent' }} />
          </Box>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 8, flex: 1 }}>
            <Btn primary big style={{ flex: 1, flexDirection: 'column', lineHeight: 1.4 }}
                 disabled={!canSave} onClick={handleSave}>
              キャリブレーション<br />保存
            </Btn>
          </div>
        </div>
      </div>

    </div>
  );
}

window.CalibrationScreen = CalibrationScreen;
