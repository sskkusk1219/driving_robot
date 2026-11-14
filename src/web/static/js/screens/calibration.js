// ── Calibration screen ────────────────────────────────────

function AxisCal({ label, axisId, currentPos, zero, full, onJog, onSetZero, onSetFull, onHome, enabled = true, selected = false, onSelect }) {
  const { INK, INK_SOFT, INK_MUTE, Btn } = window;
  const { AxisPanel, JogRow } = window;
  const stroke = (zero !== null && full !== null) ? Math.abs(full - zero) : null;
  const dis = !enabled;

  return (
    <AxisPanel label={label} axisId={axisId} selected={selected} enabled={enabled} onSelect={onSelect}>

      {/* Jog buttons + drag slider */}
      <JogRow axisId={axisId} currentPos={currentPos} onJog={onJog} zeroPos={zero} fullPos={full} disabled={dis} />

      {/* ZERO / FULL buttons */}
      <div style={{ display: 'flex', gap: 8 }}>
        <Btn style={{ flex: 1, borderColor: '#2a6c2a', color: '#54bc54', justifyContent: 'center' }}
             disabled={dis} onClick={() => onSetZero(axisId)}>
          ZERO 確定
        </Btn>
        <Btn style={{ flex: 1, borderColor: '#943030', color: '#e05050', justifyContent: 'center' }}
             disabled={dis} onClick={() => onSetFull(axisId)}>
          FULL 確定
        </Btn>
      </div>

      {/* ZERO / FULL / STROKE info + home button */}
      <div style={{ display: 'flex', gap: 18, fontSize: 12, color: INK_SOFT, alignItems: 'center' }}>
        <span>ZERO <b style={{ fontFamily: 'inherit', color: zero !== null ? INK : INK_MUTE }}>{zero ?? '—'}</b></span>
        <span>FULL <b style={{ fontFamily: 'inherit', color: full !== null ? INK : INK_MUTE }}>{full ?? '—'}</b></span>
        <span>STROKE <b style={{ fontFamily: 'inherit', color: stroke !== null ? INK : INK_MUTE }}>{stroke ?? '—'}</b></span>
        <div style={{ flex: 1 }} />
        <Btn onClick={() => onHome(axisId)} disabled={dis}>原点へ戻す</Btn>
      </div>
    </AxisPanel>
  );
}

function CalibrationScreen() {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, activeProfileId, setNav, setNavLock, robotState } = useContext(window.AppContext);
  const { Box, Btn, Note, Row, ValidationPopup, ConfirmStopPopup } = window;

  const [brk, setBrk] = useState({ currentPos: 0, zero: null, full: null });
  const [acc, setAcc] = useState({ currentPos: 0, zero: null, full: null });
  const [popup, setPopup] = useState(null);
  const [started, setStarted] = useState(false);
  const [confirmStart, setConfirmStart] = useState(false);
  const [selectedAxis, setSelectedAxis] = useState('brake');  // キー操作の対象（Tab・パネルのクリックで切替）

  // サーバーがすでにキャリブレーション中（再読み込みなど）なら、開始済みの表示から始める
  useEffect(() => {
    if (robotState === 'CALIBRATING') setStarted(true);
  }, [robotState]);

  // 開始できるのは初期化済み（READY）のときだけ。それ以外で開始すると操作がすべて 409 になる
  const canStart = robotState === 'READY' || robotState === 'CALIBRATING';

  useEffect(() => {
    if (!activeProfileId) setPopup('no_profile');
  }, []);

  // 実行中（開始後）は他ページへの離脱をロック
  useEffect(() => {
    setNavLock(started);
    return () => setNavLock(false);
  }, [started]);

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
    if (r) {
      window.showToast('キャリブレーションを保存しました', 'success');
      setStarted(false);  // セッション終了 → ナビロック解除・ボタンを「開始」に戻す
    }
  }

  const canSave = brk.zero !== null && brk.full !== null && acc.zero !== null && acc.full !== null;

  // キー操作（tests/hardware/test_calibration.py と同じ割り当て）
  //   d / s = +10 / −10、e / w = +100 / −100、Enter = ZERO 未記録なら ZERO 確定・記録済みなら FULL 確定、Tab = 軸切替
  const { useJogKeys } = window;
  const calibRef = useRef({});
  calibRef.current = { brk, acc };
  useJogKeys({
    enabled: started && !confirmStart && !popup,
    selectedAxis, setSelectedAxis,
    onJog: handleJog,
    onEnter: (axis) => {
      const ax = axis === 'brake' ? calibRef.current.brk : calibRef.current.acc;
      return ax.zero === null ? handleSetZero(axis) : handleSetFull(axis);
    },
  });

  // 手順の案内: いまどの手順にいるか（1 開始 / 2 ZERO・FULL を記録 / 3 保存）
  const currentStep = !started ? 1 : (canSave ? 3 : 2);
  const steps = [
    '「開始」を押す',
    '±ボタンかドラッグでペダルを動かし、踏んでいない位置で ZERO 確定・いちばん踏み込んだ位置で FULL 確定（ブレーキ・アクセルの両方）。キー: d/s = ±10、e/w = ±100、Enter = ZERO→FULL の順に確定、Tab = ブレーキ⇔アクセル',
    '「キャリブレーション保存」を押す',
  ];
  const doneMark = (z, f) => (z !== null && f !== null
    ? <span style={{ color: '#54bc54', fontWeight: 700 }}>済</span>
    : <span style={{ color: window.INK_MUTE }}>未</span>);

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 10, flex: 1, minHeight: 0 }}>

      {popup === 'no_profile' && (
        <ValidationPopup
          message="車両プロファイルを選択してください"
          actionLabel="プロファイルへ"
          onAction={() => setNav('profiles')}
        />
      )}

      {/* 手順の案内 */}
      <Box style={{ padding: '8px 14px', display: 'flex', gap: 18, alignItems: 'center', flexWrap: 'wrap', fontSize: 13 }}>
        {steps.map((text, i) => {
          const n = i + 1;
          const active = n === currentStep;
          const done = n < currentStep;
          return (
            <div key={n} style={{
              display: 'flex', gap: 6, alignItems: 'baseline',
              flex: n === 2 ? '1 1 360px' : '0 0 auto',
              color: active ? window.INK : window.INK_MUTE,
              fontWeight: active ? 700 : 400,
            }}>
              <span style={{ color: done ? '#54bc54' : (active ? '#c8922a' : window.INK_MUTE) }}>
                {done ? '✓' : `${n}.`}
              </span>
              <span>{text}</span>
            </div>
          );
        })}
      </Box>
      {!started && canStart && (
        <Note>「開始」を押すまでペダルは動かせません（ボタンは薄く表示されます）。</Note>
      )}
      {!started && !canStart && (
        <div style={{ display: 'flex', gap: 10, alignItems: 'center' }}>
          <Note style={{ flex: 1 }}>先に初期化を実行してください。初期化が終わる（READY になる）と「開始」を押せます（現在: {robotState}）。</Note>
          <Btn onClick={() => setNav('init')}>初期化へ</Btn>
        </div>
      )}

      {/* Top: axis panels (fill remaining height) */}
      <div style={{ display: 'flex', gap: 10, flex: 1, minHeight: 0 }}>
        <AxisCal
          label="ブレーキ" axisId="brake"
          currentPos={brk.currentPos} zero={brk.zero} full={brk.full}
          onJog={handleJog} onSetZero={handleSetZero} onSetFull={handleSetFull} onHome={handleHome}
          enabled={started} selected={selectedAxis === 'brake'} onSelect={setSelectedAxis}
        />
        <AxisCal
          label="アクセル" axisId="accel"
          currentPos={acc.currentPos} zero={acc.zero} full={acc.full}
          onJog={handleJog} onSetZero={handleSetZero} onSetFull={handleSetFull} onHome={handleHome}
          enabled={started} selected={selectedAxis === 'accel'} onSelect={setSelectedAxis}
        />
      </div>

      {/* Bottom: recorded + save button */}
      <div style={{ display: 'flex', gap: 10 }}>
        <div style={{ display: 'flex', gap: 10, flex: 1 }}>
          <Box label="記録済み" style={{ padding: '10px 14px', fontSize: 13, flex: 1 }}>
            <Row cells={[['状態', '48px'], ['軸', '1.4fr'], ['ZERO', '1fr'], ['FULL', '1fr']]} header />
            <Row cells={[
              [doneMark(brk.zero, brk.full), '48px'],
              ['ブレーキ', '1.4fr'],
              [brk.zero !== null ? String(brk.zero) : '—', '1fr'],
              [brk.full !== null ? String(brk.full) : '—', '1fr'],
            ]} />
            <Row cells={[
              [doneMark(acc.zero, acc.full), '48px'],
              ['アクセル', '1.4fr'],
              [acc.zero !== null ? String(acc.zero) : '—', '1fr'],
              [acc.full !== null ? String(acc.full) : '—', '1fr'],
            ]} />
          </Box>
          <div style={{ display: 'flex', flexDirection: 'column', flex: 1 }}>
            {started ? (
              <Btn primary big style={{ flex: 1 }} disabled={!canSave} onClick={handleSave}>
                キャリブレーション保存
              </Btn>
            ) : (
              <Btn big disabled={!canStart} style={{ flex: 1, borderColor: '#3c8c3c', background: '#0e220e', color: '#68d468' }}
                   onClick={() => { if (canStart) setConfirmStart(true); }}>
                開始
              </Btn>
            )}
          </div>
        </div>
      </div>

      {confirmStart && (
        <ConfirmStopPopup
          message="開始しますか？"
          onYes={() => { setStarted(true); setConfirmStart(false); }}
          onNo={() => setConfirmStart(false)}
        />
      )}

    </div>
  );
}

window.CalibrationScreen = CalibrationScreen;
