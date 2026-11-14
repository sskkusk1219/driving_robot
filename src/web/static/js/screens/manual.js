// ── Manual drive screen ───────────────────────────────────

function AxisJog({ label, axisId, currentPos, openingPct, maxPct, currentMa, onJog, onHome, enabled = true, selected = false, onSelect }) {
  const { INK, INK_SOFT, PAPER, HATCH, Btn } = window;
  const { AxisPanel, JogRow } = window;
  const dis = !enabled;

  return (
    <AxisPanel label={label} axisId={axisId} selected={selected} enabled={enabled} onSelect={onSelect}>

      {/* Jog buttons + drag slider */}
      <JogRow axisId={axisId} currentPos={currentPos} onJog={onJog} disabled={dis} />

      {/* Opening bar */}
      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 10, color: INK_SOFT, marginBottom: 3, fontFamily: 'inherit' }}>
          <span>0 %</span><span>50</span><span>100 % (max {maxPct ?? '—'})</span>
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
    </AxisPanel>
  );
}

function ManualScreen() {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, activeProfileId, realtimeData, robotState, setNav, setNavLock } = useContext(window.AppContext);
  const { Box, Btn, Note, Row } = window;
  const { ConfirmStopPopup, ValidationPopup, useJogKeys } = window;

  const [brk, setBrk] = useState({ currentPos: 0 });
  const [acc, setAcc] = useState({ currentPos: 0 });
  const [selectedAxis, setSelectedAxis] = useState('accel'); // キー操作の対象（Tab・パネルのクリックで切替）
  const [confirmStop, setConfirmStop] = useState(false);
  const [confirmStart, setConfirmStart] = useState(false);
  const [popup, setPopup] = useState(null);
  const [maxBrake, setMaxBrake] = useState(null);  // アクティブプロファイルの最大開度 [%]
  const [maxAccel, setMaxAccel] = useState(null);
  const [calib, setCalib] = useState(null);  // キャリブレーション（開度の換算に使う ZERO/FULL）
  const startedHereRef = useRef(false);  // この画面で開始した（位置は 0 と分かっている）

  const enabled = robotState === 'MANUAL';
  // 開始できるのは初期化済み（READY）のときだけ
  const canStart = robotState === 'READY';

  // アクティブプロファイルの最大開度を取得。プロファイル・キャリブレーションが無ければポップアップ
  useEffect(() => {
    if (!activeProfileId) {
      setPopup('no_profile');
      return;
    }
    apiFetch('GET', `/api/v1/profiles/${activeProfileId}`).then(p => {
      if (!p) return;
      if (!p.calibration?.is_valid) setPopup('no_calib');
      if (p.calibration) setCalib(p.calibration);
      if (p.max_brake_opening != null) setMaxBrake(p.max_brake_opening);
      if (p.max_accel_opening != null) setMaxAccel(p.max_accel_opening);
    });
  }, []);

  // 手動運転中は他ページへの離脱をロック
  useEffect(() => {
    setNavLock(enabled);
    return () => setNavLock(false);
  }, [enabled]);

  // 位置の表示: MANUAL 中にこの画面で開始していない（再読み込みなど）ときは位置が分からないので
  // 「—」にし、次のジョグの応答で埋める。step 0 のジョグで読み直す手は使わない
  // （位置がキャリブレーション範囲の外＝原点のままだと、範囲の端へペダルが動いてしまう）。
  // MANUAL でなくなったら（終了で両軸は原点へ戻る）表示を 0 に戻す。
  useEffect(() => {
    if (robotState === 'MANUAL') {
      if (!startedHereRef.current) {
        setBrk({ currentPos: null });
        setAcc({ currentPos: null });
      }
    } else {
      setBrk({ currentPos: 0 });
      setAcc({ currentPos: 0 });
    }
  }, [robotState]);

  // キー操作（キャリブレーションと同じ割り当て）: d/s = ±10、e/w = ±100、Tab = 軸切替
  useJogKeys({
    enabled: enabled && !confirmStart && !confirmStop && !popup,
    selectedAxis, setSelectedAxis,
    onJog: handleJog,
  });

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
    startedHereRef.current = true;
    const r = await apiFetch('POST', '/api/v1/drive/manual/start');
    if (!r) { startedHereRef.current = false; return; }
    window.showToast('手動運転を開始しました', 'success');
  }

  async function handleStop() {
    const r = await apiFetch('POST', '/api/v1/drive/manual/stop');
    if (r) {
      window.showToast('手動運転を終了しました', 'success');
      setConfirmStop(false);
      startedHereRef.current = false;
    }
  }

  // 開度 [%]: サーバーの開度は制御ループが動いている間しか出ない（手動運転では常に 0）ので、
  // 位置とキャリブレーションの ZERO/FULL から換算する。位置が分からないときは 0。
  const toOpening = (pos, zero, full) => (
    pos == null || zero == null || full == null || full === zero
      ? 0
      : Math.max(0, Math.min(100, ((pos - zero) / (full - zero)) * 100))
  );
  const brkOpeningPct = toOpening(brk.currentPos, calib?.brake_zero_pos, calib?.brake_full_pos);
  const accOpeningPct = toOpening(acc.currentPos, calib?.accel_zero_pos, calib?.accel_full_pos);
  const brkMa = realtimeData.brake_current_ma ?? 0;
  const accMa = realtimeData.accel_current_ma ?? 0;

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 10, flex: 1, minHeight: 0 }}>

      {popup === 'no_profile' && (
        <ValidationPopup
          message="車両プロファイルを選択してください"
          actionLabel="プロファイルへ"
          onAction={() => setNav('profiles')}
        />
      )}
      {popup === 'no_calib' && (
        <ValidationPopup
          message="キャリブレーションデータがありません"
          actionLabel="キャリブレーションへ"
          onAction={() => setNav('calibration')}
        />
      )}

      {!enabled && !canStart && (
        <div style={{ display: 'flex', gap: 10, alignItems: 'center' }}>
          <Note style={{ flex: 1 }}>先に初期化を実行してください。初期化が終わる（READY になる）と「開始」を押せます（現在: {robotState}）。</Note>
          <Btn onClick={() => setNav('init')}>初期化へ</Btn>
        </div>
      )}

      {/* Top: axis panels (fill remaining height) */}
      <div style={{ display: 'flex', gap: 10, flex: 1, minHeight: 0 }}>
        <AxisJog
          label="ブレーキ" axisId="brake"
          currentPos={brk.currentPos}
          openingPct={brkOpeningPct} maxPct={maxBrake} currentMa={brkMa}
          onJog={handleJog} onHome={handleHome}
          enabled={enabled} selected={selectedAxis === 'brake'} onSelect={setSelectedAxis}
        />
        <AxisJog
          label="アクセル" axisId="accel"
          currentPos={acc.currentPos}
          openingPct={accOpeningPct} maxPct={maxAccel} currentMa={accMa}
          onJog={handleJog} onHome={handleHome}
          enabled={enabled} selected={selectedAxis === 'accel'} onSelect={setSelectedAxis}
        />
      </div>

      {/* Bottom: shortcuts | status + button */}
      <div style={{ display: 'flex', gap: 10 }}>
        <Box label="キーボードショートカット" style={{ padding: '10px 12px', flex: 1 }}>
          <div style={{ fontSize: 12, lineHeight: 1.7, fontFamily: 'inherit' }}>
            <div>d / s  : ±10 pulse</div>
            <div>e / w  : ±100 pulse</div>
            <div>Tab    : 軸切替 (アクセル ↔ ブレーキ)</div>
            <div>パネルをクリック : その軸を操作対象にする</div>
          </div>
          <Note style={{ marginTop: 6 }}>ジョグ中は最大開度リミットが有効です。電流値を常時監視中。</Note>
        </Box>

        <div style={{ display: 'flex', gap: 10, flex: 1 }}>
          <Box style={{ padding: '10px 14px', fontSize: 13, flex: 1 }}>
            <Row cells={[['実車速',       '1.4fr'], [`${(realtimeData.actual_speed_kmh ?? 0).toFixed(1)} km/h`, '1fr', 'mono']]} />
            <Row cells={[['ブレーキ pos', '1.4fr'], [brk.currentPos === null ? "—" : `${brk.currentPos} pulse`, '1fr', 'mono']]} />
            <Row cells={[['アクセル pos', '1.4fr'], [acc.currentPos === null ? "—" : `${acc.currentPos} pulse`, '1fr', 'mono']]} />
          </Box>
          <div style={{ display: 'flex', flexDirection: 'column', flex: 1 }}>
            {robotState === 'MANUAL' ? (
              <Btn danger big style={{ flex: 1 }} onClick={() => setConfirmStop(true)}>■ 運転終了</Btn>
            ) : (
              <Btn big disabled={!canStart} style={{ flex: 1, borderColor: '#3c8c3c', background: '#0e220e', color: '#68d468' }}
                   onClick={() => { if (canStart) setConfirmStart(true); }}>開始</Btn>
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
