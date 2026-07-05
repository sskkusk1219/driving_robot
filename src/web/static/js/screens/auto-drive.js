// ── Auto-drive monitor screen ─────────────────────────────
// AutoDriveD layout: 3-axis graph + BigSpeed + session info + stop

function DriveMonitorScreen({
  showModeAxis = true, showPause = false, profileMaxSpeed = null,
  screenTitle = '自動走行モニター', driveStartPath = '/api/v1/drive/start', driveStartBody = null,
  driveStopPath = '/api/v1/drive/stop', driveArmPath = null, driveCancelPath = null,
  confirmStartMessage = '開始しますか？', resultPanel = null,
  modeRowLabel = showModeAxis ? '走行モード' : null,
  confirmOnly = false, busy = false, busyLabel = null, onAbort = null,
  startedToastMessage = '走行を開始しました',
}) {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, realtimeData, realtimeBuf, activeModeId, activeModeName, activeModeKind, robotState, setNavLock } = useContext(window.AppContext);
  const { INK, INK_SOFT, PAPER, PAPER_2, HATCH, Box, Btn, Row, BigSpeed } = window;

  const [modeDetail, setModeDetail] = useState(null);
  const [nowS, setNowS] = useState(0);
  const [confirmStop, setConfirmStop] = useState(false);
  const [confirmStart, setConfirmStart] = useState(false);
  const [isDriving, setIsDriving] = useState(false);
  const driveStartTimeRef = useRef(null);
  const confirmStartRef = useRef(false);
  // 一時停止区間の累積実時間 [ms] と、現在の一時停止開始時刻。プレイヘッド(nowS)を
  // バックエンドの _started_at シフトと整合させ、再開後もタイムラインを連続させる。
  const pausedAccumMsRef = useRef(0);
  const pauseStartRef = useRef(null);

  // 確認ポップアップ表示中（arm 済み・未確定）に画面を離脱したら、保持ブレーキを
  // 取り残さないよう best-effort で cancel する。confirmStartRef でアンマウント時のみ発火。
  useEffect(() => { confirmStartRef.current = confirmStart; }, [confirmStart]);
  useEffect(() => () => {
    if (confirmStartRef.current && driveCancelPath) apiFetch('POST', driveCancelPath);
  }, []);

  // 走行中（RUNNING / PAUSED）または busy（学習サイクル等の背後処理中）は他ページへの離脱をロック
  useEffect(() => {
    const driving = robotState === 'RUNNING' || robotState === 'PAUSED' || busy;
    setNavLock(driving);
    return () => setNavLock(false);
  }, [robotState, busy]);

  // T1: 走行中（RUNNING / PAUSED）のみ経過時間を進める。
  // グラフの横スクロールを滑らかにするため、整数秒の setInterval ではなく
  // requestAnimationFrame で小数秒（nowS）を更新し、毎フレーム再描画する。
  // （旧実装は 1Hz 更新だったため、ウィンドウが 1 秒ごとにカクッと左へ跳ねていた）
  // 一時停止（PAUSED）中は rAF を回さず nowS を凍結し、再開時に一時停止区間の実時間を
  // pausedAccumMs に加算してタイムライン（プレイヘッド）を連続させる。
  useEffect(() => {
    const driving = robotState === 'RUNNING' || robotState === 'PAUSED';
    if (!driving) {
      driveStartTimeRef.current = null;
      pausedAccumMsRef.current = 0;
      pauseStartRef.current = null;
      setIsDriving(false);
      setNowS(0);
      return;
    }
    if (!driveStartTimeRef.current) {
      driveStartTimeRef.current = Date.now();
      pausedAccumMsRef.current = 0;
      pauseStartRef.current = null;
      setIsDriving(true);
    }
    if (robotState === 'PAUSED') {
      // 一時停止: プレイヘッドを凍結し、再開に備えて一時停止開始時刻を記録する
      if (pauseStartRef.current === null) pauseStartRef.current = Date.now();
      return;
    }
    // RUNNING: 直前が一時停止なら、その実時間を累積へ加えてタイムラインを連続させる
    if (pauseStartRef.current !== null) {
      pausedAccumMsRef.current += Date.now() - pauseStartRef.current;
      pauseStartRef.current = null;
    }
    let raf;
    const tick = () => {
      if (driveStartTimeRef.current) {
        setNowS((Date.now() - driveStartTimeRef.current - pausedAccumMsRef.current) / 1000);
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [robotState]);

  // 表示用の整数秒（mm:ss ラベルは秒単位で十分）
  const elapsed = Math.floor(nowS);

  useEffect(() => {
    if (activeModeId && activeModeKind !== 'schedule') {
      apiFetch('GET', `/api/v1/modes/${activeModeId}`).then(d => { if (d) setModeDetail(d); });
    } else {
      setModeDetail(null);
    }
  }, [activeModeId, activeModeKind]);

  async function handleStart() {
    // confirmOnly フロー（学習サイクル等）: driveStartPath 側が自前で arm/precheck を
    // 同期実行するため、事前の arm HTTP 呼び出しはせず確認ポップアップのみ出す。
    if (confirmOnly) { setConfirmStart(true); return; }
    // arm フロー（学習運転）: arm → 確認ポップアップ → はい/いいえ
    if (driveArmPath) {
      const r = await apiFetch('POST', driveArmPath);
      if (r) setConfirmStart(true);
      return;
    }
    const r = await apiFetch('POST', driveStartPath, driveStartBody);
    if (r) window.showToast(startedToastMessage, 'success');
  }

  async function handleConfirmStartYes() {
    const r = await apiFetch('POST', driveStartPath, driveStartBody);
    if (r) { window.showToast(startedToastMessage, 'success'); setConfirmStart(false); }
  }

  async function handleConfirmStartNo() {
    if (driveCancelPath) await apiFetch('POST', driveCancelPath);
    setConfirmStart(false);
  }

  async function handleStop() {
    const r = await apiFetch('POST', driveStopPath);
    if (r) { window.showToast('走行を終了しました', 'success'); setConfirmStop(false); }
  }

  async function handlePause() {
    const r = await apiFetch('POST', '/api/v1/drive/pause');
    if (r) window.showToast('一時停止しました', 'success');
  }

  async function handleResume() {
    const r = await apiFetch('POST', '/api/v1/drive/resume');
    if (r) window.showToast('走行を再開しました', 'success');
  }

  const buf = realtimeBuf.current;
  const driveStart = driveStartTimeRef.current;
  // axis 2 (開度) は走行後のバッファデータのみ
  const recent = driveStart
    ? buf.filter(d => d.ts >= driveStart).slice(-300)
    : [];

  // Graph constants
  const VW = 900, PL = 50, PR = 16, PH1 = 120, PH2 = 120, PH3 = 60;
  const PW = VW - PL - PR;
  const G_PAD_T = 8, G_PAD_B = 2;

  function toX(i, len) { return PL + (i / Math.max(1, len - 1)) * PW; }
  function toXFull(frac) { return PL + frac * PW; }
  function toY(val, valMax, height, padT = G_PAD_T, padB = G_PAD_B) {
    return (height - padB) - (Math.min(val, valMax) / valMax) * (height - padT - padB);
  }

  function polyline(pts, color, strokeW = 2, dasharray) {
    if (pts.length < 2) return null;
    const d = pts.map((p, i) => `${i === 0 ? 'M' : 'L'}${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ');
    return <path d={d} fill="none" stroke={color} strokeWidth={strokeW} strokeDasharray={dasharray} strokeLinejoin="round" strokeLinecap="round" />;
  }

  // 現在位置ポインタ: 中央(frac=0.5)に固定し、現在のセンサ値で上下にのみ動く。
  // 軌跡(polyline)は左へ流れ、その先端にこの●が常に乗る（走行前=0でも表示）。
  // SVG は preserveAspectRatio="none" で非等比に伸縮するため、円を <circle> で描くと
  // グラフの縦横比に応じて楕円に歪む（軸3を持たない学習運転ページで顕著）。
  // 固定 px サイズの HTML 要素をパーセント位置で重ねることで、常に真円を保つ。
  function marker(val, valMax, height, color) {
    return <div style={{
      position: 'absolute',
      left: `${(toXFull(0.5) / VW) * 100}%`,
      top: `${(toY(val, valMax, height) / height) * 100}%`,
      width: 12, height: 12, boxSizing: 'border-box',
      borderRadius: '50%', background: color, border: `1.5px solid ${PAPER}`,
      transform: 'translate(-50%, -50%)', pointerEvents: 'none',
    }} />;
  }

  // Axis 3: profile data
  const refProfile = modeDetail?.reference_speed ?? [];
  const totalDurS = refProfile.length > 0 ? refProfile[refProfile.length - 1].time_s : 0;

  const profileSpeedMax = refProfile.reduce((m, p) => Math.max(m, p.speed_kmh), 0);
  const maxSpeed = Math.max(100, Math.ceil(Math.max(profileSpeedMax, profileMaxSpeed ?? 0) / 20) * 20);

  function getRefSpeedAtTime(elapsedS) {
    if (refProfile.length === 0 || elapsedS < 0) return null;
    let speed = refProfile[0].speed_kmh;
    for (const p of refProfile) {
      if (p.time_s <= elapsedS) speed = p.speed_kmh;
      else break;
    }
    return speed;
  }

  // Axis 1: 30秒ウィンドウ・中央固定プレイヘッド（時間軸基準）
  // 現在時刻(nowS)を常にグラフ中央(frac=0.5)に固定し、波形は左へ流れる。
  // 左半分=過去(実測+基準)、中央=現在、右半分=未来(基準の先読み)。
  const WINDOW_S = 30;
  const HALF_S = WINDOW_S / 2;
  const windowStartS = nowS - HALF_S;   // クランプしない（now を常に中央に保つ）。走行前は nowS=0 → [-15, +15]
  const windowEndS   = nowS + HALF_S;

  const toFrac = elapsedS => (elapsedS - windowStartS) / WINDOW_S;
  // 画面外(frac<0)の古い点は左端への山積みを避けるため除外する
  const onScreen = (d, key, max, h) => {
    const frac = toFrac((d.ts - driveStart) / 1000);
    return frac >= 0 && frac <= 1
      ? { x: toXFull(frac), y: toY(d[key], max, h) }
      : null;
  };

  // 基準車速: プロファイルから直接サンプリング（モードロード済なら常に表示）
  const speedRef_pts = refProfile.length > 0
    ? Array.from({ length: 150 }, (_, i) => {
        const frac  = i / 149;
        const t     = windowStartS + frac * WINDOW_S;
        return { x: toXFull(frac), y: toY(getRefSpeedAtTime(t) ?? 0, maxSpeed, PH1) };
      })
    : [];

  // 実車速: 同一時間軸に位置合わせ（走行中のみ）。最新点は中央(frac=0.5)に張り付く
  const speedAct_pts = isDriving && driveStart
    ? recent.map(d => onScreen(d, 'actual_speed_kmh', maxSpeed, PH1)).filter(Boolean)
    : [];

  // Axis 2: openings — axis 1 と共通の時間軸
  const accelPts = isDriving && driveStart
    ? recent.map(d => onScreen(d, 'accel_opening', 100, PH2)).filter(Boolean)
    : [];
  const brakePts = isDriving && driveStart
    ? recent.map(d => onScreen(d, 'brake_opening', 100, PH2)).filter(Boolean)
    : [];

  // T4: マーカーと進捗は走行中のみ
  // Axis 3: PAD_T=8, PAD_B=8 → speed=0 が baseline(y=PH3-8=52)に一致
  const G3_PAD_T = 8, G3_PAD_B = 8;
  const profilePts = refProfile.map(p => ({
    x: toXFull(totalDurS > 0 ? p.time_s / totalDurS : 0),
    y: toY(p.speed_kmh, maxSpeed, PH3, G3_PAD_T, G3_PAD_B),
  }));
  const progressFrac = isDriving && totalDurS > 0 ? Math.min(1, nowS / totalDurS) : 0;
  const markerX = toXFull(progressFrac);

  const fmt = s => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;

  function GraphSvg({ height, children, yMax, unit, unitLabel, padT = G_PAD_T, padB = G_PAD_B }) {
    const ticks = [0, 0.25, 0.5, 0.75, 1].map(f => ({
      v: Math.round(f * yMax),
      y: (height - padB) - f * (height - padT - padB),
    }));
    return (
      <svg viewBox={`0 0 ${VW} ${height}`} width="100%" height="100%" preserveAspectRatio="none" style={{ display: 'block' }}>
        {unitLabel && <text x={2} y={padT} textAnchor="start" fontSize="10" fill={INK_SOFT}>{unitLabel}</text>}
        {ticks.map(({ v, y }, i) => (
          <g key={v}>
            <line x1={PL} y1={y} x2={PL + PW} y2={y} stroke={HATCH} strokeWidth="1" strokeDasharray="3 4" />
            <text x={PL - 4} y={i === 0 ? y - 3 : y + 4} textAnchor="end" fontSize="11" fill={INK}>{v}{unit}</text>
          </g>
        ))}
        {children}
      </svg>
    );
  }

  const rd = realtimeData;

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 10, flex: 1, minHeight: 0 }}>

      {/* Axis 1: speed graph — 1と2軸は同じ高さ */}
      <Box style={{ padding: '6px 0 2px', flex: 3, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
        <div style={{ flex: 1, minHeight: 0, position: 'relative' }}>
          <GraphSvg height={PH1} yMax={maxSpeed} unit="" unitLabel="km/h">
            {showModeAxis && polyline(speedRef_pts, INK_SOFT, 1.5, '4 3')}
            {polyline(speedAct_pts, '#c8922a', 2.2)}
          </GraphSvg>
          {marker(rd.actual_speed_kmh, maxSpeed, PH1, '#c8922a')}
        </div>
        <div style={{ display: 'flex', gap: 20, fontSize: 12, color: INK_SOFT, paddingLeft: PL, paddingBottom: 4, alignItems: 'center' }}>
          {showModeAxis && (
            <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
              <svg width="28" height="10" style={{ flexShrink: 0 }}>
                <line x1="0" y1="5" x2="28" y2="5" stroke={INK_SOFT} strokeWidth="1.5" strokeDasharray="4 3" />
              </svg>
              基準
            </span>
          )}
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#c8922a" strokeWidth="2.2" />
            </svg>
            実測
          </span>
        </div>
      </Box>

      {/* Axis 2: openings graph — 1と2軸は同じ高さ */}
      <Box style={{ padding: '6px 0 2px', flex: 3, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
        <div style={{ flex: 1, minHeight: 0, position: 'relative' }}>
          <GraphSvg height={PH2} yMax={100} unit="%">
            {polyline(accelPts, '#78c8f0', 2.2)}
            {polyline(brakePts, '#f07070', 2.2)}
          </GraphSvg>
          {marker(rd.accel_opening, 100, PH2, '#78c8f0')}
          {marker(rd.brake_opening, 100, PH2, '#f07070')}
        </div>
        {/* 共通時間軸ラベル（5秒ごと） */}
        <svg viewBox={`0 0 ${VW} 18`} width="100%" height="18" preserveAspectRatio="none" style={{ display: 'block' }}>
          <line x1={PL} y1={0} x2={PL + PW} y2={0} stroke={HATCH} strokeWidth="0.5" />
          <text x={PL + PW + 8} y={15} textAnchor="start" fontSize="10" fill={INK_SOFT}>s</text>
          {(() => {
            const first = Math.max(0, Math.ceil(windowStartS / 5) * 5);
            const ticks = [];
            for (let t = first; t <= windowEndS; t += 5) ticks.push(t);
            return ticks.map(t => {
              const x = toXFull((t - windowStartS) / WINDOW_S);
              return (
                <g key={t}>
                  <line x1={x} y1={0} x2={x} y2={4} stroke={HATCH} strokeWidth="1" />
                  <text x={x} y={15} textAnchor="middle" fontSize="10" fill={INK_SOFT}>{t}</text>
                </g>
              );
            });
          })()}
        </svg>
        <div style={{ display: 'flex', gap: 20, fontSize: 12, color: INK_SOFT, paddingLeft: PL, paddingBottom: 4, alignItems: 'center' }}>
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#f07070" strokeWidth="2.2" />
            </svg>
            ブレーキ
          </span>
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#78c8f0" strokeWidth="2.2" />
            </svg>
            アクセル
          </span>
        </div>
      </Box>

      {/* Axis 3: full profile overview — 走行モード使用時のみ表示 */}
      {showModeAxis && (
        <Box style={{ padding: '6px 0 2px', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
          <div style={{ flex: 1, minHeight: 0 }}>
            <svg viewBox={`0 0 ${VW} ${PH3}`} width="100%" height="100%" preserveAspectRatio="none" style={{ display: 'block' }}>
              <line x1={PL} y1={PH3 - G3_PAD_B} x2={PL + PW} y2={PH3 - G3_PAD_B} stroke={HATCH} strokeWidth="0.8" />
              {profilePts.length > 0 && (() => {
                const d = profilePts.map((p, i) => `${i === 0 ? 'M' : 'L'}${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ');
                return <path d={d} fill="none" stroke={INK_SOFT} strokeWidth="1.5" strokeDasharray="4 3" />;
              })()}
              {isDriving && totalDurS > 0 && (
                <>
                  <line x1={markerX} y1={4} x2={markerX} y2={PH3 - 4} stroke="#f07070" strokeWidth="2" />
                  <text x={markerX} y={14} textAnchor="middle" fontSize="13" fill="#f07070">▼</text>
                </>
              )}
            </svg>
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 12, fontFamily: 'inherit', paddingLeft: PL, paddingRight: PR, paddingBottom: 4, color: INK_SOFT }}>
            <span>{activeModeName ?? '—'}</span>
            <span style={{ color: '#a23232', fontWeight: 700 }}>
              {isDriving && totalDurS > 0 ? `${fmt(elapsed)} / ${fmt(totalDurS)}  (${Math.round(progressFrac * 100)}%)` : '—'}
            </span>
            <span>残 {isDriving && totalDurS > 0 ? fmt(Math.max(0, totalDurS - elapsed)) : '—'}</span>
          </div>
        </Box>
      )}

      {/* Bottom: BigSpeed + profile | session info + buttons */}
      <div style={{ display: 'grid', gridTemplateColumns: '0.8fr 1.2fr', gap: 10 }}>
        <Box style={{ display: 'flex', gap: 10, alignItems: 'center', padding: 8 }}>
          <BigSpeed value={rd.actual_speed_kmh} refSpeed={rd.ref_speed_kmh ?? null} size={84} showRef={showModeAxis} />
          <Box style={{ flex: 1, padding: '6px 10px', display: 'flex', alignItems: 'center' }}>
            <div style={{ fontSize: 12, color: INK_SOFT }}>アクセル {rd.accel_opening.toFixed(1)}% / ブレーキ {rd.brake_opening.toFixed(1)}%</div>
          </Box>
        </Box>
        <div style={{ display: 'flex', gap: 10, alignItems: 'stretch' }}>
          <Box style={{ padding: '8px 12px', fontSize: 13, flex: 1.6, display: 'flex', flexDirection: 'column', minHeight: 0 }}>
            {/* T6: 経過時間は走行中のみ表示 */}
            <Row cells={[['経過時間', '1.4fr'], [isDriving ? fmt(elapsed) : '—', '1fr', 'mono']]} />
            {modeRowLabel && <Row cells={[[modeRowLabel, '1.4fr'], [activeModeName ?? '—', '1fr']]} />}
            {showModeAxis && <Row cells={[['全体時間', '1.4fr'], [totalDurS > 0 ? fmt(totalDurS) : '—', '1fr', 'mono']]} />}
            {resultPanel && (
              // 高さを固定し、内容量（未実行/学習中/完了）で枠が伸縮しないようにする。
              // はみ出す分は内部スクロールで見せる。
              <div style={{ marginTop: 6, overflowY: 'auto', height: 72 }}>
                {resultPanel}
              </div>
            )}
          </Box>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 8, flex: 1 }}>
            {busy ? (
              <>
                <div style={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center', textAlign: 'center', fontSize: 13, color: INK_SOFT, border: `1px solid ${HATCH}`, borderRadius: 4, padding: 8 }}>
                  {busyLabel ?? '実行中…'}
                </div>
                {onAbort && <Btn danger big style={{ flex: 1 }} onClick={onAbort}>■ 中断</Btn>}
              </>
            ) : robotState === 'RUNNING' ? (
              <>
                {showPause && (
                  <Btn big style={{ flex: 1, borderColor: INK_SOFT, background: PAPER_2, color: INK }} onClick={handlePause}>⏸ 一時停止</Btn>
                )}
                <Btn danger big style={{ flex: 1 }} onClick={() => setConfirmStop(true)}>■ 走行終了</Btn>
              </>
            ) : robotState === 'PAUSED' ? (
              <>
                <Btn big style={{ flex: 1, borderColor: '#3c8c3c', background: '#0e220e', color: '#68d468' }} onClick={handleResume}>▶ 走行再開</Btn>
                <Btn danger big style={{ flex: 1 }} onClick={() => setConfirmStop(true)}>■ 走行終了</Btn>
              </>
            ) : (
              <Btn big style={{ flex: 1, borderColor: '#3c8c3c', background: '#0e220e', color: '#68d468' }} onClick={handleStart}>開始</Btn>
            )}
          </div>
        </div>
      </div>

      {confirmStop && React.createElement(window.ConfirmStopPopup, {
        message: '走行を終了しますか？',
        onYes: handleStop,
        onNo: () => setConfirmStop(false),
      })}

      {confirmStart && React.createElement(window.ConfirmStopPopup, {
        message: confirmStartMessage,
        onYes: handleConfirmStartYes,
        onNo: handleConfirmStartNo,
      })}
    </div>
  );
}

function AutoDriveScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeProfileId, activeModeId, activeModeKind, setNav } = useContext(window.AppContext);
  const { ValidationPopup } = window;

  const [popup, setPopup] = useState(null);
  const isSchedule = activeModeKind === 'schedule';

  useEffect(() => {
    if (!activeProfileId) { setPopup('no_profile'); return; }
    if (!activeModeId)    { setPopup('no_mode'); return; }
    apiFetch('GET', `/api/v1/profiles/${activeProfileId}`).then(p => {
      if (p && !p.calibration?.is_valid) setPopup('no_calib');
    });
  }, []);

  const POPUP_CONFIG = {
    no_profile: { message: '車両プロファイルを選択してください',     actionLabel: 'プロファイルへ',       nav: 'profiles' },
    no_calib:   { message: 'キャリブレーションデータがありません',   actionLabel: 'キャリブレーションへ', nav: 'calibration' },
    no_mode:    { message: '走行モードまたはタイムスケジュールを選択してください', actionLabel: '走行モードへ', nav: 'modes' },
  };

  const cfg = popup ? POPUP_CONFIG[popup] : null;

  return (
    <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
      {isSchedule ? (
        <DriveMonitorScreen showPause={false} showModeAxis={false} modeRowLabel="スケジュール" screenTitle="自動走行モニター"
          driveStartPath="/api/v1/drive/schedule/start"
          driveStartBody={activeModeId ? { schedule_id: activeModeId } : null}
          driveStopPath="/api/v1/drive/schedule/stop"
          confirmStartMessage="開始しますか？" />
      ) : (
        <DriveMonitorScreen showPause={true} screenTitle="自動走行モニター"
          driveStartPath="/api/v1/drive/start"
          driveStartBody={activeModeId ? { mode_id: activeModeId } : null}
          driveStopPath="/api/v1/drive/stop"
          driveArmPath="/api/v1/drive/arm"
          driveCancelPath="/api/v1/drive/cancel"
          confirmStartMessage="開始しますか？" />
      )}
      {cfg && (
        <ValidationPopup
          message={cfg.message}
          actionLabel={cfg.actionLabel}
          onAction={() => setNav(cfg.nav)}
        />
      )}
    </div>
  );
}

window.DriveMonitorScreen = DriveMonitorScreen;
window.AutoDriveScreen = AutoDriveScreen;
