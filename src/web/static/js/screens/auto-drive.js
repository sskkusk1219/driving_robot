// ── Auto-drive monitor screen ─────────────────────────────
// AutoDriveD layout: 3-axis graph + BigSpeed + session info + pause/stop

function DriveMonitorScreen({ showPause = true, showModeAxis = true, profileMaxSpeed = null, screenTitle = '自動走行モニター', driveStartPath = '/api/v1/drive/start', driveStartBody = null }) {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, realtimeData, realtimeBuf, activeModeId, activeModeName, activeProfileName, robotState } = useContext(window.AppContext);
  const { INK, INK_SOFT, PAPER, PAPER_2, HATCH, Box, Btn, Row, BigSpeed } = window;

  const [modeDetail, setModeDetail] = useState(null);
  const [elapsed, setElapsed] = useState(0);
  const [paused, setPaused] = useState(false);
  const [confirmStop, setConfirmStop] = useState(false);
  const [isDriving, setIsDriving] = useState(false);
  const driveStartTimeRef = useRef(null);

  // T1: タイマーを robotState=RUNNING 時のみ動作させる
  useEffect(() => {
    if (robotState !== 'RUNNING') {
      driveStartTimeRef.current = null;
      setIsDriving(false);
      return;
    }
    if (!driveStartTimeRef.current) {
      driveStartTimeRef.current = Date.now();
      setElapsed(0);
      setIsDriving(true);
    }
    const id = setInterval(() => {
      if (driveStartTimeRef.current) {
        setElapsed(Math.floor((Date.now() - driveStartTimeRef.current) / 1000));
      }
    }, 1000);
    return () => clearInterval(id);
  }, [robotState]);

  useEffect(() => {
    if (activeModeId) {
      apiFetch('GET', `/api/v1/modes/${activeModeId}`).then(d => { if (d) setModeDetail(d); });
    }
  }, [activeModeId]);

  async function handleStart() {
    const r = await apiFetch('POST', driveStartPath, driveStartBody);
    if (r) window.showToast('走行を開始しました', 'success');
  }

  async function handleStop() {
    const r = await apiFetch('POST', '/api/v1/drive/stop');
    if (r) { window.showToast('走行を終了しました', 'success'); setConfirmStop(false); }
  }

  async function handlePause() {
    const endpoint = paused ? '/api/v1/drive/resume' : '/api/v1/drive/pause';
    const r = await apiFetch('POST', endpoint);
    if (r) { setPaused(p => !p); window.showToast(paused ? '走行を再開しました' : '一時停止しました', 'success'); }
  }

  // T2: グラフデータを走行開始後のみに絞る（30秒ウィンドウ）
  const buf = realtimeBuf.current;
  const WINDOW = 300; // 30s at 100ms
  const driveStart = driveStartTimeRef.current;
  const recent = driveStart
    ? buf.filter(d => d.ts >= driveStart).slice(-WINDOW)
    : [];

  // Graph constants
  const VW = 900, PL = 50, PR = 16, PH1 = 120, PH2 = 120, PH3 = 60;
  const PW = VW - PL - PR;
  // padB=2: y=0グリッド線をチャート底辺にほぼ一致させる
  const G_PAD_T = 8, G_PAD_B = 2;

  function toX(i, len) { return PL + (i / Math.max(1, len - 1)) * PW; }
  function toXFull(frac) { return PL + frac * PW; }
  function toY(val, valMax, height, padT = G_PAD_T, padB = G_PAD_B) {
    return (height - padB) - (Math.min(val, valMax) / valMax) * (height - padT - padB);
  }

  function polyline(pts, color, strokeW = 2, dasharray) {
    if (pts.length < 2) return null;
    const d = pts.map((p, i) => `${i === 0 ? 'M' : 'L'}${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ');
    return <path d={d} fill="none" stroke={color} strokeWidth={strokeW} strokeDasharray={dasharray} />;
  }

  // Axis 3: profile data
  const refProfile = modeDetail?.reference_speed ?? [];
  const totalDurS = refProfile.length > 0 ? refProfile[refProfile.length - 1].time_s : 0;

  // maxSpeed: モードプロファイルまたは車両プロファイルの最大速度を20の倍数に切り上げ（最小100）
  const profileSpeedMax = refProfile.reduce((m, p) => Math.max(m, p.speed_kmh), 0);
  const maxSpeed = Math.max(100, Math.ceil(Math.max(profileSpeedMax, profileMaxSpeed ?? 0) / 20) * 20);

  // T3: プロファイルから基準車速をステップ関数で算出
  function getRefSpeedAtTime(elapsedS) {
    if (refProfile.length === 0 || elapsedS < 0) return null;
    let speed = refProfile[0].speed_kmh;
    for (const p of refProfile) {
      if (p.time_s <= elapsedS) speed = p.speed_kmh;
      else break;
    }
    return speed;
  }

  // Axis 1: speed — 基準車速はサーバー値優先、なければプロファイルから計算
  const speedRef_pts = recent.map((d, i) => {
    const elapsedS = driveStart ? (d.ts - driveStart) / 1000 : 0;
    const refSpeed = d.ref_speed_kmh ?? getRefSpeedAtTime(elapsedS);
    return { x: toX(i, recent.length), y: toY(refSpeed ?? 0, maxSpeed, PH1) };
  });
  const speedAct_pts = recent.map((d, i) => ({
    x: toX(i, recent.length),
    y: toY(d.actual_speed_kmh, maxSpeed, PH1),
  }));

  // Axis 2: openings
  const accelPts = recent.map((d, i) => ({
    x: toX(i, recent.length),
    y: toY(d.accel_opening, 100, PH2),
  }));
  const brakePts = recent.map((d, i) => ({
    x: toX(i, recent.length),
    y: toY(d.brake_opening, 100, PH2),
  }));

  // T4: マーカーと進捗は走行中のみ
  // Axis 3: PAD_T=8, PAD_B=8 → speed=0 が baseline(y=PH3-8=52)に一致
  const G3_PAD_T = 8, G3_PAD_B = 8;
  const profilePts = refProfile.map(p => ({
    x: toXFull(totalDurS > 0 ? p.time_s / totalDurS : 0),
    y: toY(p.speed_kmh, maxSpeed, PH3, G3_PAD_T, G3_PAD_B),
  }));
  const progressFrac = isDriving && totalDurS > 0 ? Math.min(1, elapsed / totalDurS) : 0;
  const markerX = toXFull(progressFrac);

  const fmt = s => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;

  function GraphSvg({ height, children, yMax, unit, unitLabel, padT = G_PAD_T, padB = G_PAD_B }) {
    const ticks = [0, 0.25, 0.5, 0.75, 1].map(f => ({
      v: Math.round(f * yMax),
      y: (height - padB) - f * (height - padT - padB),
    }));
    return (
      <svg viewBox={`0 0 ${VW} ${height}`} width="100%" height="100%" preserveAspectRatio="none" style={{ display: 'block' }}>
        {unitLabel && <text x={2} y={padT} textAnchor="start" fontSize="10" fill="#888">{unitLabel}</text>}
        {ticks.map(({ v, y }, i) => (
          <g key={v}>
            <line x1={PL} y1={y} x2={PL + PW} y2={y} stroke={HATCH} strokeWidth="1" strokeDasharray="3 4" />
            <text x={PL - 4} y={i === 0 ? y - 3 : y + 4} textAnchor="end" fontSize="10" fill="#666">{v}{unit}</text>
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
      <Box style={{ padding: '6px 0 2px', flex: 2, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
        <div style={{ flex: 1, minHeight: 0 }}>
          <GraphSvg height={PH1} yMax={maxSpeed} unit="" unitLabel="km/h">
            {polyline(speedRef_pts, '#888', 1.5, '4 3')}
            {polyline(speedAct_pts, '#2f5780', 2.2)}
          </GraphSvg>
        </div>
        <div style={{ display: 'flex', gap: 20, fontSize: 12, color: INK_SOFT, paddingLeft: PL, paddingBottom: 4, alignItems: 'center' }}>
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#888" strokeWidth="1.5" strokeDasharray="4 3" />
            </svg>
            基準
          </span>
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#2f5780" strokeWidth="2.2" />
            </svg>
            実測
          </span>
        </div>
      </Box>

      {/* Axis 2: openings graph — 1と2軸は同じ高さ */}
      <Box style={{ padding: '6px 0 2px', flex: 2, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
        <div style={{ flex: 1, minHeight: 0 }}>
          <GraphSvg height={PH2} yMax={100} unit="%">
            {polyline(accelPts, '#2f5780', 2.2)}
            {polyline(brakePts, '#a23232', 2.2)}
          </GraphSvg>
        </div>
        <div style={{ display: 'flex', gap: 20, fontSize: 12, color: INK_SOFT, paddingLeft: PL, paddingBottom: 4, alignItems: 'center' }}>
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#a23232" strokeWidth="2.2" />
            </svg>
            ブレーキ
          </span>
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <svg width="28" height="10" style={{ flexShrink: 0 }}>
              <line x1="0" y1="5" x2="28" y2="5" stroke="#2f5780" strokeWidth="2.2" />
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
                return <path d={d} fill="none" stroke="#888" strokeWidth="1.5" strokeDasharray="4 3" />;
              })()}
              {isDriving && totalDurS > 0 && (
                <>
                  <line x1={markerX} y1={4} x2={markerX} y2={PH3 - 4} stroke="#a23232" strokeWidth="2" />
                  <text x={markerX} y={14} textAnchor="middle" fontSize="13" fill="#a23232">▼</text>
                </>
              )}
            </svg>
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 12, fontFamily: 'monospace', paddingLeft: PL, paddingRight: PR, paddingBottom: 4, color: INK_SOFT }}>
            <span>{activeModeName ?? '—'}</span>
            <span style={{ color: '#a23232', fontWeight: 700 }}>
              {isDriving && totalDurS > 0 ? `${fmt(elapsed)} / ${fmt(totalDurS)}  (${Math.round(progressFrac * 100)}%)` : '—'}
            </span>
            <span>残 {isDriving && totalDurS > 0 ? fmt(Math.max(0, totalDurS - elapsed)) : '—'}</span>
          </div>
        </Box>
      )}

      {/* Bottom: BigSpeed + profile | session info + buttons */}
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 10 }}>
        <Box style={{ display: 'flex', gap: 12, alignItems: 'center', padding: 12 }}>
          <BigSpeed value={rd.actual_speed_kmh} refSpeed={rd.ref_speed_kmh ?? null} size={110} />
          <Box label="プロファイル / モード" style={{ flex: 1, padding: '8px 12px' }}>
            <div style={{ fontSize: 15, fontWeight: 700 }}>{activeProfileName ?? '—'}</div>
            <div style={{ fontSize: 12, color: INK_SOFT, marginTop: 4 }}>{activeModeName ?? '—'}</div>
            <div style={{ fontSize: 12, color: INK_SOFT, marginTop: 2 }}>アクセル {rd.accel_opening.toFixed(1)}% / ブレーキ {rd.brake_opening.toFixed(1)}%</div>
          </Box>
        </Box>
        <div style={{ display: 'flex', gap: 10, alignItems: 'stretch' }}>
          <Box style={{ padding: '10px 14px', fontSize: 13, flex: 1 }}>
            {/* T6: 経過時間は走行中のみ表示 */}
            <Row cells={[['経過時間', '1.4fr'], [isDriving ? fmt(elapsed) : '—', '1fr', 'mono']]} />
            {showModeAxis && <Row cells={[['走行モード', '1.4fr'], [activeModeName ?? '—', '1fr']]} />}
            {showModeAxis && <Row cells={[['全体時間', '1.4fr'], [totalDurS > 0 ? fmt(totalDurS) : '—', '1fr', 'mono']]} />}
          </Box>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 8, flex: 1 }}>
            {showPause && (
              <Btn big style={{ flex: 1 }} disabled={robotState !== 'RUNNING'} onClick={handlePause}>
                {paused ? '▶ 再開' : '⏸ 一時停止'}
              </Btn>
            )}
            {robotState === 'RUNNING' ? (
              <Btn danger big style={{ flex: 1 }} onClick={() => setConfirmStop(true)}>■ 走行終了</Btn>
            ) : (
              <Btn big style={{ flex: 1, borderColor: '#3f6b3f', background: '#dfeadc', color: '#22421f' }} onClick={handleStart}>▶ 走行開始</Btn>
            )}
          </div>
        </div>
      </div>

      {confirmStop && React.createElement(window.ConfirmStopPopup, {
        message: '走行を終了しますか？',
        onYes: handleStop,
        onNo: () => setConfirmStop(false),
      })}
    </div>
  );
}

function AutoDriveScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeProfileId, activeModeId, setNav } = useContext(window.AppContext);
  const { ValidationPopup } = window;

  const [popup, setPopup] = useState(null);

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
    no_mode:    { message: '走行モードを選択してください',           actionLabel: '走行モードへ',         nav: 'modes' },
  };

  const cfg = popup ? POPUP_CONFIG[popup] : null;

  return (
    <div style={{ position: 'relative', flex: 1, minHeight: 0, display: 'flex', flexDirection: 'column' }}>
      <DriveMonitorScreen showPause={true} screenTitle="自動走行モニター"
        driveStartPath="/api/v1/drive/start"
        driveStartBody={activeModeId ? { mode_id: activeModeId } : null} />
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
