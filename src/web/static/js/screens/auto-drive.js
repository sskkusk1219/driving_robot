// ── Auto-drive monitor screen ─────────────────────────────
// AutoDriveD layout: 3-axis graph + BigSpeed + session info + pause/stop

function DriveMonitorScreen({ showPause = true, screenTitle = '自動走行モニター' }) {
  const { useState, useEffect, useContext, useRef } = React;
  const { apiFetch, realtimeData, realtimeBuf, activeModeId, activeModeName, activeProfileName } = useContext(window.AppContext);
  const { INK, INK_SOFT, PAPER, PAPER_2, HATCH, Box, Btn, Row, BigSpeed } = window;

  const [modeDetail, setModeDetail] = useState(null);
  const [elapsed, setElapsed] = useState(0);
  const [paused, setPaused] = useState(false);
  const [confirmStop, setConfirmStop] = useState(false);
  const timerRef = useRef(null);
  const sessionStartRef = useRef(Date.now());

  useEffect(() => {
    timerRef.current = setInterval(() => setElapsed(s => s + 1), 1000);
    return () => clearInterval(timerRef.current);
  }, []);

  useEffect(() => {
    if (activeModeId) {
      apiFetch('GET', `/api/v1/modes/${activeModeId}`).then(d => { if (d) setModeDetail(d); });
    }
  }, [activeModeId]);

  async function handleStop() {
    const r = await apiFetch('POST', '/api/v1/drive/stop');
    if (r) { window.showToast('走行を終了しました', 'success'); setConfirmStop(false); }
  }

  async function handlePause() {
    const endpoint = paused ? '/api/v1/drive/resume' : '/api/v1/drive/pause';
    const r = await apiFetch('POST', endpoint);
    if (r) { setPaused(p => !p); window.showToast(paused ? '走行を再開しました' : '一時停止しました', 'success'); }
  }

  const buf = realtimeBuf.current;
  const WINDOW = 780; // ~13 min at 100ms
  const recent = buf.slice(-WINDOW);

  // Graph constants — same as wireframe (VW=900, PL=50, PR=16)
  const VW = 900, PL = 50, PR = 16, PH1 = 140, PH2 = 100, PH3 = 100;
  const PW = VW - PL - PR;

  function toX(i, len) { return PL + (i / Math.max(1, len - 1)) * PW; }
  function toXFull(frac) { return PL + frac * PW; }

  function polyline(pts, color, strokeW = 2, dasharray) {
    if (pts.length < 2) return null;
    const d = pts.map((p, i) => `${i === 0 ? 'M' : 'L'}${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ');
    return <path d={d} fill="none" stroke={color} strokeWidth={strokeW} strokeDasharray={dasharray} />;
  }

  // Axis 1: speed
  const maxSpeed = 130;
  const speedRef_pts = recent.map((d, i) => ({
    x: toX(i, recent.length),
    y: PH1 - (Math.min(d.ref_speed_kmh, maxSpeed) / maxSpeed) * (PH1 - 12),
  }));
  const speedAct_pts = recent.map((d, i) => ({
    x: toX(i, recent.length),
    y: PH1 - (Math.min(d.actual_speed_kmh, maxSpeed) / maxSpeed) * (PH1 - 12),
  }));

  // Axis 2: openings
  const accelPts = recent.map((d, i) => ({
    x: toX(i, recent.length),
    y: PH2 - (d.accel_opening / 100) * (PH2 - 8),
  }));
  const brakePts = recent.map((d, i) => ({
    x: toX(i, recent.length),
    y: PH2 - (d.brake_opening / 100) * (PH2 - 8),
  }));

  // Axis 3: full mode profile overview
  const refProfile = modeDetail?.reference_speed ?? [];
  const totalDurS = refProfile.reduce((s, p) => s + (p.duration_s ?? 0), 0);
  const profilePts = (() => {
    if (refProfile.length === 0) return [];
    let t = 0, pts = [];
    for (const seg of refProfile) {
      const x = toXFull(t / totalDurS);
      const y = PH3 - (Math.min(seg.speed, maxSpeed) / maxSpeed) * (PH3 - 8);
      pts.push({ x, y });
      t += seg.duration_s;
      pts.push({ x: toXFull(t / totalDurS), y });
    }
    return pts;
  })();
  const progressFrac = totalDurS > 0 ? Math.min(1, elapsed / totalDurS) : 0;
  const markerX = toXFull(progressFrac);

  const fmt = s => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;

  function GraphSvg({ height, children, yMax, unit }) {
    const ticks = [0, 0.25, 0.5, 0.75, 1].map(f => ({ v: Math.round(f * yMax), y: height - f * (height - 8) }));
    return (
      <svg viewBox={`0 0 ${VW} ${height}`} width="100%" height={height} preserveAspectRatio="none" style={{ display: 'block' }}>
        {ticks.map(({ v, y }) => (
          <g key={v}>
            <line x1={PL} y1={y} x2={PL + PW} y2={y} stroke={HATCH} strokeWidth="1" strokeDasharray="3 4" />
            <text x={PL - 4} y={y + 4} textAnchor="end" fontSize="10" fill="#666">{v}{unit}</text>
          </g>
        ))}
        {children}
      </svg>
    );
  }

  const rd = realtimeData;

  return (
    <div style={{ position: 'relative', display: 'flex', flexDirection: 'column', gap: 10, flex: 1, minHeight: 0 }}>

      {/* Axis 1: speed graph */}
      <Box label={`1軸目 · 基準車速 vs 実車速 [km/h]  —  直近 ${Math.round(recent.length * 0.1 / 60)} 分ウィンドウ`}
           style={{ padding: '6px 0 2px' }}>
        <GraphSvg height={PH1} yMax={maxSpeed} unit="">
          {polyline(speedRef_pts, '#888', 1.5, '4 3')}
          {polyline(speedAct_pts, '#2f5780', 2.2)}
        </GraphSvg>
        <div style={{ display: 'flex', gap: 14, fontSize: 12, color: INK_SOFT, paddingLeft: PL, paddingBottom: 4 }}>
          <span>実車速 <b style={{ fontFamily: 'monospace', color: INK }}>{rd.actual_speed_kmh.toFixed(1)}</b> km/h</span>
          <span>基準 <b style={{ fontFamily: 'monospace', color: INK }}>{rd.ref_speed_kmh?.toFixed(1) ?? '—'}</b></span>
          <span>偏差 <b style={{ fontFamily: 'monospace', color: INK }}>
            {rd.ref_speed_kmh != null ? (rd.actual_speed_kmh - rd.ref_speed_kmh).toFixed(2) : '—'}
          </b></span>
        </div>
      </Box>

      {/* Axis 2: openings graph */}
      <Box label="2軸目 · アクセル / ブレーキ開度 [%]  —  直近ウィンドウ" style={{ padding: '6px 0 2px' }}>
        <GraphSvg height={PH2} yMax={100} unit="%">
          {polyline(accelPts, INK, 2.2)}
          {polyline(brakePts, '#a23232', 2.2)}
        </GraphSvg>
        <div style={{ display: 'flex', gap: 14, fontSize: 12, color: INK_SOFT, paddingLeft: PL, paddingBottom: 4 }}>
          <span>アクセル <b style={{ fontFamily: 'monospace', color: INK }}>{rd.accel_opening.toFixed(1)} %</b></span>
          <span>ブレーキ <b style={{ fontFamily: 'monospace', color: INK }}>{rd.brake_opening.toFixed(1)} %</b></span>
        </div>
      </Box>

      {/* Axis 3: full profile overview */}
      <Box label={`3軸目 · 全体プロファイル — 独立x軸 (0 → ${totalDurS > 0 ? `${Math.round(totalDurS / 60)}:00` : '—'}) · 現在位置マーカー`}
           style={{ padding: '6px 0 2px' }}>
        <svg viewBox={`0 0 ${VW} ${PH3}`} width="100%" height={PH3} preserveAspectRatio="none" style={{ display: 'block' }}>
          <line x1={PL} y1={PH3 - 8} x2={PL + PW} y2={PH3 - 8} stroke={HATCH} strokeWidth="0.8" />
          {profilePts.length > 0 && (() => {
            const d = profilePts.map((p, i) => `${i === 0 ? 'M' : 'L'}${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ');
            return <path d={d} fill="none" stroke="#888" strokeWidth="1.5" strokeDasharray="4 3" />;
          })()}
          {totalDurS > 0 && (
            <>
              <line x1={markerX} y1={4} x2={markerX} y2={PH3 - 4} stroke="#a23232" strokeWidth="2" />
              <text x={markerX} y={14} textAnchor="middle" fontSize="13" fill="#a23232">▼</text>
            </>
          )}
        </svg>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 12, fontFamily: 'monospace', paddingLeft: PL, paddingRight: PR, paddingBottom: 4, color: INK_SOFT }}>
          <span>{activeModeName ?? '—'}</span>
          <span style={{ color: '#a23232', fontWeight: 700 }}>
            {totalDurS > 0 ? `${fmt(elapsed)} / ${fmt(totalDurS)}  (${Math.round(progressFrac * 100)}%)` : '—'}
          </span>
          <span>残 {totalDurS > 0 ? fmt(Math.max(0, totalDurS - elapsed)) : '—'}</span>
        </div>
      </Box>

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
            <Row cells={[['経過時間', '1.4fr'], [fmt(elapsed), '1fr', 'mono']]} />
            <Row cells={[['走行モード', '1.4fr'], [activeModeName ?? '—', '1fr']]} />
            <Row cells={[['全体時間', '1.4fr'], [totalDurS > 0 ? fmt(totalDurS) : '—', '1fr', 'mono']]} />
          </Box>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 8, flex: 1 }}>
            {showPause && (
              <Btn big style={{ flex: 1 }} onClick={handlePause}>
                {paused ? '▶ 再開' : '⏸ 一時停止'}
              </Btn>
            )}
            <Btn danger big style={{ flex: 1 }} onClick={() => setConfirmStop(true)}>■ 走行終了</Btn>
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
  return <DriveMonitorScreen showPause={true} screenTitle="自動走行モニター" />;
}

window.DriveMonitorScreen = DriveMonitorScreen;
window.AutoDriveScreen = AutoDriveScreen;
