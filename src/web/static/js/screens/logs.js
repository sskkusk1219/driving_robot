// ── Logs screen（段11 UI 見直し）─────────────────────────────
// 左: セッション一覧（cycle_id があるものは学習サイクルの下に入れる。一覧の中だけスクロール）
// 右: 選んだセッションの詳細。タブ（走行グラフ／運転指標・偏差／ログ表／メッセージ）で 1 画面に収める。
//   走行グラフは 3 段（車速／ペダル開度＋ボタン押下／偏差。x 軸共有）。運転指標（EER・IWR・ASCR）と偏差の KPI はモード走行のセッションだけ（GET /api/v1/sessions/{id}/analysis）。
//   合否・基準の線は出さない。サイクルを選ぶと、配下の全セッションをつないだ 1 本の図・まとめを出す。

const RUN_TYPE_LABEL = {
  auto: '自動運転',
  learning: '学習運転',
  manual: '手動運転',
  tuning: 'PID適合走行',
};

const SESSION_STATUS_LABEL = {
  running: '実行中',
  completed: '完了',
  error: 'エラー',
  emergency: '非常停止',
  aborted: '中断',
};

const CYCLE_STATUS_LABEL = SESSION_STATUS_LABEL;

function statusColor(status) {
  if (status === 'emergency' || status === 'error') return '#e06666';
  if (status === 'running') return '#c8922a';
  return '#666';
}

const COL_REF = '#888', COL_ACT = '#c8922a', COL_ACC = '#6ec6f0', COL_BRK = '#e06666';

function formatDuration(startedAt, endedAt) {
  if (!endedAt) return '実行中';
  const diff = Math.round((new Date(endedAt) - new Date(startedAt)) / 1000);
  return `${Math.floor(diff / 60)} 分 ${diff % 60} 秒`;
}

function sessionTitle(s, modeNames) {
  const modeName = s.mode_id && modeNames ? modeNames[s.mode_id] : null;
  const type = RUN_TYPE_LABEL[s.run_type] ?? s.run_type;
  // 車の名前（vehicle_name）。旧プロファイル時代の行（vehicle_id なし）は出さない
  const vehicle = s.vehicle_name ?? s.vehicle_id ?? null;
  return `${vehicle ? `${vehicle}　` : ''}${type}${modeName ? `　${modeName}` : ''}　${new Date(s.started_at).toLocaleString('ja-JP')}`;
}

function LogsScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch } = useContext(window.AppContext);
  const { Note, INK, PAPER_2, HATCH } = window;

  const [sessions, setSessions] = useState([]);
  const [cycles, setCycles] = useState([]);
  const [modeNames, setModeNames] = useState({});
  const [selected, setSelected] = useState(null);   // { kind: 'session' | 'cycle', id }
  const [expandedCycleId, setExpandedCycleId] = useState(null);

  useEffect(() => { loadSessions(); }, []);

  async function loadSessions() {
    const [sessData, cycleData, modeData] = await Promise.all([
      apiFetch('GET', '/api/v1/sessions/'),
      apiFetch('GET', '/api/v1/sessions/cycles'),
      apiFetch('GET', '/api/v1/modes/'),
    ]);
    if (sessData) setSessions(sessData);
    if (cycleData) setCycles(cycleData);
    if (Array.isArray(modeData)) {
      const m = {};
      for (const md of modeData) m[md.id] = md.name;
      setModeNames(m);
    }
  }

  // cycle_id を持つセッションはサイクル配下へ、持たないセッションはフラットのまま。
  const membersByCycle = {};
  const flatSessions = [];
  for (const s of sessions) {
    if (s.cycle_id) (membersByCycle[s.cycle_id] ??= []).push(s);
    else flatSessions.push(s);
  }
  for (const members of Object.values(membersByCycle)) {
    members.sort((a, b) => new Date(a.started_at) - new Date(b.started_at));
  }
  const items = [
    ...flatSessions.map(s => ({ kind: 'session', startedAt: s.started_at, session: s })),
    ...cycles.map(c => ({ kind: 'cycle', startedAt: c.started_at, cycle: c, members: membersByCycle[c.id] || [] })),
  ].sort((a, b) => new Date(b.startedAt) - new Date(a.startedAt));

  const isSel = (kind, id) => selected?.kind === kind && selected?.id === id;
  const rowStyle = (on, nested) => ({
    padding: nested ? '6px 10px 6px 22px' : '8px 12px', borderRadius: 4, cursor: 'pointer',
    background: on ? 'rgba(200,146,42,0.14)' : (nested ? 'transparent' : PAPER_2),
    border: `${on ? 2 : 1}px solid ${on ? INK : HATCH}`,
  });
  const sessionRow = (s, nested) => (
    <div key={s.id} onClick={() => setSelected({ kind: 'session', id: s.id })} style={rowStyle(isSel('session', s.id), nested)}>
      <div style={{ fontWeight: 700, fontSize: 13, lineHeight: 1.35 }}>{sessionTitle(s, modeNames)}</div>
      <div style={{ fontSize: 11, color: '#888' }}>
        <span style={{ color: statusColor(s.status), fontWeight: statusColor(s.status) === '#666' ? 400 : 700 }}>{SESSION_STATUS_LABEL[s.status] ?? s.status}</span>
        {`　${formatDuration(s.started_at, s.ended_at)}`}
      </div>
    </div>
  );

  const selSession = selected?.kind === 'session' ? sessions.find(s => s.id === selected.id) : null;
  const selCycle = selected?.kind === 'cycle' ? cycles.find(c => c.id === selected.id) : null;

  return (
    <div style={{ display: 'flex', gap: 12, flex: 1, minHeight: 0 }}>
      <div style={{ flex: '0 0 clamp(240px, 26%, 340px)', minHeight: 0, display: 'flex', flexDirection: 'column' }}>
        {items.length === 0
          ? <Note>セッションログがありません。</Note>
          : (
            <div style={{ display: 'flex', flexDirection: 'column', gap: 6, overflowY: 'auto', flex: 1, minHeight: 0 }}>
              {items.map(item => item.kind === 'session' ? sessionRow(item.session, false) : (
                <div key={item.cycle.id} style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
                  <div onClick={() => {
                    setSelected({ kind: 'cycle', id: item.cycle.id });
                    setExpandedCycleId(prev => prev === item.cycle.id && isSel('cycle', item.cycle.id) ? null : item.cycle.id);
                  }} style={rowStyle(isSel('cycle', item.cycle.id), false)}>
                    <div style={{ fontWeight: 700, fontSize: 13 }}>
                      <span style={{ display: 'inline-block', marginRight: 6, padding: '0 6px', border: `1px solid ${INK}`, borderRadius: 3, fontSize: 11 }}>学習サイクル</span>
                      {new Date(item.cycle.started_at).toLocaleString('ja-JP')}
                    </div>
                    <div style={{ fontSize: 11, color: '#888' }}>
                      <span style={{ color: statusColor(item.cycle.status), fontWeight: statusColor(item.cycle.status) === '#666' ? 400 : 700 }}>{CYCLE_STATUS_LABEL[item.cycle.status] ?? item.cycle.status}</span>
                      {`　${formatDuration(item.cycle.started_at, item.cycle.ended_at)}　${item.cycle.session_count} セッション`}
                    </div>
                  </div>
                  {expandedCycleId === item.cycle.id && item.members.map(m => sessionRow(m, true))}
                </div>
              ))}
            </div>
          )}
      </div>
      <div style={{ flex: 1, minWidth: 0, minHeight: 0, display: 'flex', flexDirection: 'column',
                    border: `1px solid ${HATCH}`, borderRadius: 4, background: PAPER_2, padding: '8px 12px' }}>
        {selSession
          ? <SessionDetail key={selSession.id} session={selSession} modeNames={modeNames} />
          : selCycle
            ? <CycleDetail key={selCycle.id} cycle={selCycle} members={membersByCycle[selCycle.id] || []} />
            : <div style={{ fontSize: 13, color: '#888' }}>左の一覧からセッションを選んでください。</div>}
      </div>
    </div>
  );
}

// タブ（走行グラフ／運転指標・偏差／ログ表／メッセージ）
function TabBar({ tabs, value, onChange }) {
  const { INK, HATCH } = window;
  return (
    <div style={{ display: 'flex', gap: 6, flexShrink: 0 }}>
      {tabs.map(([k, label]) => (
        <div key={k} onClick={() => onChange(k)} style={{
          padding: '4px 14px', fontSize: 13, cursor: 'pointer', borderRadius: '4px 4px 0 0',
          borderTop: `1px solid ${value === k ? INK : HATCH}`, borderLeft: `1px solid ${value === k ? INK : HATCH}`,
          borderRight: `1px solid ${value === k ? INK : HATCH}`, borderBottom: 'none',
          background: value === k ? 'rgba(200,146,42,0.14)' : 'transparent', fontWeight: value === k ? 700 : 400,
        }}>{label}</div>
      ))}
    </div>
  );
}

function SessionDetail({ session, modeNames }) {
  const { useState, useEffect } = React;
  const { apiFetch } = useContext_AppContext();
  const { HATCH } = window;
  const [logs, setLogs] = useState(null);       // null = 読み込み中
  const [analysis, setAnalysis] = useState(null);
  const [tab, setTab] = useState('chart');
  const isAuto = session.run_type === 'auto';

  useEffect(() => {
    let alive = true;
    apiFetch('GET', `/api/v1/sessions/${session.id}/logs`).then(d => { if (alive) setLogs(d ?? []); });
    apiFetch('GET', `/api/v1/sessions/${session.id}/analysis`).then(d => { if (alive) setAnalysis(d ?? { available: false }); });
    return () => { alive = false; };
  }, [session.id]);

  const tabs = [['chart', '走行グラフ'], ...(isAuto ? [['analysis', '運転指標・偏差']] : []), ['table', 'ログ表'], ['messages', 'メッセージ']];
  return (
    <div style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0, gap: 6 }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 12, flexShrink: 0 }}>
        <div style={{ flex: 1, minWidth: 0, fontWeight: 700, fontSize: 14, whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' }}>{sessionTitle(session, modeNames)}</div>
        <CsvDownloadButton sessionId={session.id} />
      </div>
      <TabBar tabs={tabs} value={tab} onChange={setTab} />
      <div style={{ flex: 1, minHeight: 0, borderTop: `1px solid ${HATCH}`, paddingTop: 8, display: 'flex', flexDirection: 'column' }}>
        {tab === 'messages'
          ? <SessionMessages sessionId={session.id} />
          : logs === null
            ? <div style={{ fontSize: 13, color: '#888' }}>読み込み中…</div>
            : logs.length === 0
              ? <div style={{ fontSize: 13, color: '#888' }}>ログデータなし（走行ログの行がないセッションです）</div>
              : tab === 'chart'
                ? <div style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0, gap: 6 }}>
                    <div style={{ flexShrink: 0 }}><LogSummary groups={[{ label: '', logs }]} /></div>
                    <div style={{ flex: 1, minHeight: 0 }}><SessionFigure analysis={analysis} /></div>
                  </div>
                : tab === 'analysis'
                  ? <AnalysisPanel analysis={analysis} />
                  : <LogTable logs={logs} />}
      </div>
    </div>
  );
}

function useContext_AppContext() { return React.useContext(window.AppContext); }

function CycleDetail({ cycle, members }) {
  const { useState, useEffect } = React;
  const { apiFetch } = useContext_AppContext();
  const [groups, setGroups] = useState(null); // null=読み込み中
  const memberKey = members.map(m => m.id).join(',');
  useEffect(() => {
    let cancelled = false;
    setGroups(null);
    (async () => {
      const all = await Promise.all(members.map(m => apiFetch('GET', `/api/v1/sessions/${m.id}/logs`)));
      if (cancelled) return;
      const counts = {};
      for (const m of members) counts[m.run_type] = (counts[m.run_type] || 0) + 1;
      const seen = {};
      const gs = [];
      members.forEach((m, i) => {
        seen[m.run_type] = (seen[m.run_type] || 0) + 1;
        const base = RUN_TYPE_LABEL[m.run_type] ?? m.run_type;
        const label = counts[m.run_type] > 1 ? `${base} ${seen[m.run_type]}` : base;
        const l = all[i];
        if (Array.isArray(l) && l.length > 0) gs.push({ label, logs: l });
      });
      setGroups(gs);
    })();
    return () => { cancelled = true; };
  }, [memberKey]);
  return (
    <div style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0, gap: 6 }}>
      <div style={{ fontWeight: 700, fontSize: 14, flexShrink: 0 }}>学習サイクル　{new Date(cycle.started_at).toLocaleString('ja-JP')}</div>
      {groups === null
        ? <div style={{ fontSize: 13, color: '#888' }}>読み込み中…</div>
        : groups.length === 0
          ? <div style={{ fontSize: 13, color: '#888' }}>ログデータなし</div>
          : <>
              <div style={{ flexShrink: 0 }}><LogSummary groups={groups} /></div>
              <div style={{ flex: 1, minHeight: 0 }}><LogChart groups={groups} /></div>
            </>}
    </div>
  );
}

// 運転指標（EER・IWR・ASCR）と偏差（KPI の行・区間別の表）。偏差の図は「走行グラフ」タブの 3 段目。合否・基準の線は出さない。
function AnalysisPanel({ analysis }) {
  const { INK, INK_SOFT, HATCH, PAPER_2 } = window;
  if (analysis == null) return <div style={{ fontSize: 13, color: '#888' }}>読み込み中…</div>;
  if (!analysis.available) return <div style={{ fontSize: 13, color: '#888' }}>モード走行のログがないため、運転指標・偏差は出せません。</div>;
  const k = analysis.kpi;
  const f = (v, d = 2) => (v == null ? '—' : Number(v).toFixed(d));
  const sg = (v, d = 2) => (v == null ? '—' : `${v >= 0 ? '+' : ''}${Number(v).toFixed(d)}`);
  const th = { textAlign: 'right', padding: '3px 8px', fontSize: 11, color: INK_SOFT, fontWeight: 600, borderBottom: `1.5px solid ${HATCH}`, whiteSpace: 'nowrap' };
  const td = { textAlign: 'right', padding: '3px 8px', fontSize: 12, fontVariantNumeric: 'tabular-nums', whiteSpace: 'nowrap' };
  const Item = (label, value) => (
    <div key={label}>
      <div style={{ fontSize: 11, color: '#888' }}>{label}</div>
      <div style={{ fontSize: 15, fontWeight: 700 }}>{value}</div>
    </div>
  );
  return (
    <div style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0, gap: 8 }}>
      <div style={{ display: 'flex', gap: 28, flexWrap: 'wrap', flexShrink: 0 }}>
        {Item('最大|偏差|', `${f(k.max)} km/h`)}
        {Item('p95', `${f(k.p95)} km/h`)}
        {Item('|偏差|>0.5', `${f(k.over_0_5_s, 1)} s`)}
        {Item('|偏差|>1', `${f(k.over_1_s, 1)} s`)}
        {Item('平均偏差', `${sg(k.mean)} km/h`)}
        {Item('走行時間', `${f(k.duration_s, 0)} s`)}
      </div>
      <div style={{ display: 'flex', gap: 16, flexShrink: 0, alignItems: 'flex-start', overflow: 'auto', }}>
        <table style={{ borderCollapse: 'collapse' }}>
          <thead><tr>
            <th style={{ ...th, textAlign: 'left' }}>運転指標</th><th style={th}>EER [%]</th><th style={th}>IWR [%]</th><th style={th}>ASCR [%]</th>
          </tr></thead>
          <tbody>
            {analysis.drive_index.rows.map((r, i) => (
              <tr key={r.label} style={{ background: i % 2 ? PAPER_2 : 'transparent' }}>
                <td style={{ ...td, textAlign: 'left' }}>{r.label}</td>
                <td style={td}>{sg(r.eer_pct)}</td><td style={td}>{sg(r.iwr_pct)}</td><td style={td}>{sg(r.ascr_pct)}</td>
              </tr>
            ))}
          </tbody>
        </table>
        {analysis.sections.length > 0 && (
          <table style={{ borderCollapse: 'collapse' }}>
            <thead><tr>
              <th style={{ ...th, textAlign: 'left' }}>区間</th><th style={th}>時間 [s]</th><th style={th}>p95</th><th style={th}>最大</th><th style={th}>平均偏差</th><th style={th}>1.0 超え [s]</th>
            </tr></thead>
            <tbody>
              {analysis.sections.map((r, i) => (
                <tr key={r.name} style={{ background: i % 2 ? PAPER_2 : 'transparent' }}>
                  <td style={{ ...td, textAlign: 'left' }}>{r.name}</td>
                  <td style={td}>{f(r.duration_s, 0)}</td><td style={td}>{f(r.p95)}</td><td style={td}>{f(r.max)}</td>
                  <td style={td}>{sg(r.mean)}</td><td style={td}>{f(r.over_1_s, 1)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </div>
    </div>
  );
}

// ── グラフ ───────────────────────────────────────────────
// 描画は SVG（0..1000 × 0..100 の正規化座標を preserveAspectRatio="none" で伸ばす）、
// 目盛り・凡例の文字は HTML を重ねる（伸ばしても文字が潰れない。DriveMonitorScreen と同じ）。

const CHART_BUCKETS = 1200;

// 点列 [{t, v}]（v=null は線を切る印）を、点が多いときは x ピクセル相当の箱ごとに
// 最小・最大だけ残して間引く（時間順を保つ）。
function downsample(points, tMax) {
  if (points.length <= CHART_BUCKETS * 2) return points;
  const out = [];
  let bucket = -1, mn = null, mx = null;
  const flush = () => {
    if (mn == null) return;
    if (mn === mx) out.push(mn);
    else if (mn.t <= mx.t) out.push(mn, mx);
    else out.push(mx, mn);
    mn = mx = null;
  };
  for (const p of points) {
    if (p.v == null) { flush(); bucket = -1; out.push(p); continue; }
    const b = Math.min(CHART_BUCKETS - 1, Math.floor((p.t / tMax) * CHART_BUCKETS));
    if (b !== bucket) { flush(); bucket = b; }
    if (mn == null) { mn = mx = p; }
    else { if (p.v < mn.v) mn = p; if (p.v > mx.v) mx = p; }
  }
  flush();
  return out;
}

function niceStep(span, maxTicks = 8) {
  const steps = [1, 2, 5, 10, 20, 30, 60, 120, 300, 600, 1200, 1800, 3600];
  for (const s of steps) if (span / s <= maxTicks) return s;
  return 7200;
}

// groups: [{label, logs}]（時刻順）。セッションの切れ目では線を切る。
function buildChartData(groups) {
  const t0 = new Date(groups[0].logs[0].timestamp).getTime();
  const series = { ref: [], act: [], acc: [], brk: [] };
  const marks = [];
  let tMax = 0, hasRef = false, dataMax = 0;
  groups.forEach((g, gi) => {
    if (gi > 0) for (const k in series) series[k].push({ t: tMax, v: null });
    marks.push({ t: (new Date(g.logs[0].timestamp).getTime() - t0) / 1000, label: g.label });
    for (const l of g.logs) {
      const t = (new Date(l.timestamp).getTime() - t0) / 1000;
      tMax = Math.max(tMax, t);
      if (l.ref_speed_kmh != null) hasRef = true;
      series.ref.push({ t, v: l.ref_speed_kmh ?? null });
      series.act.push({ t, v: l.actual_speed_kmh ?? null });
      series.acc.push({ t, v: l.accel_opening ?? null });
      series.brk.push({ t, v: l.brake_opening ?? null });
      dataMax = Math.max(dataMax, l.ref_speed_kmh ?? 0, l.actual_speed_kmh ?? 0);
    }
  });
  tMax = Math.max(1, tMax);
  return { series, marks, tMax, hasRef, speedMax: Math.max(20, Math.ceil(dataMax / 20) * 20) };
}

function seriesPath(points, tMax, vMax, vMin = 0) {
  const pts = downsample(points, tMax);
  let d = '', pen = false;
  for (const p of pts) {
    if (p.v == null) { pen = false; continue; }
    const x = (p.t / tMax) * 1000;
    const y = 100 - ((Math.min(Math.max(p.v, vMin), vMax) - vMin) / (vMax - vMin)) * 100;
    d += `${pen ? 'L' : 'M'}${x.toFixed(2)},${y.toFixed(2)} `;
    pen = true;
  }
  return d;
}

function ChartPanel({ height, yTicks, yLabel, vMax, vMin = 0, lines, tMax, xTicks, marks, showMarkLabels, legend, showXAxis, xLabel = '経過時間 [s]', overlay = [], legendRow = false, yFmt = v => v, note = null, yFor = null }) {
  const { INK, INK_SOFT, HATCH, PAPER_2 } = window;
  const AX = 44, RGT = 12;
  const yOf = v => 100 - ((v - vMin) / (vMax - vMin)) * 100;
  return React.createElement('div', { style: { position: 'relative', marginLeft: AX, marginRight: RGT, height, marginBottom: showXAxis ? 36 : 10 } },
    React.createElement('svg', {
      viewBox: '0 0 1000 100', preserveAspectRatio: 'none',
      style: { position: 'absolute', inset: 0, width: '100%', height: '100%', border: `1px solid ${INK}`, boxSizing: 'border-box', background: 'transparent' },
    },
      ...yTicks.map(v => React.createElement('line', {
        key: 'y' + v, x1: 0, x2: 1000, y1: yOf(v), y2: yOf(v),
        stroke: HATCH, strokeWidth: 1, strokeDasharray: '3 4', vectorEffect: 'non-scaling-stroke',
      })),
      ...xTicks.map(t => React.createElement('line', {
        key: 'x' + t, x1: (t / tMax) * 1000, x2: (t / tMax) * 1000, y1: 0, y2: 100,
        stroke: HATCH, strokeWidth: 1, strokeDasharray: '3 4', vectorEffect: 'non-scaling-stroke',
      })),
      ...marks.map((m, i) => React.createElement('line', {
        key: 'm' + i, x1: (m.t / tMax) * 1000, x2: (m.t / tMax) * 1000, y1: 0, y2: 100,
        stroke: INK_SOFT, strokeWidth: 1.2, strokeDasharray: '1 4', vectorEffect: 'non-scaling-stroke',
      })),
      ...overlay,
      ...lines.map((ln, i) => React.createElement('path', {
        key: 'p' + i, d: ln.d, fill: 'none', stroke: ln.color, strokeWidth: ln.width,
        strokeDasharray: ln.dash, vectorEffect: 'non-scaling-stroke', strokeLinejoin: 'round',
      })),
    ),
    // Y 軸目盛り（HTML）
    ...yTicks.map(v => React.createElement('div', {
      key: 'yl' + v,
      style: { position: 'absolute', right: '100%', paddingRight: 5, top: `${yOf(v)}%`, transform: 'translateY(-50%)', fontSize: 11, color: INK, lineHeight: 1 },
    }, yFmt(v))),
    React.createElement('div', {
      style: { position: 'absolute', left: -AX, top: 0, bottom: 0, width: 14, display: 'flex', alignItems: 'center', justifyContent: 'center' },
    }, React.createElement('span', { style: { transform: 'rotate(-90deg)', whiteSpace: 'nowrap', fontSize: 11, color: INK_SOFT } }, yLabel)),
    // セッション名（点線の右上）
    showMarkLabels && marks.map((m, i) => React.createElement('div', {
      key: 'ml' + i,
      style: { position: 'absolute', left: `${(m.t / tMax) * 100}%`, top: 2, marginLeft: 3, fontSize: 10, whiteSpace: 'nowrap', background: PAPER_2, color: INK, padding: '0 3px', borderRadius: 2 },
    }, m.label)),
    // 凡例
    legend.length > 0 && React.createElement('div', {
      style: { position: 'absolute', right: 6, top: showMarkLabels ? 18 : 4, display: 'flex', flexDirection: legendRow ? 'row' : 'column', gap: legendRow ? 12 : 2, fontSize: 11, background: PAPER_2, color: INK, padding: '2px 6px', border: `1px solid ${HATCH}`, borderRadius: 3 },
    }, legend.map(lg => React.createElement('div', { key: lg.label, style: { display: 'flex', alignItems: 'center', gap: 6 } },
      lg.rect
        ? React.createElement('span', { style: { display: 'inline-block', width: 14, height: 9, background: lg.color + '66', border: `1.2px solid ${lg.color}`, boxSizing: 'border-box' } })
        : React.createElement('span', { style: { display: 'inline-block', width: 18, height: 0, borderTop: `2px ${lg.dash ? 'dashed' : 'solid'} ${lg.color}` } }),
      lg.label,
    ))),
    // X 軸目盛り（最下段のみ）
    showXAxis && xTicks.map(t => React.createElement('div', {
      key: 'xl' + t,
      style: { position: 'absolute', top: '100%', left: `${(t / tMax) * 100}%`, transform: 'translateX(-50%)', marginTop: 4, fontSize: 11, color: INK, lineHeight: 1 },
    }, t)),
    showXAxis && React.createElement('div', {
      style: { position: 'absolute', top: '100%', left: 0, right: 0, textAlign: 'center', marginTop: 17, fontSize: 11, color: INK_SOFT, lineHeight: 1 },
    }, xLabel),
    note && React.createElement('div', { style: { position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center', fontSize: 12, color: INK_SOFT, pointerEvents: 'none' } }, note),
  );
}

function LogChart({ groups }) {
  const { tMax, series, marks, hasRef, speedMax } = buildChartData(groups);
  const step = niceStep(tMax);
  const xTicks = [];
  for (let t = 0; t <= tMax + 1e-9; t += step) xTicks.push(t);
  const ySpeedTicks = [];
  for (let v = 0; v <= speedMax; v += speedMax / 4) ySpeedTicks.push(Math.round(v));
  const shownMarks = groups.length > 1 ? marks : [];

  const speedLines = [];
  const speedLegend = [];
  if (hasRef) {
    speedLines.push({ d: seriesPath(series.ref, tMax, speedMax), color: COL_REF, width: 1.5, dash: '6 4' });
    speedLegend.push({ label: '基準車速', color: COL_REF, dash: true });
  }
  speedLines.push({ d: seriesPath(series.act, tMax, speedMax), color: COL_ACT, width: 2 });
  speedLegend.push({ label: '実車速', color: COL_ACT });

  const common = { tMax, xTicks, marks: shownMarks };
  return React.createElement('div', { style: { width: '100%', height: '100%', display: 'flex', flexDirection: 'column' } },
    React.createElement('div', { style: { flex: '1.2 1 0', minHeight: 0 } },
      React.createElement(ChartPanel, {
        ...common, height: 'calc(100% - 10px)', yTicks: ySpeedTicks, yLabel: '車速 [km/h]', vMax: speedMax,
        lines: speedLines, legend: speedLegend, showMarkLabels: shownMarks.length > 0, showXAxis: false,
      })),
    React.createElement('div', { style: { flex: '1 1 0', minHeight: 0 } },
      React.createElement(ChartPanel, {
        ...common, height: 'calc(100% - 36px)', yTicks: [0, 20, 40, 60, 80, 100], yLabel: '開度 [%]', vMax: 100,
        lines: [
          { d: seriesPath(series.acc, tMax, 100), color: COL_ACC, width: 1.8 },
          { d: seriesPath(series.brk, tMax, 100), color: COL_BRK, width: 1.8 },
        ],
        legend: [{ label: 'アクセル', color: COL_ACC }, { label: 'ブレーキ', color: COL_BRK }],
        showMarkLabels: false, showXAxis: true,
      })),
  );
}

// 走行グラフ（試験の走行レポートと同じ 3 段。x 軸共有・高さ 1 : 1 : 0.5）。
//   車速（基準＝破線・実車速）／ ペダル開度（アクセル・ブレーキ＋ボタン押下の矩形。押していない 0・押している 50）／ 偏差（実車速 − 基準。基準の線なし）。
// figure は GET /sessions/{id}/analysis の figure（x はモード走行ならモード時間・学習などは経過時間）。
function SessionFigure({ analysis }) {
  // ボタンの色・名前・押している高さは自動運転画面（auto-drive.js）の定数をそのまま使う
  const { INK_SOFT, BUTTON_COLORS, BUTTON_LABELS, BUTTON_PRESSED_LEVEL } = window;
  if (analysis == null) return <div style={{ fontSize: 13, color: '#888' }}>読み込み中…</div>;
  const figure = analysis.figure;
  if (!figure) return <div style={{ fontSize: 13, color: '#888' }}>図にできるログの行がありません。</div>;
  const n = figure.t.length;
  const tMax = Math.max(1, figure.t[n - 1]);
  const pts = arr => figure.t.map((t, i) => ({ t, v: arr[i] }));
  const hasRef = figure.ref != null;
  let vMaxData = 0;
  for (let i = 0; i < n; i++) vMaxData = Math.max(vMaxData, figure.act[i] ?? 0, hasRef ? (figure.ref[i] ?? 0) : 0);
  const speedMax = Math.max(20, Math.ceil(vMaxData / 20) * 20);
  const ySpeedTicks = [];
  for (let v = 0; v <= speedMax; v += speedMax / 4) ySpeedTicks.push(Math.round(v));
  const step = niceStep(tMax);
  const xTicks = [];
  for (let t = 0; t <= tMax + 1e-9; t += step) xTicks.push(t);

  const speedLines = [];
  const speedLegend = [];
  if (hasRef) {
    speedLines.push({ d: seriesPath(pts(figure.ref), tMax, speedMax), color: COL_REF, width: 1.5, dash: '6 4' });
    speedLegend.push({ label: '基準車速', color: COL_REF, dash: true });
  }
  speedLines.push({ d: seriesPath(pts(figure.act), tMax, speedMax), color: COL_ACT, width: 2 });
  speedLegend.push({ label: '実車速', color: COL_ACT });

  // 偏差（基準のあるときだけ）。対称な縦軸 ±m
  const dev = hasRef ? figure.act.map((a, i) => (figure.ref[i] == null ? null : a - figure.ref[i])) : [];
  let m = 0.5;
  for (const v of dev) if (v != null) m = Math.max(m, Math.abs(v));
  m = Math.ceil(m * 2) / 2;
  const devTicks = [-m, 0, m];

  // ボタンの矩形（立ち上がり → 押している間は横に伸びる → 立ち下がり）。図の範囲に切り詰める
  const X = t => (t / tMax) * 1000;
  const Y = v => 100 - v;
  const rects = (figure.buttons ?? []).filter(b => b.end_s >= 0 && b.start_s <= tMax).map((b, i) => {
    const xs = X(b.start_s), xe = X(b.end_s);
    const color = BUTTON_COLORS[b.button] ?? INK_SOFT;
    return React.createElement('path', {
      key: `btn${i}`, d: `M${xs.toFixed(2)},${Y(0)} L${xs.toFixed(2)},${Y(BUTTON_PRESSED_LEVEL)} L${xe.toFixed(2)},${Y(BUTTON_PRESSED_LEVEL)} L${xe.toFixed(2)},${Y(0)} Z`,
      fill: color, fillOpacity: 0.28, stroke: color, strokeWidth: 1.5, vectorEffect: 'non-scaling-stroke', strokeLinejoin: 'round',
    });
  });
  const pedalLegend = [
    { label: 'アクセル', color: COL_ACC }, { label: 'ブレーキ', color: COL_BRK },
    ...Object.keys(BUTTON_COLORS).map(k => ({ label: BUTTON_LABELS[k], color: BUTTON_COLORS[k], rect: true })),
  ];

  const common = { tMax, xTicks, marks: [], showMarkLabels: false };
  return React.createElement('div', { style: { width: '100%', height: '100%', display: 'flex', flexDirection: 'column' } },
    React.createElement('div', { style: { flex: '1 1 0', minHeight: 0 } },
      React.createElement(ChartPanel, {
        ...common, height: 'calc(100% - 10px)', yTicks: ySpeedTicks, yLabel: '車速 [km/h]', vMax: speedMax,
        lines: speedLines, legend: speedLegend, showXAxis: false,
      })),
    React.createElement('div', { style: { flex: '1 1 0', minHeight: 0 } },
      React.createElement(ChartPanel, {
        ...common, height: 'calc(100% - 10px)', yTicks: [0, 20, 40, 60, 80, 100], yLabel: '開度 [%]', vMax: 100,
        overlay: rects, legendRow: true,
        lines: [
          { d: seriesPath(pts(figure.accel), tMax, 100), color: COL_ACC, width: 1.8 },
          { d: seriesPath(pts(figure.brake), tMax, 100), color: COL_BRK, width: 1.8 },
        ],
        legend: pedalLegend, showXAxis: false,
      })),
    React.createElement('div', { style: { flex: '0.5 1 0', minHeight: 0 } },
      React.createElement(ChartPanel, {
        ...common, height: 'calc(100% - 36px)', yTicks: devTicks, yLabel: '偏差 [km/h]', vMax: m, vMin: -m,
        yFmt: v => (v > 0 ? `+${v}` : `${v}`),
        lines: hasRef ? [{ d: seriesPath(pts(dev), tMax, m, -m), color: '#5a8fd6', width: 1.4 }] : [],
        legend: [], note: hasRef ? null : '基準車速がないため偏差はありません',
        showXAxis: true, xLabel: figure.x_label,
      })),
  );
}

// CSV ダウンロードボタン（ブラウザの添付ダウンロードをそのまま利用）
function CsvDownloadButton({ sessionId }) {
  const { INK, HATCH, PAPER_2 } = window;
  return React.createElement('a', {
    href: `/api/v1/sessions/${sessionId}/logs.csv`,
    download: '',
    style: {
      display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
      border: `1.5px solid ${HATCH}`, borderRadius: 5,
      padding: '6px 14px', background: PAPER_2, color: INK,
      fontSize: 14, fontWeight: 500, textDecoration: 'none',
      cursor: 'pointer', whiteSpace: 'nowrap', flexShrink: 0,
    },
  }, 'CSVダウンロード（全列）');
}

// まとめ（合否は出さない）: 最高車速・最大逸脱（時刻つき）・偏差 p95・記録時間・サンプル数。
// 逸脱は基準車速のある行の |実車速 − 基準|。基準が無ければ「—（基準車速なし）」。
function LogSummary({ groups }) {
  let maxSpeed = 0, n = 0, durSec = 0;
  const devs = [];
  let maxDev = null, maxDevT = 0;
  const t0 = new Date(groups[0].logs[0].timestamp).getTime();
  for (const g of groups) {
    const first = g.logs[0], last = g.logs[g.logs.length - 1];
    durSec += (new Date(last.timestamp) - new Date(first.timestamp)) / 1000;
    n += g.logs.length;
    for (const l of g.logs) {
      maxSpeed = Math.max(maxSpeed, l.actual_speed_kmh ?? 0);
      if (l.ref_speed_kmh == null) continue;
      const d = Math.abs(l.actual_speed_kmh - l.ref_speed_kmh);
      devs.push(d);
      if (maxDev == null || d > maxDev) { maxDev = d; maxDevT = (new Date(l.timestamp).getTime() - t0) / 1000; }
    }
  }
  let p95 = null;
  if (devs.length) {
    devs.sort((a, b) => a - b);
    p95 = devs[Math.min(devs.length - 1, Math.ceil(devs.length * 0.95) - 1)];
  }
  const Item = (label, value, note, color) => React.createElement('div', null,
    React.createElement('div', { style: { fontSize: 11, color: '#888' } }, label,
      note && React.createElement('span', { style: { marginLeft: 6, fontWeight: 400 } }, note)),
    React.createElement('div', { style: { fontSize: 15, fontWeight: 700, color } }, ...[].concat(value)),
  );
  const noRef = '—（基準車速なし）';

  return React.createElement('div', { style: { display: 'flex', gap: 36, flexWrap: 'wrap' } },
    Item('最高車速', `${maxSpeed.toFixed(1)} km/h`),
    Item('最大逸脱', maxDev == null ? noRef : `${maxDev.toFixed(2)} km/h（t=${maxDevT.toFixed(1)}s）`),
    Item('偏差 p95', p95 == null ? noRef : `${p95.toFixed(2)} km/h`),
    Item('記録時間', `${durSec.toFixed(1)} 秒`),
    Item('サンプル数', `${n}`),
  );
}

// 表は先頭の行だけ描く（WLTP 全体は約 3 万行で、ブラウザが重くなるため）。全列・全行は CSV で。
const LOG_TABLE_MAX_ROWS = 1000;

function LogTable({ logs: allLogs }) {
  const { HATCH, PAPER_2, INK_SOFT } = window;
  const logs = allLogs.length > LOG_TABLE_MAX_ROWS ? allLogs.slice(0, LOG_TABLE_MAX_ROWS) : allLogs;
  const cols = [
    ['時刻', l => new Date(l.timestamp).toLocaleTimeString('ja-JP', { hour12: false }) + '.' + String(new Date(l.timestamp).getMilliseconds()).padStart(3, '0')],
    ['基準[km/h]', l => l.ref_speed_kmh == null ? '—' : l.ref_speed_kmh.toFixed(2)],
    ['実車速[km/h]', l => l.actual_speed_kmh.toFixed(2)],
    ['Acc開度[%]', l => l.accel_opening.toFixed(1)],
    ['Brk開度[%]', l => l.brake_opening.toFixed(1)],
    ['Acc電流[mA]', l => l.accel_current.toFixed(0)],
    ['Brk電流[mA]', l => l.brake_current.toFixed(0)],
  ];

  const th = { textAlign: 'right', padding: '4px 10px', fontSize: 11, color: '#888', fontWeight: 600, position: 'sticky', top: 0, background: PAPER_2, borderBottom: `1.5px solid ${HATCH}` };
  const td = { textAlign: 'right', padding: '3px 10px', fontSize: 12, fontVariantNumeric: 'tabular-nums', whiteSpace: 'nowrap' };

  return React.createElement('div', { style: { display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 } },
    allLogs.length > logs.length && React.createElement('div', { style: { fontSize: 12, color: INK_SOFT, marginBottom: 4 } },
      `先頭 ${logs.length} 行を表示（全 ${allLogs.length} 行。全列・全行は「CSVダウンロード（全列）」で取れます）`),
    React.createElement('div', { style: { flex: 1, minHeight: 0, overflow: 'auto', border: `1.5px solid ${HATCH}` } },
    React.createElement('table', { style: { borderCollapse: 'collapse', width: '100%' } },
      React.createElement('thead', null,
        React.createElement('tr', null, ...cols.map(([label]) => React.createElement('th', { key: label, style: th }, label))),
      ),
      React.createElement('tbody', null,
        ...logs.map((l, i) => React.createElement('tr', {
          key: l.id ?? i,
          style: { background: i % 2 ? PAPER_2 : 'transparent' },
        }, ...cols.map(([label, fn]) => React.createElement('td', { key: label, style: td }, fn(l))))),
      ),
    ),
    ),
  );
}

// 進行のメッセージ（GET /api/v1/sessions/{id}/messages）。等幅・経過秒つき。WARNING は黄・ERROR は赤。
function SessionMessages({ sessionId }) {
  const { useState, useEffect } = React;
  const { INK, INK_SOFT, INK_MUTE, HATCH, PAPER } = window;
  const [msgs, setMsgs] = useState(null);   // null = 読み込み中
  useEffect(() => {
    let alive = true;
    setMsgs(null);
    fetch(`/api/v1/sessions/${sessionId}/messages`)
      .then(r => (r.ok ? r.json() : []))
      .then(j => { if (alive) setMsgs(Array.isArray(j) ? j : []); })
      .catch(() => { if (alive) setMsgs([]); });
    return () => { alive = false; };
  }, [sessionId]);
  const colorOf = lv => (lv === 'ERROR' || lv === 'CRITICAL' ? '#f07070' : lv === 'WARNING' ? '#f0c84a' : INK);
  return React.createElement('div', { style: { display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 } },
    React.createElement('div', { style: { fontSize: 13, fontWeight: 700, marginBottom: 4 } },
      '進行のメッセージ', msgs ? React.createElement('span', { style: { fontWeight: 400, color: INK_SOFT, marginLeft: 8 } }, `${msgs.length} 行`) : null),
    React.createElement('div', {
      style: { flex: 1, minHeight: 0, overflowY: 'auto', border: `1px solid ${HATCH}`, borderRadius: 3, padding: '6px 10px', background: PAPER,
               fontFamily: 'ui-monospace, Menlo, Consolas, monospace', fontSize: 12, lineHeight: 1.45 },
    },
      msgs === null
        ? React.createElement('div', { style: { color: INK_MUTE } }, '読み込み中…')
        : msgs.length === 0
          ? React.createElement('div', { style: { color: INK_MUTE } }, 'メッセージはありません')
          : msgs.map(m => React.createElement('div', { key: m.seq, style: { whiteSpace: 'pre-wrap', wordBreak: 'break-all', color: colorOf(m.level) } },
              `[${Number(m.elapsed_s).toFixed(1).padStart(6)}s] ${m.message}`)),
    ),
  );
}

window.LogsScreen = LogsScreen;
