// ── Logs screen ───────────────────────────────────────────
// セッション一覧は cycle_id でグループ表示する（1学習サイクル=1折りたたみ項目:
// 学習運転1件+PID適合走行N件）。cycle_id が無いセッション（manual・通常auto）は
// 従来どおりフラット表示する。サイクルを開くと配下の全セッションのログを時刻順に
// つないだ 1 本の図・まとめを出す。

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

// KPI 基準（src/domain/control/kpi_monitor.py と同じ値）
const KPI_MAX_DEV = 1.0;
const KPI_P95_DEV = 0.4;

const COL_REF = '#888', COL_ACT = '#c8922a', COL_ACC = '#6ec6f0', COL_BRK = '#e06666';

function formatDuration(startedAt, endedAt) {
  if (!endedAt) return '実行中';
  const diff = Math.round((new Date(endedAt) - new Date(startedAt)) / 1000);
  return `${Math.floor(diff / 60)} 分 ${diff % 60} 秒`;
}

function sessionTitle(s, modeNames) {
  const modeName = s.mode_id && modeNames ? modeNames[s.mode_id] : null;
  const type = RUN_TYPE_LABEL[s.run_type] ?? s.run_type;
  return `${type}${modeName ? `　${modeName}` : ''}　${new Date(s.started_at).toLocaleString('ja-JP')}`;
}

function LogsScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch } = useContext(window.AppContext);
  const { Note } = window;

  const [sessions, setSessions] = useState([]);
  const [cycles, setCycles] = useState([]);
  const [modeNames, setModeNames] = useState({});
  const [selectedId, setSelectedId] = useState(null);
  const [logs, setLogs] = useState(null);
  const [loadingLogs, setLoadingLogs] = useState(false);
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

  async function handleSelect(id) {
    if (selectedId === id) { setSelectedId(null); setLogs(null); return; }
    setSelectedId(id);
    setLogs(null);
    setLoadingLogs(true);
    const d = await apiFetch('GET', `/api/v1/sessions/${id}/logs`);
    setLoadingLogs(false);
    if (d) setLogs(d);
  }

  // cycle_id を持つセッションはサイクル配下へ、持たないセッションはフラットのまま。
  const membersByCycle = {};
  const flatSessions = [];
  for (const s of sessions) {
    if (s.cycle_id) {
      (membersByCycle[s.cycle_id] ??= []).push(s);
    } else {
      flatSessions.push(s);
    }
  }
  for (const members of Object.values(membersByCycle)) {
    members.sort((a, b) => new Date(a.started_at) - new Date(b.started_at));
  }

  const items = [
    ...flatSessions.map(s => ({ kind: 'session', startedAt: s.started_at, session: s })),
    ...cycles.map(c => ({ kind: 'cycle', startedAt: c.started_at, cycle: c, members: membersByCycle[c.id] || [] })),
  ].sort((a, b) => new Date(b.startedAt) - new Date(a.startedAt));

  const sessionRowProps = { selectedId, onSelect: handleSelect, loadingLogs, logs, modeNames };

  return React.createElement('div', { style: { display: 'flex', flexDirection: 'column', height: '100%', padding: 32 } },
    items.length === 0
      ? React.createElement(Note, null, 'セッションログがありません。')
      : React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 10, overflowY: 'auto', flex: 1, minHeight: 0 } },
          ...items.map(item => item.kind === 'session'
            ? React.createElement(SessionRow, { key: item.session.id, s: item.session, ...sessionRowProps })
            : React.createElement(CycleGroup, {
                key: item.cycle.id,
                cycle: item.cycle,
                members: item.members,
                expanded: expandedCycleId === item.cycle.id,
                onToggle: () => setExpandedCycleId(prev => prev === item.cycle.id ? null : item.cycle.id),
                apiFetch,
                ...sessionRowProps,
              })
          ),
        ),
  );
}

// 1件のセッション行 + 展開時のログ詳細パネル。フラット表示・サイクル配下表示の両方で使う。
function SessionRow({ s, selectedId, onSelect, loadingLogs, logs, modeNames, nested = false }) {
  const { INK, PAPER_2, HATCH } = window;
  const isSelected = s.id === selectedId;
  return React.createElement('div', { key: s.id },
    React.createElement('div', {
      style: {
        padding: nested ? '10px 16px' : '14px 20px',
        background: nested ? 'transparent' : PAPER_2,
        borderRadius: 4,
        border: `${isSelected ? 2.5 : 1.5}px solid ${isSelected ? INK : HATCH}`,
        cursor: 'pointer',
      },
      onClick: () => onSelect(s.id),
    },
      React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 12 } },
        React.createElement('div', { style: { flex: 1 } },
          React.createElement('div', { style: { fontWeight: 700, fontSize: nested ? 13 : 14, marginBottom: 2 } },
            sessionTitle(s, modeNames)
          ),
          React.createElement('div', { style: { fontSize: 12, color: '#666' } },
            '状態: ',
            React.createElement('span', { style: { color: statusColor(s.status), fontWeight: statusColor(s.status) === '#666' ? 400 : 700 } },
              SESSION_STATUS_LABEL[s.status] ?? s.status),
            `　時間: ${formatDuration(s.started_at, s.ended_at)}`
          ),
        ),
        React.createElement('span', { style: { fontSize: 18, color: '#888' } }, isSelected ? '▲' : '▼'),
      ),
    ),
    // Log detail panel
    isSelected && React.createElement('div', {
      style: { border: `1.5px solid ${HATCH}`, borderTop: 'none', padding: 20, background: PAPER_2 }
    },
      loadingLogs
        ? React.createElement('div', { style: { fontSize: 13, color: '#888' } }, '読み込み中…')
        : logs && logs.length > 0
          ? React.createElement(LogDetail, { sessionId: s.id, logs })
          : React.createElement('div', { style: { display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 12 } },
              React.createElement('span', { style: { fontSize: 13, color: '#888' } }, 'ログデータなし'),
              React.createElement(CsvDownloadButton, { sessionId: s.id }),
            ),
    ),
  );
}

// 学習サイクル1件=1折りたたみ項目。展開すると配下の全セッションをつないだ図・まとめと、
// 配下セッション（学習1+適合N）の一覧を表示する。
function CycleGroup({ cycle, members, expanded, onToggle, selectedId, onSelect, loadingLogs, logs, modeNames, apiFetch }) {
  const { useState, useEffect } = React;
  const { INK, PAPER_2, HATCH } = window;
  const statusLabel = CYCLE_STATUS_LABEL[cycle.status] ?? cycle.status;
  const [groups, setGroups] = useState(null); // null=読み込み中
  const memberKey = members.map(m => m.id).join(',');

  useEffect(() => {
    if (!expanded) return;
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
  }, [expanded, memberKey]);

  return React.createElement('div', { key: cycle.id },
    React.createElement('div', {
      style: {
        padding: '14px 20px',
        background: PAPER_2,
        borderRadius: 4,
        border: `${expanded ? 2.5 : 1.5}px solid ${expanded ? INK : HATCH}`,
        cursor: 'pointer',
      },
      onClick: onToggle,
    },
      React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 12 } },
        React.createElement('div', { style: { flex: 1 } },
          React.createElement('div', { style: { fontWeight: 700, fontSize: 14, marginBottom: 2 } },
            React.createElement('span', {
              style: {
                display: 'inline-block', marginRight: 8, padding: '0 6px',
                border: `1px solid ${INK}`, borderRadius: 3, fontSize: 11,
              },
            }, '学習サイクル'),
            `${new Date(cycle.started_at).toLocaleString('ja-JP')}`
          ),
          React.createElement('div', { style: { fontSize: 12, color: '#666' } },
            '状態: ',
            React.createElement('span', { style: { color: statusColor(cycle.status), fontWeight: statusColor(cycle.status) === '#666' ? 400 : 700 } }, statusLabel),
            `　時間: ${formatDuration(cycle.started_at, cycle.ended_at)}　セッション数: ${cycle.session_count}`
          ),
        ),
        React.createElement('span', { style: { fontSize: 18, color: '#888' } }, expanded ? '▲' : '▼'),
      ),
    ),
    expanded && React.createElement('div', {
      style: { border: `1.5px solid ${HATCH}`, borderTop: 'none', padding: '10px 12px', display: 'flex', flexDirection: 'column', gap: 8 },
    },
      members.length === 0
        ? React.createElement('div', { style: { fontSize: 13, color: '#888' } }, 'サイクル配下のセッションがありません。')
        : React.createElement(React.Fragment, null,
            React.createElement('div', { style: { padding: '8px 8px 12px' } },
              groups === null
                ? React.createElement('div', { style: { fontSize: 13, color: '#888' } }, '読み込み中…')
                : groups.length === 0
                  ? React.createElement('div', { style: { fontSize: 13, color: '#888' } }, 'ログデータなし')
                  : React.createElement(React.Fragment, null,
                      React.createElement(LogSummary, { groups }),
                      React.createElement('div', { style: { marginTop: 16 } }, React.createElement(LogChart, { groups })),
                    ),
            ),
            members.map(s => React.createElement(SessionRow, {
              key: s.id, s, selectedId, onSelect, loadingLogs, logs, modeNames, nested: true,
            })),
          ),
    ),
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

function seriesPath(points, tMax, vMax) {
  const pts = downsample(points, tMax);
  let d = '', pen = false;
  for (const p of pts) {
    if (p.v == null) { pen = false; continue; }
    const x = (p.t / tMax) * 1000;
    const y = 100 - (Math.min(Math.max(p.v, 0), vMax) / vMax) * 100;
    d += `${pen ? 'L' : 'M'}${x.toFixed(2)},${y.toFixed(2)} `;
    pen = true;
  }
  return d;
}

function ChartPanel({ height, yTicks, yLabel, vMax, lines, tMax, xTicks, marks, showMarkLabels, legend, showXAxis }) {
  const { INK, INK_SOFT, HATCH, PAPER_2 } = window;
  const AX = 44, RGT = 12;
  return React.createElement('div', { style: { position: 'relative', marginLeft: AX, marginRight: RGT, height, marginBottom: showXAxis ? 36 : 10 } },
    React.createElement('svg', {
      viewBox: '0 0 1000 100', preserveAspectRatio: 'none',
      style: { position: 'absolute', inset: 0, width: '100%', height: '100%', border: `1px solid ${INK}`, boxSizing: 'border-box', background: 'transparent' },
    },
      ...yTicks.map(v => React.createElement('line', {
        key: 'y' + v, x1: 0, x2: 1000, y1: 100 - (v / vMax) * 100, y2: 100 - (v / vMax) * 100,
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
      ...lines.map((ln, i) => React.createElement('path', {
        key: 'p' + i, d: ln.d, fill: 'none', stroke: ln.color, strokeWidth: ln.width,
        strokeDasharray: ln.dash, vectorEffect: 'non-scaling-stroke', strokeLinejoin: 'round',
      })),
    ),
    // Y 軸目盛り（HTML）
    ...yTicks.map(v => React.createElement('div', {
      key: 'yl' + v,
      style: { position: 'absolute', right: '100%', paddingRight: 5, top: `${100 - (v / vMax) * 100}%`, transform: 'translateY(-50%)', fontSize: 11, color: INK, lineHeight: 1 },
    }, v)),
    React.createElement('div', {
      style: { position: 'absolute', left: -AX, top: 0, bottom: 0, width: 14, display: 'flex', alignItems: 'center', justifyContent: 'center' },
    }, React.createElement('span', { style: { transform: 'rotate(-90deg)', whiteSpace: 'nowrap', fontSize: 11, color: INK_SOFT } }, yLabel)),
    // セッション名（点線の右上）
    showMarkLabels && marks.map((m, i) => React.createElement('div', {
      key: 'ml' + i,
      style: { position: 'absolute', left: `${(m.t / tMax) * 100}%`, top: 2, marginLeft: 3, fontSize: 10, whiteSpace: 'nowrap', background: PAPER_2, color: INK, padding: '0 3px', borderRadius: 2 },
    }, m.label)),
    // 凡例
    React.createElement('div', {
      style: { position: 'absolute', right: 6, top: showMarkLabels ? 18 : 4, display: 'flex', flexDirection: 'column', gap: 2, fontSize: 11, background: PAPER_2, color: INK, padding: '2px 6px', border: `1px solid ${HATCH}`, borderRadius: 3 },
    }, legend.map(lg => React.createElement('div', { key: lg.label, style: { display: 'flex', alignItems: 'center', gap: 6 } },
      React.createElement('span', { style: { display: 'inline-block', width: 18, height: 0, borderTop: `2px ${lg.dash ? 'dashed' : 'solid'} ${lg.color}` } }),
      lg.label,
    ))),
    // X 軸目盛り（最下段のみ）
    showXAxis && xTicks.map(t => React.createElement('div', {
      key: 'xl' + t,
      style: { position: 'absolute', top: '100%', left: `${(t / tMax) * 100}%`, transform: 'translateX(-50%)', marginTop: 4, fontSize: 11, color: INK, lineHeight: 1 },
    }, t)),
    showXAxis && React.createElement('div', {
      style: { position: 'absolute', top: '100%', left: 0, right: 0, textAlign: 'center', marginTop: 17, fontSize: 11, color: INK_SOFT, lineHeight: 1 },
    }, '経過時間 [s]'),
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
  return React.createElement('div', { style: { width: '100%' } },
    React.createElement(ChartPanel, {
      ...common, height: 190, yTicks: ySpeedTicks, yLabel: '車速 [km/h]', vMax: speedMax,
      lines: speedLines, legend: speedLegend, showMarkLabels: shownMarks.length > 0, showXAxis: false,
    }),
    React.createElement(ChartPanel, {
      ...common, height: 150, yTicks: [0, 20, 40, 60, 80, 100], yLabel: '開度 [%]', vMax: 100,
      lines: [
        { d: seriesPath(series.acc, tMax, 100), color: COL_ACC, width: 1.8 },
        { d: seriesPath(series.brk, tMax, 100), color: COL_BRK, width: 1.8 },
      ],
      legend: [{ label: 'アクセル', color: COL_ACC }, { label: 'ブレーキ', color: COL_BRK }],
      showMarkLabels: false, showXAxis: true,
    }),
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
  }, 'CSVダウンロード');
}

function LogDetail({ sessionId, logs }) {
  const groups = [{ label: '', logs }];
  return React.createElement('div', null,
    React.createElement('div', { style: { display: 'flex', justifyContent: 'flex-end', marginBottom: 12 } },
      React.createElement(CsvDownloadButton, { sessionId }),
    ),
    React.createElement(LogSummary, { groups }),
    React.createElement('div', { style: { marginTop: 16 } }, React.createElement(LogChart, { groups })),
    React.createElement('div', { style: { marginTop: 16 } }, React.createElement(LogTable, { logs })),
  );
}

// まとめ: 最高車速・最大逸脱（時刻つき）・偏差 p95・記録時間・サンプル数。
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
  // 合否は色つきの文字で（Pi のブラウザは絵文字が白黒の記号になり見分けにくい）
  const mark = ok => React.createElement('span', {
    style: { marginLeft: 8, fontSize: 12, padding: '1px 6px', borderRadius: 3,
             color: ok ? '#68d468' : '#e06666', border: `1px solid ${ok ? '#3c8c3c' : '#a23232'}` },
  }, ok ? '合格' : '不合格');
  const Item = (label, value, note, color) => React.createElement('div', null,
    React.createElement('div', { style: { fontSize: 11, color: '#888' } }, label,
      note && React.createElement('span', { style: { marginLeft: 6, fontWeight: 400 } }, note)),
    React.createElement('div', { style: { fontSize: 15, fontWeight: 700, color } }, ...[].concat(value)),
  );
  const noRef = '—（基準車速なし）';

  return React.createElement('div', { style: { display: 'flex', gap: 36, flexWrap: 'wrap' } },
    Item('最高車速', `${maxSpeed.toFixed(1)} km/h`),
    Item('最大逸脱', maxDev == null ? noRef : [`${maxDev.toFixed(2)} km/h（t=${maxDevT.toFixed(1)}s）`, mark(maxDev <= KPI_MAX_DEV)], `基準 ≤${KPI_MAX_DEV.toFixed(1)}`),
    Item('偏差 p95', p95 == null ? noRef : [`${p95.toFixed(2)} km/h`, mark(p95 <= KPI_P95_DEV)], `基準 ≤${KPI_P95_DEV.toFixed(1)}`),
    Item('記録時間', `${durSec.toFixed(1)} 秒`),
    Item('サンプル数', `${n}`),
  );
}

function LogTable({ logs }) {
  const { HATCH, PAPER_2 } = window;
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

  return React.createElement('div', { style: { maxHeight: 320, overflow: 'auto', border: `1.5px solid ${HATCH}` } },
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
  );
}

window.LogsScreen = LogsScreen;
