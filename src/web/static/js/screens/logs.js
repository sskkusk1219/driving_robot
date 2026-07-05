// ── Logs screen ───────────────────────────────────────────
// セッション一覧は cycle_id でグループ表示する（1学習サイクル=1折りたたみ項目:
// 学習運転1件+PID適合走行N件）。cycle_id が無いセッション（manual・通常auto）は
// 従来どおりフラット表示する。

const RUN_TYPE_LABEL = {
  auto: '自動走行',
  learning: '学習運転',
  manual: '手動運転',
  tuning: 'PID適合走行',
};

const CYCLE_STATUS_LABEL = {
  running: '実行中',
  completed: '完了',
  error: 'エラー',
  aborted: '中断',
};

function formatDuration(startedAt, endedAt) {
  if (!endedAt) return '実行中';
  const diff = Math.round((new Date(endedAt) - new Date(startedAt)) / 1000);
  return `${Math.floor(diff / 60)} 分 ${diff % 60} 秒`;
}

function LogsScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch } = useContext(window.AppContext);
  const { Note } = window;

  const [sessions, setSessions] = useState([]);
  const [cycles, setCycles] = useState([]);
  const [selectedId, setSelectedId] = useState(null);
  const [logs, setLogs] = useState(null);
  const [loadingLogs, setLoadingLogs] = useState(false);
  const [expandedCycleId, setExpandedCycleId] = useState(null);

  useEffect(() => { loadSessions(); }, []);

  async function loadSessions() {
    const [sessData, cycleData] = await Promise.all([
      apiFetch('GET', '/api/v1/sessions/'),
      apiFetch('GET', '/api/v1/sessions/cycles'),
    ]);
    if (sessData) setSessions(sessData);
    if (cycleData) setCycles(cycleData);
  }

  async function handleSelect(id) {
    if (selectedId === id) { setSelectedId(null); setLogs(null); return; }
    setSelectedId(id);
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

  const sessionRowProps = { selectedId, onSelect: handleSelect, loadingLogs, logs };

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
                ...sessionRowProps,
              })
          ),
        ),
  );
}

// 1件のセッション行 + 展開時のログ詳細パネル。フラット表示・サイクル配下表示の両方で使う。
function SessionRow({ s, selectedId, onSelect, loadingLogs, logs, nested = false }) {
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
            `${RUN_TYPE_LABEL[s.run_type] ?? s.run_type}　${new Date(s.started_at).toLocaleString('ja-JP')}`
          ),
          React.createElement('div', { style: { fontSize: 12, color: '#666' } },
            `状態: ${s.status}　時間: ${formatDuration(s.started_at, s.ended_at)}`
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

// 学習サイクル1件=1折りたたみ項目。展開すると配下セッション（学習1+適合N）を表示する。
function CycleGroup({ cycle, members, expanded, onToggle, selectedId, onSelect, loadingLogs, logs }) {
  const { INK, PAPER_2, HATCH } = window;
  const statusLabel = CYCLE_STATUS_LABEL[cycle.status] ?? cycle.status;
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
            `状態: ${statusLabel}　時間: ${formatDuration(cycle.started_at, cycle.ended_at)}　セッション数: ${cycle.session_count}`
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
        : members.map(s => React.createElement(SessionRow, {
            key: s.id, s, selectedId, onSelect, loadingLogs, logs, nested: true,
          })),
    ),
  );
}

function LogChart({ logs }) {
  const { INK, HATCH } = window;
  const W = 700, H = 140, PL = 44, PR = 16, PB = 20, PT = 8;
  const PW = W - PL - PR, PH = H - PB - PT;

  // 自動運転ページと同じく、最高値を 20 の倍数（20,40,…,160）へ切り上げる。
  const dataMax = Math.max(...logs.map(l => Math.max(l.ref_speed_kmh ?? 0, l.actual_speed_kmh ?? 0)), 0);
  const maxSpeed = Math.max(20, Math.ceil(dataMax / 20) * 20);
  const n = logs.length;
  // x 軸はサンプル番号ではなく実経過時間ベース。ログ欠落（バス再送によるサイクル
  // スキップ）区間が時間的に圧縮されず、実走行時間どおりの横軸で描画する。
  const t0 = new Date(logs[0].timestamp).getTime();
  const tSpan = Math.max(1, new Date(logs[n - 1].timestamp).getTime() - t0);
  const toX = i => PL + ((new Date(logs[i].timestamp).getTime() - t0) / tSpan) * PW;
  const toY = v => PT + PH - (Math.max(0, v) / maxSpeed) * PH;

  const refPath  = logs.map((l, i) => `${i === 0 ? 'M' : 'L'}${toX(i).toFixed(1)},${toY(l.ref_speed_kmh ?? 0).toFixed(1)}`).join(' ');
  const actPath  = logs.map((l, i) => `${i === 0 ? 'M' : 'L'}${toX(i).toFixed(1)},${toY(l.actual_speed_kmh).toFixed(1)}`).join(' ');

  return React.createElement('div', null,
    React.createElement('div', { style: { fontSize: 12, color: '#666', marginBottom: 6 } },
      '速度ログ　',
      React.createElement('span', { style: { color: '#888' } }, '— 基準  '),
      React.createElement('span', { style: { color: '#a23232' } }, '— 実車速'),
    ),
    React.createElement('svg', {
      viewBox: `0 0 ${W} ${H}`,
      preserveAspectRatio: 'none',
      width: '100%',
      style: { display: 'block', width: '100%', height: 'auto', maxWidth: W },
    },
      // Grid lines（自動運転ページと同じく 4 等分 = 5 本）
      ...[0, 0.25, 0.5, 0.75, 1].map((f, i) =>
        React.createElement('g', { key: f },
          React.createElement('line', {
            x1: PL, y1: PT + PH - f * PH, x2: PL + PW, y2: PT + PH - f * PH,
            stroke: HATCH, strokeWidth: 1, strokeDasharray: '3 4',
          }),
          React.createElement('text', {
            x: PL - 4, y: PT + PH - f * PH + (i === 0 ? -3 : 4), textAnchor: 'end', fontSize: 11, fill: INK,
          }, `${Math.round(f * maxSpeed)}`),
        )
      ),
      React.createElement('path', { d: refPath, fill: 'none', stroke: '#888', strokeWidth: 1.5 }),
      React.createElement('path', { d: actPath, fill: 'none', stroke: '#a23232', strokeWidth: 2 }),
    ),
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
  return React.createElement('div', null,
    React.createElement('div', { style: { display: 'flex', justifyContent: 'flex-end', marginBottom: 12 } },
      React.createElement(CsvDownloadButton, { sessionId }),
    ),
    React.createElement(LogSummary, { logs }),
    React.createElement('div', { style: { marginTop: 16 } }, React.createElement(LogChart, { logs })),
    React.createElement('div', { style: { marginTop: 16 } }, React.createElement(LogTable, { logs })),
  );
}

function LogSummary({ logs }) {
  const maxSpeed = Math.max(...logs.map(l => l.actual_speed_kmh ?? 0));
  const deviations = logs
    .filter(l => l.ref_speed_kmh != null)
    .map(l => Math.abs(l.actual_speed_kmh - l.ref_speed_kmh));
  const avgDev = deviations.length
    ? deviations.reduce((a, b) => a + b, 0) / deviations.length
    : null;
  // 実タイムスタンプ（末尾 - 先頭）から記録時間を求める。サンプル数×100ms の概算は
  // バス再送によるサイクルスキップでログが欠落すると実走行時間より大幅に短く出る。
  const first = logs[0], last = logs[logs.length - 1];
  const durSec = (new Date(last.timestamp) - new Date(first.timestamp)) / 1000;

  const Item = (label, value) => React.createElement('div', null,
    React.createElement('div', { style: { fontSize: 11, color: '#888' } }, label),
    React.createElement('div', { style: { fontSize: 15, fontWeight: 700 } }, value),
  );

  return React.createElement('div', { style: { display: 'flex', gap: 36, flexWrap: 'wrap' } },
    Item('最高車速', `${maxSpeed.toFixed(1)} km/h`),
    Item('平均逸脱', avgDev == null ? '—' : `${avgDev.toFixed(2)} km/h`),
    Item('記録時間', `${durSec.toFixed(1)} 秒`),
    Item('サンプル数', `${logs.length}`),
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
