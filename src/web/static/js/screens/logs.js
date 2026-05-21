// ── Logs screen ───────────────────────────────────────────

function LogsScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch } = useContext(window.AppContext);
  const { INK, PAPER, PAPER_2, HATCH, Box, Btn, H2, Note } = window;

  const [sessions, setSessions] = useState([]);
  const [selectedId, setSelectedId] = useState(null);
  const [logs, setLogs] = useState(null);
  const [loadingLogs, setLoadingLogs] = useState(false);

  useEffect(() => { loadSessions(); }, []);

  async function loadSessions() {
    const data = await apiFetch('GET', '/api/v1/sessions/');
    if (data) setSessions(data);
  }

  async function handleSelect(id) {
    if (selectedId === id) { setSelectedId(null); setLogs(null); return; }
    setSelectedId(id);
    setLoadingLogs(true);
    const d = await apiFetch('GET', `/api/v1/sessions/${id}/logs`);
    setLoadingLogs(false);
    if (d) setLogs(d);
  }

  const RUN_TYPE_LABEL = {
    auto: '自動走行',
    learning: '学習運転',
    manual: '手動運転',
  };

  function formatDuration(startedAt, endedAt) {
    if (!endedAt) return '実行中';
    const diff = Math.round((new Date(endedAt) - new Date(startedAt)) / 1000);
    return `${Math.floor(diff / 60)} 分 ${diff % 60} 秒`;
  }

  return React.createElement('div', { style: { padding: 32 } },
    React.createElement('div', { style: { display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 24 } },
      React.createElement(H2, null, 'ログ'),
      React.createElement(Btn, { variant: 'ghost', onClick: loadSessions }, '更新'),
    ),

    sessions.length === 0
      ? React.createElement(Note, null, 'セッションログがありません。')
      : React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 10 } },
          ...sessions.map(s => {
            const isSelected = s.id === selectedId;
            return React.createElement('div', { key: s.id },
              React.createElement(Box, {
                style: {
                  padding: '14px 20px',
                  borderColor: isSelected ? INK : HATCH,
                  borderWidth: isSelected ? 2.5 : 1.5,
                  cursor: 'pointer',
                },
                onClick: () => handleSelect(s.id),
              },
                React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 12 } },
                  React.createElement('div', { style: { flex: 1 } },
                    React.createElement('div', { style: { fontWeight: 700, fontSize: 14, marginBottom: 2 } },
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
                    ? React.createElement(LogChart, { logs })
                    : React.createElement('div', { style: { fontSize: 13, color: '#888' } }, 'ログデータなし'),
              ),
            );
          }),
        ),
  );
}

function LogChart({ logs }) {
  const { INK, HATCH } = window;
  const W = 700, H = 140, PL = 44, PR = 16, PB = 20, PT = 8;
  const PW = W - PL - PR, PH = H - PB - PT;

  const maxSpeed = Math.max(...logs.map(l => Math.max(l.ref_speed_kmh ?? 0, l.actual_speed_kmh ?? 0)), 1);
  const n = logs.length;
  const toX = i => PL + (i / Math.max(1, n - 1)) * PW;
  const toY = v => PT + PH - (Math.max(0, v) / maxSpeed) * PH;

  const refPath  = logs.map((l, i) => `${i === 0 ? 'M' : 'L'}${toX(i).toFixed(1)},${toY(l.ref_speed_kmh ?? 0).toFixed(1)}`).join(' ');
  const actPath  = logs.map((l, i) => `${i === 0 ? 'M' : 'L'}${toX(i).toFixed(1)},${toY(l.actual_speed_kmh).toFixed(1)}`).join(' ');

  const maxSpeedActual = Math.max(...logs.map(l => l.actual_speed_kmh ?? 0)).toFixed(1);

  return React.createElement('div', null,
    React.createElement('div', { style: { fontSize: 12, color: '#666', marginBottom: 6 } },
      '速度ログ　',
      React.createElement('span', { style: { color: '#888' } }, '— 基準  '),
      React.createElement('span', { style: { color: '#a23232' } }, '— 実車速'),
    ),
    React.createElement('svg', { width: W, height: H },
      // Grid lines
      ...[0, 0.5, 1].map(f =>
        React.createElement('g', { key: f },
          React.createElement('line', {
            x1: PL, y1: PT + PH - f * PH, x2: PL + PW, y2: PT + PH - f * PH,
            stroke: HATCH, strokeWidth: 1,
          }),
          React.createElement('text', {
            x: PL - 4, y: PT + PH - f * PH + 4, textAnchor: 'end', fontSize: 10, fill: '#666',
          }, `${(f * maxSpeed).toFixed(0)}`),
        )
      ),
      React.createElement('path', { d: refPath, fill: 'none', stroke: '#888', strokeWidth: 1.5 }),
      React.createElement('path', { d: actPath, fill: 'none', stroke: '#a23232', strokeWidth: 2 }),
    ),
    React.createElement('div', { style: { marginTop: 10, display: 'flex', gap: 32, fontSize: 13 } },
      React.createElement('span', null, `最高車速: ${maxSpeedActual} km/h`),
      React.createElement('span', null, `サンプル数: ${logs.length}`),
    ),
  );
}

window.LogsScreen = LogsScreen;
