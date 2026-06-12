// ── Modes screens ─────────────────────────────────────────

// 一覧用ミニグラフ: 軸なし・ラベルなしで速度プロファイルの形だけ表示
function MiniSpeedGraph({ rows }) {
  const W = 200, H = 50;
  const pad = 3;
  const pw = W - pad * 2, ph = H - pad * 2;

  if (!rows || rows.length === 0) {
    return React.createElement('div', {
      style: {
        width: 160, height: 44,
        border: `1.4px dashed ${window.HATCH}`,
        background: window.PAPER_2,
      }
    });
  }

  const totalDur = rows[rows.length - 1].time_s;
  const maxSpeed = Math.max(...rows.map(r => r.speed_kmh), 1);

  const xAt = (t) => pad + (t / totalDur) * pw;
  const yAt = (v) => pad + ph - (v / maxSpeed) * ph;

  // データ量が多い場合は間引いて描画
  const step = Math.max(1, Math.floor(rows.length / 200));
  const sampled = rows.filter((_, i) => i % step === 0 || i === rows.length - 1);
  const pts = sampled.map((r, i) =>
    `${i === 0 ? 'M' : 'L'}${xAt(r.time_s).toFixed(1)},${yAt(r.speed_kmh).toFixed(1)}`
  ).join(' ');

  return React.createElement('svg', {
    viewBox: `0 0 ${W} ${H}`,
    width: 160,
    height: 44,
    style: { display: 'block' },
  },
    React.createElement('rect', {
      x: 0, y: 0, width: W, height: H,
      fill: window.PAPER_2, stroke: window.HATCH, strokeWidth: 1,
    }),
    React.createElement('path', {
      d: pts, fill: 'none', stroke: window.INK, strokeWidth: 2,
    }),
  );
}

// 新規作成画面用の大きなグラフ: 軸・ラベルあり
function CsvSpeedGraph({ rows, width = 600, height = 220 }) {
  const W = 600, H = 220;
  const padL = 40, padR = 14, padT = 14, padB = 36;  // 下余白を増やしてラベル重なりを解消
  const pw = W - padL - padR, ph = H - padT - padB;

  if (!rows || rows.length === 0) {
    return React.createElement('div', {
      style: {
        width: '100%', height, border: `1.4px dashed ${window.HATCH}`,
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        fontSize: 13, color: window.INK_SOFT, background: window.PAPER_2,
      }
    }, 'CSVをアップロードするとプレビューが表示されます');
  }

  const totalDur = rows[rows.length - 1].time_s;
  const maxSpeed = Math.max(...rows.map(r => r.speed_kmh), 1);

  // Y軸最大値をキリの良い値に切り上げ
  const niceYSteps = [10, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 250, 300];
  const yMax = niceYSteps.find(s => s >= maxSpeed) ?? Math.ceil(maxSpeed / 20) * 20;

  const xAt = (t) => padL + (t / totalDur) * pw;
  const yAt = (v) => padT + ph - (v / yMax) * ph;

  const pts = rows.map((r, i) => `${i === 0 ? 'M' : 'L'}${xAt(r.time_s).toFixed(1)},${yAt(r.speed_kmh).toFixed(1)}`).join(' ');

  // X軸目盛り (5〜7本になるよう自動調整)
  const niceXIntervals = [10, 30, 60, 120, 300, 600, 900, 1800];
  const tickInterval = niceXIntervals.find(v => v >= totalDur / 5) ?? niceXIntervals[niceXIntervals.length - 1];
  const ticks = [];
  for (let t = 0; t <= totalDur; t += tickInterval) {
    ticks.push({ x: xAt(t), label: `${Math.round(t)}s` });
  }

  // Y軸目盛り (0, 半分, 最大)
  const yTicks = [0, yMax / 2, yMax];

  return React.createElement('svg', {
    viewBox: `0 0 ${W} ${H}`,
    width, height,
    style: { display: 'block' },
  },
    // 枠
    React.createElement('rect', {
      x: padL, y: padT, width: pw, height: ph,
      fill: 'none', stroke: window.INK, strokeWidth: '1.2',
    }),
    // グリッド線
    ...yTicks.map(v =>
      React.createElement('line', {
        key: v,
        x1: padL, y1: yAt(v), x2: padL + pw, y2: yAt(v),
        stroke: window.HATCH, strokeWidth: 0.8, strokeDasharray: '2 3',
      })
    ),
    // Y軸ラベル
    ...yTicks.map(v =>
      React.createElement('text', {
        key: `yl${v}`,
        x: padL - 5, y: yAt(v) + 4,
        fontSize: 11, textAnchor: 'end', fontFamily: 'inherit', fill: window.INK,
      }, `${v}`)
    ),
    // 速度ライン
    React.createElement('path', { d: pts, fill: 'none', stroke: window.INK, strokeWidth: 2 }),
    // X軸目盛りラベル (プロットエリア下端 +14px)
    ...ticks.map(({ x, label }) =>
      React.createElement('text', {
        key: label,
        x, y: padT + ph + 14,
        textAnchor: 'middle', fontSize: 10, fontFamily: 'inherit', fill: window.INK_SOFT,
      }, label)
    ),
    // X軸タイトル (目盛りの下、さらに +14px)
    React.createElement('text', {
      x: padL + pw / 2, y: padT + ph + 28,
      fontSize: 10, textAnchor: 'middle', fontFamily: 'inherit', fill: window.INK_SOFT,
    }, 'time [s] →'),
    React.createElement('text', {
      x: 6, y: padT - 2,
      fontSize: 10, fontFamily: 'inherit', fill: window.INK_SOFT,
    }, 'km/h'),
  );
}

function ModesScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeModeId, setActiveModeId, activeModeName, setActiveModeName } = useContext(window.AppContext);
  const { INK, INK_SOFT, INK_MUTE, PAPER_2, HATCH, Box, Btn, H2, Note, Pill, Row, Hatch, RowActions } = window;

  const [modes, setModes] = useState([]);
  const [detailMap, setDetailMap] = useState({}); // id -> reference_speed[]
  const [loading, setLoading] = useState(true);
  const [mode, setMode] = useState('list'); // list | create | edit
  const [editTarget, setEditTarget] = useState(null);
  const [search, setSearch] = useState('');
  const [sortKey, setSortKey] = useState(null);   // 'name' | 'total_duration' | 'max_speed'
  const [sortAsc, setSortAsc] = useState(true);
  const [editingNameId, setEditingNameId] = useState(null);
  const [editingNameValue, setEditingNameValue] = useState('');

  useEffect(() => { loadModes(); }, []);

  async function loadModes() {
    setLoading(true);
    const data = await apiFetch('GET', '/api/v1/modes/');
    if (!data) { setLoading(false); return; }
    // 詳細を先に並行取得してから両方まとめて更新 (React 18 自動バッチで1回のレンダリング)
    const details = await Promise.all(data.map(m => apiFetch('GET', `/api/v1/modes/${m.id}`)));
    const map = {};
    details.forEach((d, i) => {
      if (d?.reference_speed) map[data[i].id] = d.reference_speed;
    });
    setModes(data);
    setDetailMap(map);
    setLoading(false);
  }

  function startEditName(m) {
    setEditingNameId(m.id);
    setEditingNameValue(m.name);
  }

  function cancelEditName() {
    setEditingNameId(null);
    setEditingNameValue('');
  }

  async function handleSaveName(m) {
    const trimmed = editingNameValue.trim();
    if (!trimmed) { window.showToast('モード名を入力してください', 'error'); return; }
    if (trimmed === m.name) { cancelEditName(); return; }
    const r = await apiFetch('PATCH', `/api/v1/modes/${m.id}`, { name: trimmed });
    if (r) {
      window.showToast('名前を更新しました', 'success');
      if (m.id === activeModeId) setActiveModeName(trimmed);
      setEditingNameId(null);
      setEditingNameValue('');
      loadModes();
    }
  }

  async function handleCopy(m) {
    const speedRows = detailMap[m.id];
    if (!speedRows) {
      window.showToast('データ読み込み中です。少し待ってから再試行してください', 'error');
      return;
    }
    const csv = 'time_s,speed_kmh\n' + speedRows.map(r => `${r.time_s},${r.speed_kmh}`).join('\n');
    const blob = new Blob([csv], { type: 'text/csv' });
    const file = new File([blob], 'copy.csv', { type: 'text/csv' });
    const fd = new FormData();
    fd.append('file', file);
    fd.append('name', `${m.name} のコピー`);
    fd.append('description', m.description || '');
    const r = await apiFetch('POST', '/api/v1/modes/upload', fd, true);
    if (r) {
      window.showToast(`「${r.name}」を作成しました`, 'success');
      loadModes();
    }
  }

  async function handleSelect(m) {
    setActiveModeId(m.id);
    setActiveModeName(m.name);
    window.showToast(`「${m.name}」を選択しました`, 'success');
  }

  async function handleDelete(m) {
    if (!confirm(`「${m.name}」を削除しますか？`)) return;
    const r = await apiFetch('DELETE', `/api/v1/modes/${m.id}`);
    if (r !== null) {
      window.showToast('走行モードを削除しました', 'success');
      if (activeModeId === m.id) {
        setActiveModeId(null);
        setActiveModeName(null);
      }
      loadModes();
    }
  }

  if (mode === 'create') {
    return React.createElement(ModeCreate, {
      onSave: () => { setMode('list'); loadModes(); },
      onCancel: () => setMode('list'),
    });
  }

  if (mode === 'edit' && editTarget) {
    return React.createElement(ModeEdit, {
      initial: editTarget,
      referenceSpeed: detailMap[editTarget.id] || [],
      onSave: async (name, description, file) => {
        let r;
        if (file) {
          const fd = new FormData();
          fd.append('file', file);
          fd.append('name', name);
          fd.append('description', description);
          r = await apiFetch('PUT', `/api/v1/modes/${editTarget.id}`, fd, true);
        } else {
          r = await apiFetch('PATCH', `/api/v1/modes/${editTarget.id}`, { name, description });
        }
        if (r) {
          window.showToast(`「${r.name}」を更新しました`, 'success');
          if (editTarget.id === activeModeId) setActiveModeName(r.name);
          setMode('list');
          setEditTarget(null);
          loadModes();
        }
      },
      onCancel: () => { setMode('list'); setEditTarget(null); },
      onDelete: async () => {
        if (!confirm(`「${editTarget.name}」を削除しますか？`)) return;
        const r = await apiFetch('DELETE', `/api/v1/modes/${editTarget.id}`);
        if (r !== null) {
          window.showToast('走行モードを削除しました', 'success');
          if (activeModeId === editTarget.id) { setActiveModeId(null); setActiveModeName(null); }
          setMode('list');
          setEditTarget(null);
          loadModes();
        }
      },
    });
  }

  const fmtDuration = (s) => {
    const min = Math.round(s / 60);
    return `${Math.round(s)}s (${min}分)`;
  };

  function handleSortClick(key) {
    if (sortKey === key) {
      setSortAsc(a => !a);
    } else {
      setSortKey(key);
      setSortAsc(true);
    }
  }

  function sortIcon(key) {
    if (sortKey !== key) return ' ↕';
    return sortAsc ? ' ↑' : ' ↓';
  }

  const q = search.trim().toLowerCase();
  const filtered = modes.filter(m =>
    m.name.toLowerCase().includes(q) ||
    (m.description || '').toLowerCase().includes(q)
  );
  const sorted = sortKey
    ? [...filtered].sort((a, b) => {
        const va = a[sortKey], vb = b[sortKey];
        const cmp = typeof va === 'string' ? va.localeCompare(vb, 'ja') : va - vb;
        return sortAsc ? cmp : -cmp;
      })
    : filtered;

  return React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14, height: '100%' } },

    // ── ヘッダ ──────────────────────────────────────────────
    React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 12 } },
      React.createElement(H2, {
        sub: '基準車速CSVをアップロードして管理。自動走行で選択します。',
      }, '走行モード'),
      React.createElement('div', { style: { flex: 1 } }),
      // 検索ボックス
      React.createElement('input', {
        type: 'text',
        value: search,
        onChange: e => setSearch(e.target.value),
        placeholder: '名前・説明で検索…',
        style: {
          padding: '6px 12px', fontSize: 14,
          border: `1.3px solid ${INK}`, borderRadius: 4,
          fontFamily: 'inherit', background: PAPER, outline: 'none',
          width: 200,
        },
      }),
      React.createElement(Btn, { primary: true, big: true, onClick: () => setMode('create') }, '＋ 新規作成'),
    ),

    // ── テーブル ─────────────────────────────────────────────
    React.createElement(Box, { style: { padding: 0 } },
      // ヘッダ行 (ソート可能な列はクリッカブル)
      React.createElement('div', {
        style: {
          display: 'grid',
          gridTemplateColumns: '2fr 1.5fr 1fr 1fr 2fr 2fr',
          borderBottom: `1px solid ${INK}`,
          padding: '10px 14px',
          background: PAPER_2,
          fontSize: 14, fontWeight: 700,
        }
      },
        ...[
          ['名前',     'name'],
          ['説明',     null],
          ['長さ',     'total_duration'],
          ['最高車速', 'max_speed'],
          ['プレビュー', null],
          ['操作',     null],
        ].map(([label, key]) =>
          React.createElement('div', {
            key: label,
            onClick: key ? () => handleSortClick(key) : undefined,
            style: {
              cursor: key ? 'pointer' : 'default',
              userSelect: 'none',
              fontFamily: (key === 'total_duration' || key === 'max_speed') ? 'monospace' : 'inherit',
            },
          }, label + (key ? sortIcon(key) : ''))
        ),
      ),

      loading
        ? React.createElement('div', {
            style: { padding: '20px 14px', color: INK_SOFT, fontSize: 14 }
          }, '読み込み中…')
        : sorted.length === 0
        ? React.createElement('div', {
            style: { padding: '20px 14px', color: INK_SOFT, fontSize: 14 }
          }, modes.length === 0
              ? '走行モードがありません。CSV ファイルをアップロードして作成してください。'
              : `「${search}」に一致するモードがありません。`)
        : sorted.map((m) => {
            const isActive = m.id === activeModeId;
            return React.createElement(Row, {
              key: m.id,
              cells: [
                // 名前 + 選択中バッジ / インライン編集
                [editingNameId === m.id
                  ? React.createElement('div', { key: 'ne', style: { display: 'flex', gap: 4, alignItems: 'center' } },
                      React.createElement('input', {
                        autoFocus: true,
                        value: editingNameValue,
                        onChange: e => setEditingNameValue(e.target.value),
                        onKeyDown: e => {
                          if (e.key === 'Enter') handleSaveName(m);
                          if (e.key === 'Escape') cancelEditName();
                        },
                        style: {
                          fontSize: 14, padding: '2px 6px',
                          border: `1.3px solid ${INK}`, borderRadius: 3,
                          fontFamily: 'inherit', width: '100%',
                        },
                      }),
                      React.createElement('span', {
                        onClick: () => handleSaveName(m),
                        title: '保存',
                        style: { cursor: 'pointer', fontSize: 16, userSelect: 'none' },
                      }, '✓'),
                      React.createElement('span', {
                        onClick: cancelEditName,
                        title: 'キャンセル',
                        style: { cursor: 'pointer', fontSize: 16, userSelect: 'none' },
                      }, '✗'),
                    )
                  : React.createElement('div', { key: 'n', style: { display: 'flex', alignItems: 'center', gap: 6 } },
                      React.createElement('b', null, m.name),
                      React.createElement('span', {
                        onClick: () => startEditName(m),
                        title: '名前を変更',
                        style: { cursor: 'pointer', opacity: 0.5, fontSize: 13, userSelect: 'none' },
                      }, '✎'),
                    ),
                  '2fr'],
                // 説明
                [m.description || React.createElement('span', { key: 'd', style: { color: INK_MUTE } }, '—'), '1.5fr'],
                // 長さ
                [fmtDuration(m.total_duration), '1fr', 'mono'],
                // 最高車速
                [`${m.max_speed.toFixed(1)} km/h`, '1fr', 'mono'],
                // プレビュー (実データグラフ、ロード中は Hatch)
                [detailMap[m.id]
                  ? React.createElement(MiniSpeedGraph, { key: 'g', rows: detailMap[m.id] })
                  : React.createElement(Hatch, { key: 'g', width: 160, height: 44 }),
                '2fr'],
                // 操作
                [React.createElement(RowActions, {
                  key: 'b',
                  isActive,
                  onSelect: () => handleSelect(m),
                  onEdit:   () => { setEditTarget(m); setMode('edit'); },
                  onCopy:   () => handleCopy(m),
                  onDelete: () => handleDelete(m),
                }), '2fr'],
              ],
              style: {
                padding: '10px 14px',
                background: isActive ? '#201e16' : 'transparent',
                borderBottom: `1px dashed ${HATCH}`,
              },
            });
          }),
    ),
  );
}

// ── ModeEdit — 新規作成と同じ UI で名前・説明・CSV差替えを編集 ──
function ModeEdit({ initial, referenceSpeed, onSave, onCancel, onDelete }) {
  const { useState } = React;
  const { Box, Btn, H2, Input, Note } = window;
  const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH } = window;

  const [name, setName] = useState(initial.name);
  const [desc, setDesc] = useState(initial.description || '');
  const [file, setFile] = useState(null);
  // 新CSVがあればそちら、なければ既存データをプレビューに使う
  const [csvRows, setCsvRows] = useState(referenceSpeed || []);
  const [csvSample, setCsvSample] = useState([]);
  const [validations, setValidations] = useState([]);
  const [saving, setSaving] = useState(false);

  // ModeCreate と同じパース関数
  function parseCsvClient(text) {
    const norm = s => s.toLowerCase().replace(/[^a-z0-9]/g, '');
    const TIME_NORMS  = new Set(['times','timesec','timesecs','time','t','elapsed','elapseds','ts']);
    const SPEED_NORMS = new Set(['speedkmh','speedkm','speed','vkmh','v','vel','velocity','kmh','refspeed']);
    const lines = text.split('\n').map(l => l.trim()).filter(l => l);
    if (lines.length === 0) return { rows: [], sample: [], validations: [] };
    const header = lines[0].split(',').map(h => h.replace(/^"|"$/g, '').trim());
    const normed = header.map(h => norm(h));
    const timeIdx  = normed.findIndex(n => TIME_NORMS.has(n));
    const speedIdx = normed.findIndex(n => SPEED_NORMS.has(n));
    const ti = timeIdx  >= 0 ? timeIdx  : 0;
    const si = speedIdx >= 0 ? speedIdx : 1;
    const detected = timeIdx >= 0 && speedIdx >= 0;
    const vals = []; const sample = [lines[0]];
    let prevTime = -Infinity, hasNegSpeed = false, hasNonMono = false;
    for (let i = 1; i < lines.length; i++) {
      const cols = lines[i].split(',');
      const t = parseFloat(cols[ti]), s = parseFloat(cols[si]);
      if (isNaN(t) || isNaN(s)) continue;
      if (t <= prevTime) hasNonMono = true;
      if (s < 0) hasNegSpeed = true;
      vals.push({ time_s: t, speed_kmh: s });
      if (sample.length <= 5) sample.push(lines[i]);
      prevTime = t;
    }
    const maxS = vals.length > 0 ? Math.max(...vals.map(r => r.speed_kmh)) : 0;
    const colInfo = detected ? `${header[ti]} / ${header[si]}` : `列${ti+1} / 列${si+1} (位置で判定)`;
    return {
      rows: vals, sample,
      validations: [
        { ok: vals.length > 0, msg: `列検出: ${colInfo}` },
        { ok: !hasNonMono,     msg: '時刻 単調増加' },
        { ok: !hasNegSpeed,    msg: `車速範囲 0 – ${maxS.toFixed(1)} km/h` },
        { ok: vals.length > 0, msg: `サンプル数 ${vals.length} 行` },
      ],
    };
  }

  function handleFileChange(e) {
    const f = e.target.files[0];
    if (!f) return;
    setFile(f);
    const reader = new FileReader();
    reader.onload = ev => {
      const { rows, sample, validations: v } = parseCsvClient(ev.target.result);
      setCsvRows(rows);
      setCsvSample(sample);
      setValidations(v);
    };
    reader.readAsText(f);
  }

  async function handleSave() {
    if (!name.trim()) { window.showToast('モード名を入力してください', 'error'); return; }
    setSaving(true);
    await onSave(name.trim(), desc.trim(), file || null);
    setSaving(false);
  }

  // 統計 (新CSVまたは既存データ)
  const rows = csvRows;
  const totalDur  = rows.length > 0 ? rows[rows.length - 1].time_s : null;
  const maxSpeed  = rows.length > 0 ? Math.max(...rows.map(r => r.speed_kmh)) : null;
  const avgSpeed  = rows.length > 0 ? rows.reduce((s, r) => s + r.speed_kmh, 0) / rows.length : null;
  const stopRatio = rows.length > 0 ? rows.filter(r => r.speed_kmh === 0).length / rows.length * 100 : null;
  const fmtDur = (s) => s !== null ? `${Math.round(s)} s (${Math.round(s / 60)}分)` : '—';
  const fmtKmh = (v) => v !== null ? `${v.toFixed(1)} km/h` : '—';
  const fmtPct = (v) => v !== null ? `${v.toFixed(1)} %` : '—';

  return React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14, height: '100%' } },

    // ── ヘッダ ──────────────────────────────────────────────
    React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 12 } },
      React.createElement(Btn, { onClick: onCancel }, '← 一覧に戻る'),
      React.createElement(H2, {
        sub: `走行モード「${initial.name}」を編集します`,
      }, '走行モード · 編集'),
    ),

    // ── 2カラム本体 ─────────────────────────────────────────
    React.createElement('div', {
      style: { display: 'grid', gridTemplateColumns: '340px 1fr', gap: 14, flex: 1, minHeight: 0 }
    },

      // ── 左: フォーム ──────────────────────────────────────
      React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14 } },

        // 基本情報
        React.createElement(Box, { label: '基本情報', style: { padding: 18 } },
          React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14 } },
            React.createElement(Input, { label: 'モード名', value: name, onChange: setName, width: '100%' }),
            React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 3 } },
              React.createElement('div', { style: { fontSize: 13, color: INK_SOFT } }, '説明'),
              React.createElement('textarea', {
                value: desc,
                onChange: e => setDesc(e.target.value),
                style: {
                  width: '100%', padding: '7px 10px',
                  border: `1.3px solid ${INK}`, borderRadius: 3,
                  fontFamily: "'Patrick Hand', cursive",
                  fontSize: 15, background: PAPER,
                  minHeight: 60, resize: 'vertical', outline: 'none',
                  boxSizing: 'border-box',
                },
              }),
            ),
          ),
        ),

        // CSV 差し替え
        React.createElement(Box, {
          label: 'CSVファイル（Time[s], Speed[km/h]）', style: { padding: 18 },
        },
          React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 10 } },
            React.createElement('label', {
              style: {
                display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
                border: `1.5px solid ${INK}`, borderRadius: 6,
                padding: '6px 14px', background: PAPER,
                fontSize: 15, cursor: 'pointer', fontFamily: 'inherit', whiteSpace: 'nowrap',
              }
            },
              'ファイルを選択',
              React.createElement('input', {
                type: 'file', accept: '.csv',
                onChange: handleFileChange,
                style: { display: 'none' },
              }),
            ),
            React.createElement('div', {
              style: { fontSize: 13, color: file ? INK : INK_MUTE, fontFamily: 'inherit', whiteSpace: file ? 'normal' : 'nowrap', wordBreak: 'break-all' }
            }, file ? file.name : '差し替えない場合は選択不要'),
          ),

          validations.length > 0 && React.createElement('div', {
            style: { marginTop: 12, display: 'flex', flexDirection: 'column', gap: 5 }
          },
            React.createElement('div', { style: { fontSize: 13, fontWeight: 700, color: INK_SOFT, marginBottom: 2 } }, 'バリデーション'),
            ...validations.map(({ ok, msg }) =>
              React.createElement('div', {
                key: msg,
                style: { fontSize: 13, fontFamily: 'inherit', color: ok ? '#68d468' : '#f07070' }
              }, `${ok ? '✓' : '✗'} ${msg}`)
            ),
          ),
        ),

        React.createElement(Note, null, 'CSVを選択しない場合、名前と説明のみ更新されます。'),

        React.createElement('div', { style: { flex: 1 } }),

        // ボタン
        React.createElement('div', { style: { display: 'flex', gap: 10, justifyContent: 'flex-end' } },
          React.createElement(Btn, { danger: true, onClick: onDelete }, '削除'),
          React.createElement(Btn, { onClick: onCancel }, 'キャンセル'),
          React.createElement(Btn, { primary: true, big: true, onClick: handleSave, disabled: saving },
            saving ? '保存中…' : '保存'
          ),
        ),
      ),

      // ── 右: プレビュー ────────────────────────────────────
      React.createElement(Box, {
        label: 'プレビュー',
        style: { padding: 20, paddingTop: 28, display: 'flex', flexDirection: 'column', gap: 16 },
      },
        React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 8 } },
          React.createElement('div', { style: { fontSize: 14, color: INK_SOFT } },
            file ? '新しい基準車速プロファイル' : '現在の基準車速プロファイル'
          ),
          React.createElement(CsvSpeedGraph, { rows: csvRows, width: '100%', height: 280 }),
        ),
        React.createElement('div', {
          style: {
            display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 12,
            borderTop: `1px dashed ${HATCH}`, paddingTop: 14,
          }
        },
          ...[
            ['総時間',   fmtDur(totalDur)],
            ['最高車速', fmtKmh(maxSpeed)],
            ['平均車速', fmtKmh(avgSpeed)],
            ['停止比率', fmtPct(stopRatio)],
          ].map(([label, val]) =>
            React.createElement('div', { key: label, style: { display: 'flex', flexDirection: 'column', gap: 2 } },
              React.createElement('div', { style: { fontSize: 12, color: INK_SOFT } }, label),
              React.createElement('div', { style: { fontSize: 18, fontWeight: 700, fontFamily: 'inherit' } }, val),
            )
          ),
        ),
        csvSample.length > 0 && React.createElement(Box, { label: 'CSVサンプル (先頭5行)', style: { padding: 12 } },
          React.createElement('div', { style: { fontFamily: 'inherit', fontSize: 13, lineHeight: 1.8, color: INK_SOFT } },
            csvSample.map((line, i) =>
              React.createElement('div', { key: i, style: i === 0 ? { color: INK, fontWeight: 700 } : {} }, line)
            ),
            csvRows.length > 5 && React.createElement('div', { style: { color: INK_MUTE } }, `… (${csvRows.length - 5}行省略)`),
          ),
        ),
      ),
    ),
  );
}

function ModeCreate({ onSave, onCancel }) {
  const { useState, useContext } = React;
  const { apiFetch } = useContext(window.AppContext);
  const { Box, Btn, H2, Input, Note } = window;

  const [name, setName] = useState('');
  const [desc, setDesc] = useState('');
  const [file, setFile] = useState(null);
  const [csvRows, setCsvRows] = useState([]);
  const [csvSample, setCsvSample] = useState([]);
  const [validations, setValidations] = useState([]);
  const [uploading, setUploading] = useState(false);

  function detectColumns(header) {
    // 記号・空白を除去して小文字化
    const norm = s => s.toLowerCase().replace(/[^a-z0-9]/g, '');
    const TIME_NORMS  = new Set(['times','timesec','timesecs','time','t','elapsed','elapseds','ts']);
    const SPEED_NORMS = new Set(['speedkmh','speedkm','speed','vkmh','v','vel','velocity','kmh','refspeed']);
    const normed = header.map(h => norm(h));
    const timeIdx  = normed.findIndex(n => TIME_NORMS.has(n))  ?? -1;
    const speedIdx = normed.findIndex(n => SPEED_NORMS.has(n)) ?? -1;
    // 見つからなければ位置フォールバック (1列目=時刻, 2列目=速度)
    return {
      timeIdx:  timeIdx  >= 0 ? timeIdx  : 0,
      speedIdx: speedIdx >= 0 ? speedIdx : 1,
      detected: timeIdx >= 0 && speedIdx >= 0,
    };
  }

  function parseCsvClient(text) {
    const lines = text.split('\n').map(l => l.trim()).filter(l => l);
    if (lines.length === 0) return { rows: [], sample: [], validations: [] };

    const header = lines[0].split(',').map(h => h.replace(/^"|"$/g, '').trim());
    const { timeIdx, speedIdx, detected } = detectColumns(header);

    const vals = [];
    const sample = [lines[0]];
    let prevTime = -Infinity;
    let hasNegSpeed = false;
    let hasNonMono = false;

    for (let i = 1; i < lines.length; i++) {
      const cols = lines[i].split(',');
      const t = parseFloat(cols[timeIdx]);
      const s = parseFloat(cols[speedIdx]);
      if (isNaN(t) || isNaN(s)) continue;
      if (t <= prevTime) hasNonMono = true;
      if (s < 0) hasNegSpeed = true;
      vals.push({ time_s: t, speed_kmh: s });
      if (sample.length <= 5) sample.push(lines[i]);
      prevTime = t;
    }

    const maxS = vals.length > 0 ? Math.max(...vals.map(r => r.speed_kmh)) : 0;
    const colInfo = detected
      ? `${header[timeIdx]} / ${header[speedIdx]}`
      : `列${timeIdx + 1} / 列${speedIdx + 1} (位置で判定)`;
    const results = [
      { ok: vals.length > 0, msg: `列検出: ${colInfo}` },
      { ok: !hasNonMono, msg: '時刻 単調増加' },
      { ok: !hasNegSpeed, msg: `車速範囲 0 – ${maxS.toFixed(1)} km/h` },
      { ok: vals.length > 0, msg: `サンプル数 ${vals.length} 行` },
    ];

    return { rows: vals, sample, validations: results };
  }

  function handleFileChange(e) {
    const f = e.target.files[0];
    if (!f) return;
    setFile(f);
    if (!name.trim()) {
      const base = f.name.replace(/\.[^.]+$/, '');
      setName(base);
    }
    const reader = new FileReader();
    reader.onload = ev => {
      const { rows, sample, validations: v } = parseCsvClient(ev.target.result);
      setCsvRows(rows);
      setCsvSample(sample);
      setValidations(v);
    };
    reader.readAsText(f);
  }

  async function handleUpload() {
    if (!name.trim()) { window.showToast('モード名を入力してください', 'error'); return; }
    if (!file) { window.showToast('CSV ファイルを選択してください', 'error'); return; }
    setUploading(true);
    const fd = new FormData();
    fd.append('file', file);
    fd.append('name', name.trim());
    fd.append('description', desc.trim());
    const r = await apiFetch('POST', '/api/v1/modes/upload', fd, true);
    setUploading(false);
    if (r) { window.showToast(`「${r.name}」を作成しました`, 'success'); onSave(); }
  }

  // 統計計算
  const totalDur = csvRows.length > 0 ? csvRows[csvRows.length - 1].time_s : null;
  const maxSpeed = csvRows.length > 0 ? Math.max(...csvRows.map(r => r.speed_kmh)) : null;
  const avgSpeed = csvRows.length > 0
    ? (csvRows.reduce((s, r) => s + r.speed_kmh, 0) / csvRows.length)
    : null;
  const stopRatio = csvRows.length > 0
    ? (csvRows.filter(r => r.speed_kmh === 0).length / csvRows.length * 100)
    : null;

  const fmtDur = (s) => s !== null ? `${Math.round(s)} s (${Math.round(s / 60)}分)` : '—';
  const fmtKmh = (v) => v !== null ? `${v.toFixed(1)} km/h` : '—';
  const fmtPct = (v) => v !== null ? `${v.toFixed(1)} %` : '—';

  const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH } = window;

  return React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14, height: '100%' } },

    // ── ヘッダ ──────────────────────────────────────────────
    React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 12 } },
      React.createElement(Btn, { onClick: onCancel }, '← 一覧に戻る'),
      React.createElement(H2, {
        sub: '基準車速CSVをアップロードして新しい走行モードを登録します',
      }, '走行モード · 新規作成'),
    ),

    // ── 2カラム本体 ─────────────────────────────────────────
    React.createElement('div', {
      style: {
        display: 'grid', gridTemplateColumns: '340px 1fr',
        gap: 14, flex: 1, minHeight: 0,
      }
    },

      // ── 左: フォーム ──────────────────────────────────────
      React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14 } },

        // 基本情報
        React.createElement(Box, { label: '基本情報', style: { padding: 18 } },
          React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14 } },
            React.createElement(Input, {
              label: 'モード名', value: name,
              onChange: setName, width: '100%',
            }),
            React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 3 } },
              React.createElement('div', { style: { fontSize: 13, color: INK_SOFT } }, '説明'),
              React.createElement('textarea', {
                value: desc,
                onChange: e => setDesc(e.target.value),
                placeholder: '例: 国際標準 WLTP クラス3',
                style: {
                  width: '100%', padding: '7px 10px',
                  border: `1.3px solid ${INK}`, borderRadius: 3,
                  fontFamily: "'Patrick Hand', cursive",
                  fontSize: 15, background: PAPER,
                  minHeight: 60, resize: 'vertical', outline: 'none',
                  boxSizing: 'border-box',
                },
              }),
            ),
          ),
        ),

        // CSV アップロード
        React.createElement(Box, {
          label: 'CSVファイル（Time[s], Speed[km/h]）', style: { padding: 18 },
        },
          React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 10 } },
            React.createElement('label', {
              style: {
                display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
                border: `1.5px solid ${INK}`, borderRadius: 6,
                padding: '6px 14px', background: PAPER,
                fontSize: 15, cursor: 'pointer', fontFamily: 'inherit', whiteSpace: 'nowrap',
              }
            },
              'ファイルを選択',
              React.createElement('input', {
                type: 'file', accept: '.csv',
                onChange: handleFileChange,
                style: { display: 'none' },
              }),
            ),
            React.createElement('div', {
              style: { fontSize: 13, color: file ? INK : INK_MUTE, fontFamily: 'inherit', whiteSpace: file ? 'normal' : 'nowrap', wordBreak: 'break-all' }
            }, file ? file.name : '.csv / UTF-8 / ヘッダ行あり'),
          ),

          // バリデーション結果
          validations.length > 0 && React.createElement('div', {
            style: { marginTop: 12, display: 'flex', flexDirection: 'column', gap: 5 }
          },
            React.createElement('div', {
              style: { fontSize: 13, fontWeight: 700, color: INK_SOFT, marginBottom: 2 }
            }, 'バリデーション'),
            ...validations.map(({ ok, msg }) =>
              React.createElement('div', {
                key: msg,
                style: {
                  fontSize: 13, fontFamily: 'inherit',
                  color: ok ? '#68d468' : '#f07070',
                }
              }, `${ok ? '✓' : '✗'} ${msg}`)
            ),
          ),
        ),

        React.createElement(Note, null, 'CSVの1列目は秒単位の時刻、2列目は km/h の基準車速。ヘッダ行は必須です。'),

        React.createElement('div', { style: { flex: 1 } }),

        // ボタン
        React.createElement('div', {
          style: { display: 'flex', gap: 10, justifyContent: 'flex-end' }
        },
          React.createElement(Btn, { onClick: onCancel }, 'キャンセル'),
          React.createElement(Btn, {
            primary: true, big: true,
            onClick: handleUpload, disabled: uploading,
          }, uploading ? 'アップロード中…' : '保存'),
        ),
      ),

      // ── 右: プレビュー ────────────────────────────────────
      React.createElement(Box, {
        label: 'プレビュー',
        style: { padding: 20, paddingTop: 28, display: 'flex', flexDirection: 'column', gap: 16 },
      },

        // 速度グラフ
        React.createElement('div', {
          style: { display: 'flex', flexDirection: 'column', gap: 8 }
        },
          React.createElement('div', { style: { fontSize: 14, color: INK_SOFT } }, '基準車速プロファイル'),
          React.createElement(CsvSpeedGraph, { rows: csvRows, width: '100%', height: 280 }),
        ),

        // 統計サマリ
        React.createElement('div', {
          style: {
            display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 12,
            borderTop: `1px dashed ${HATCH}`, paddingTop: 14,
          }
        },
          ...[
            ['総時間', fmtDur(totalDur)],
            ['最高車速', fmtKmh(maxSpeed)],
            ['平均車速', fmtKmh(avgSpeed)],
            ['停止比率', fmtPct(stopRatio)],
          ].map(([label, val]) =>
            React.createElement('div', {
              key: label,
              style: { display: 'flex', flexDirection: 'column', gap: 2 }
            },
              React.createElement('div', { style: { fontSize: 12, color: INK_SOFT } }, label),
              React.createElement('div', {
                style: { fontSize: 18, fontWeight: 700, fontFamily: 'inherit' }
              }, val),
            )
          ),
        ),

        // CSVサンプル
        React.createElement(Box, { label: 'CSVサンプル (先頭5行)', style: { padding: 12 } },
          csvSample.length > 0
            ? React.createElement('div', {
                style: { fontFamily: 'inherit', fontSize: 13, lineHeight: 1.8, color: INK_SOFT }
              },
                csvSample.map((line, i) =>
                  React.createElement('div', {
                    key: i,
                    style: i === 0 ? { color: INK, fontWeight: 700 } : {},
                  }, line)
                ),
                csvRows.length > 5 && React.createElement('div', {
                  style: { color: INK_MUTE }
                }, `… (${csvRows.length - 5}行省略)`),
              )
            : React.createElement('div', {
                style: { fontSize: 13, color: INK_MUTE, fontFamily: 'inherit' }
              }, 'CSVをアップロードすると表示されます'),
        ),
      ),
    ),
  );
}

window.ModesScreen = ModesScreen;
