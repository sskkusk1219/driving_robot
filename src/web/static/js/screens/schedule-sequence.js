// ── Schedule (タイムスケジュール) & Sequence screens ────────

// タイムスケジュール: ペダル開度とボタン押下を統合タイムラインで再生する。
// 1行=1時刻点の表エディタで編集し、保存時に pedal_points / button_events へ変換する。

const CHANNEL_NAMES = {
  0: 'スタート', 1: 'P', 2: 'N', 3: 'D',
  4: 'オプション1', 5: 'オプション2', 6: 'オプション3', 7: 'オプション4',
  8: 'オプション5', 9: 'オプション6', 10: 'オプション7', 11: 'オプション8',
  12: 'オプション9', 13: 'オプション10', 14: 'オプション11', 15: 'オプション12',
};
const DEFAULT_CHANNELS = [0, 1, 2, 3];
const EXCLUSIVE_CHANNELS = [1, 2, 3]; // P/N/D は同一行で排他（最後に押したONを優先）
const round2 = (v) => Math.round(v * 100) / 100;

// ── 表 ⇔ APIモデル 変換 ───────────────────────────────────────

// rows: [{ time_s, accel, brake, buttons: { [channel]: boolean } }]
function rowsToApi(rows, visibleChannels) {
  const pedal_points = rows.map(r => ({
    time_s: r.time_s, accel_opening: r.accel, brake_opening: r.brake,
  }));
  const button_events = [];
  visibleChannels.slice().sort((a, b) => a - b).forEach(ch => {
    let onStart = null;
    for (const r of rows) {
      const on = !!r.buttons[ch];
      if (on && onStart === null) {
        onStart = r.time_s;
      } else if (!on && onStart !== null) {
        button_events.push({ time_s: onStart, channel: ch, press_duration_s: round2(r.time_s - onStart) });
        onStart = null;
      }
    }
  });
  button_events.sort((a, b) => a.time_s - b.time_s || a.channel - b.channel);
  return { pedal_points, button_events };
}

// detail: ScheduleDetailResponse（pedal_points / button_events）
function apiToRows(detail) {
  const pedalPoints = (detail.pedal_points || []).slice().sort((a, b) => a.time_s - b.time_s);
  const buttonEvents = detail.button_events || [];

  const times = new Set(pedalPoints.map(p => p.time_s));
  buttonEvents.forEach(e => {
    times.add(e.time_s);
    times.add(round2(e.time_s + e.press_duration_s));
  });
  const sortedTimes = Array.from(times).sort((a, b) => a - b);

  function interp(t) {
    if (pedalPoints.length === 0) return { accel: 0, brake: 0 };
    if (t <= pedalPoints[0].time_s) {
      return { accel: pedalPoints[0].accel_opening, brake: pedalPoints[0].brake_opening };
    }
    const last = pedalPoints[pedalPoints.length - 1];
    if (t >= last.time_s) return { accel: last.accel_opening, brake: last.brake_opening };
    for (let i = 0; i < pedalPoints.length - 1; i++) {
      const a = pedalPoints[i], b = pedalPoints[i + 1];
      if (t >= a.time_s && t <= b.time_s) {
        const f = (t - a.time_s) / (b.time_s - a.time_s);
        return {
          accel: round2(a.accel_opening + (b.accel_opening - a.accel_opening) * f),
          brake: round2(a.brake_opening + (b.brake_opening - a.brake_opening) * f),
        };
      }
    }
    return { accel: 0, brake: 0 };
  }

  const rows = sortedTimes.map(t => ({ time_s: t, ...interp(t), buttons: {} }));
  if (rows.length === 0) rows.push({ time_s: 0, accel: 0, brake: 0, buttons: {} });

  const channelsUsed = new Set(DEFAULT_CHANNELS);
  buttonEvents.forEach(e => {
    channelsUsed.add(e.channel);
    const start = e.time_s, end = round2(e.time_s + e.press_duration_s);
    rows.forEach(row => {
      if (row.time_s >= start && row.time_s < end) row.buttons[e.channel] = true;
    });
  });

  return { rows, visibleChannels: Array.from(channelsUsed).sort((a, b) => a - b) };
}

// 各 ch の連続 ON 区間（保存前プレビュー用。未終端の ON も含めて返す）
function computeIntervals(rows, visibleChannels) {
  const result = [];
  visibleChannels.forEach(ch => {
    let start = null;
    for (const r of rows) {
      const on = !!r.buttons[ch];
      if (on && start === null) {
        start = r.time_s;
      } else if (!on && start !== null) {
        result.push({ channel: ch, start, end: r.time_s });
        start = null;
      }
    }
    if (start !== null) result.push({ channel: ch, start, end: null });
  });
  return result.sort((a, b) => a.start - b.start);
}

// 隣接する行ペアごとの開度区間（アクセル・ブレーキ開度プレビュー用）
function computePedalSegments(rows) {
  const segments = [];
  for (let i = 0; i < rows.length - 1; i++) {
    const a = rows[i], b = rows[i + 1];
    segments.push({
      start: a.time_s, end: b.time_s,
      accelStart: a.accel, accelEnd: b.accel,
      brakeStart: a.brake, brakeEnd: b.brake,
    });
  }
  return segments;
}

function validateSchedule(name, rows) {
  const errors = [];
  if (!name.trim()) errors.push({ message: 'スケジュール名を入力してください' });
  if (rows.length === 0) errors.push({ message: '行を1つ以上追加してください' });

  rows.forEach((r, i) => {
    if (i > 0 && r.time_s <= rows[i - 1].time_s) {
      errors.push({ rowIdx: i, field: 'time_s', message: `${i + 1}行目: 時刻は前の行より大きい値にしてください` });
    }
    if (r.accel < 0 || r.accel > 100) {
      errors.push({ rowIdx: i, field: 'accel', message: `${i + 1}行目: Accは0〜100の範囲にしてください` });
    }
    if (r.brake < 0 || r.brake > 100) {
      errors.push({ rowIdx: i, field: 'brake', message: `${i + 1}行目: Brkは0〜100の範囲にしてください` });
    }
  });

  const last = rows[rows.length - 1];
  if (last && Object.values(last.buttons).some(Boolean)) {
    errors.push({ rowIdx: rows.length - 1, message: '最終行はすべてのボタンをOFFにしてください（押下終了時刻が確定しません）' });
  }

  return { errors };
}

function ScheduleScreen() {
  const { useState, useEffect, useContext } = React;
  const {
    apiFetch, robotState,
    activeModeId, activeModeKind,
    setActiveModeId, setActiveModeName, setActiveModeKind,
  } = useContext(window.AppContext);
  const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH, Box, Btn, RowActions } = window;

  const [schedules, setSchedules] = useState([]);
  const [loading, setLoading] = useState(true);
  const [mode, setMode] = useState('list'); // list | create | edit
  const [editTarget, setEditTarget] = useState(null);
  const [search, setSearch] = useState('');

  const running = robotState === 'RUNNING';
  const selectedId = activeModeKind === 'schedule' ? activeModeId : null;

  async function refresh() {
    setLoading(true);
    const data = await apiFetch('GET', '/api/v1/schedules/');
    setSchedules(data || []);
    setLoading(false);
  }

  useEffect(() => { refresh(); }, []);

  function handleSelect(s) {
    setActiveModeId(s.id);
    setActiveModeName(s.name);
    setActiveModeKind('schedule');
    window.showToast(`「${s.name}」を選択しました。自動運転ページで開始できます`, 'success');
  }

  async function handleDelete(s) {
    const r = await apiFetch('DELETE', `/api/v1/schedules/${s.id}`);
    if (r !== null) {
      window.showToast('スケジュールを削除しました', 'success');
      if (activeModeKind === 'schedule' && activeModeId === s.id) {
        setActiveModeId(null);
        setActiveModeName(null);
      }
      refresh();
    }
  }

  async function handleEdit(s) {
    const detail = await apiFetch('GET', `/api/v1/schedules/${s.id}`);
    if (detail) { setEditTarget(detail); setMode('edit'); }
  }

  if (mode === 'create' || mode === 'edit') {
    return (
      <ScheduleEditForm
        initial={mode === 'edit' ? editTarget : null}
        onCancel={() => { setMode('list'); setEditTarget(null); }}
        onSaved={() => { setMode('list'); setEditTarget(null); refresh(); }}
      />
    );
  }

  const gridCols = '2fr 1.6fr 0.9fr 0.8fr 0.8fr 2fr';
  const q = search.trim().toLowerCase();
  const filtered = schedules.filter(s =>
    s.name.toLowerCase().includes(q) ||
    (s.description || '').toLowerCase().includes(q)
  );

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 14, height: '100%' }}>

      <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
        <div style={{ flex: 1 }} />
        <input
          type="text"
          value={search}
          onChange={e => setSearch(e.target.value)}
          placeholder="名前・説明で検索…"
          style={{
            padding: '6px 12px', fontSize: 14,
            border: `1.3px solid ${INK}`, borderRadius: 4,
            fontFamily: 'inherit', background: PAPER, color: INK, outline: 'none',
            width: 200,
          }}
        />
        <Btn primary big onClick={() => setMode('create')} disabled={running}>＋ 新規作成</Btn>
      </div>

      <Box style={{ padding: 0, display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 }}>
        <div style={{
          display: 'grid', gridTemplateColumns: gridCols,
          borderBottom: `1px solid ${INK}`, padding: '10px 14px',
          background: PAPER_2, fontSize: 14, fontWeight: 700, flexShrink: 0,
        }}>
          <div>名前</div>
          <div>説明</div>
          <div style={{ fontFamily: 'monospace' }}>総時間[s]</div>
          <div style={{ fontFamily: 'monospace' }}>ペダル点</div>
          <div style={{ fontFamily: 'monospace' }}>ボタン</div>
          <div>操作</div>
        </div>

        <div style={{ overflowY: 'auto', flex: 1, minHeight: 0 }}>
          {loading ? (
            <div style={{ padding: '20px 14px', color: INK_SOFT, fontSize: 14 }}>読み込み中…</div>
          ) : filtered.length === 0 ? (
            <div style={{ padding: '20px 14px', color: INK_SOFT, fontSize: 14 }}>
              {schedules.length === 0
                ? 'スケジュールがありません。「新規作成」から追加してください。'
                : `「${search}」に一致するスケジュールがありません。`}
            </div>
          ) : (
            filtered.map(s => {
              const isActive = selectedId === s.id;
              return (
                <div key={s.id} style={{
                  display: 'grid', gridTemplateColumns: gridCols,
                  padding: '10px 14px', alignItems: 'center',
                  background: isActive ? '#201e16' : 'transparent',
                  borderBottom: `1px dashed ${HATCH}`,
                }}>
                  <div><b>{s.name}</b></div>
                  <div style={{ color: s.description ? INK : INK_MUTE }}>{s.description || '—'}</div>
                  <div style={{ fontFamily: 'monospace' }}>{s.total_duration.toFixed(1)}</div>
                  <div style={{ fontFamily: 'monospace' }}>{s.pedal_point_count}</div>
                  <div style={{ fontFamily: 'monospace' }}>{s.button_event_count}</div>
                  <RowActions
                    isActive={isActive}
                    onSelect={() => handleSelect(s)}
                    onEdit={running ? null : () => handleEdit(s)}
                    onDelete={running ? null : () => handleDelete(s)}
                  />
                </div>
              );
            })
          )}
        </div>
      </Box>
    </div>
  );
}

// 新規作成・編集共通の統合タイムライン表エディタ
// initial: ScheduleDetailResponse | null（あり=編集/PUT、なし=新規/POST）
function ScheduleEditForm({ initial, onCancel, onSaved }) {
  const { useState } = React;
  const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH, Box, Btn, H2, Note, Input } = window;
  const isEdit = !!initial;

  const initialConverted = initial ? apiToRows(initial) : null;

  const [name, setName] = useState(initial?.name ?? '');
  const [desc, setDesc] = useState(initial?.description ?? '');
  const [rows, setRows] = useState(() =>
    initialConverted ? initialConverted.rows : [{ time_s: 0, accel: 0, brake: 0, buttons: {} }]);
  const [visibleChannels, setVisibleChannels] = useState(() =>
    initialConverted ? initialConverted.visibleChannels : DEFAULT_CHANNELS.slice());
  const [addSelectValue, setAddSelectValue] = useState('');
  const [errors, setErrors] = useState([]);
  const [saving, setSaving] = useState(false);

  const errorSet = new Set(errors.filter(e => e.rowIdx !== undefined).map(e => `${e.rowIdx}:${e.field || ''}`));
  const cellStyle = (rowIdx, field) => ({
    ...numberInputStyle,
    borderColor: errorSet.has(`${rowIdx}:${field}`) ? '#b04040' : HATCH,
  });

  function updateRow(idx, patch) {
    setRows(rs => rs.map((r, i) => (i === idx ? { ...r, ...patch } : r)));
  }

  function toggleButton(idx, ch) {
    setRows(rs => rs.map((r, i) => {
      if (i !== idx) return r;
      const turningOn = !r.buttons[ch];
      const buttons = { ...r.buttons };
      if (turningOn && EXCLUSIVE_CHANNELS.includes(ch)) {
        // P/N/D は同一行で共存不可。最後に押したONを優先し他は自動OFF
        EXCLUSIVE_CHANNELS.forEach(c => { if (c !== ch) buttons[c] = false; });
      }
      buttons[ch] = turningOn;
      return { ...r, buttons };
    }));
  }

  function insertRowBelow(idx) {
    setRows(rs => {
      const prev = rs[idx];
      const newRow = { time_s: round2(prev.time_s + 1.0), accel: prev.accel, brake: prev.brake, buttons: { ...prev.buttons } };
      const copy = rs.slice();
      copy.splice(idx + 1, 0, newRow);
      return copy;
    });
  }

  function deleteRow(idx) {
    setRows(rs => (rs.length <= 1 ? rs : rs.filter((_, i) => i !== idx)));
  }

  function addColumn(ch) {
    setVisibleChannels(vs => [...vs, ch].sort((a, b) => a - b));
    setAddSelectValue('');
  }

  function removeColumn(ch) {
    const allOff = rows.every(r => !r.buttons[ch]);
    if (!allOff) { window.showToast('この列をすべての行でOFFにしてから削除してください', 'error'); return; }
    setVisibleChannels(vs => vs.filter(c => c !== ch));
  }

  async function handleSave() {
    const { errors: errs } = validateSchedule(name, rows);
    setErrors(errs);
    if (errs.length > 0) { window.showToast(errs[0].message, 'error'); return; }

    const { pedal_points, button_events } = rowsToApi(rows, visibleChannels);
    const payload = { name: name.trim(), description: desc, pedal_points, button_events };
    setSaving(true);
    const r = isEdit
      ? await window.apiFetch('PUT', `/api/v1/schedules/${initial.id}`, payload)
      : await window.apiFetch('POST', '/api/v1/schedules/', payload);
    setSaving(false);
    if (r) {
      window.showToast(`「${r.name}」を${isEdit ? '更新' : '作成'}しました`, 'success');
      onSaved();
    }
  }

  const totalDuration = rows.length ? rows[rows.length - 1].time_s : 0;
  const intervals = computeIntervals(rows, visibleChannels);
  const pedalSegments = computePedalSegments(rows);
  const availableChannels = Array.from({ length: 12 }, (_, i) => i + 4).filter(ch => !visibleChannels.includes(ch));
  const gridCols = `84px 84px 84px ${visibleChannels.map(() => '92px').join(' ')} 1fr`;

  return (
    <div style={{ padding: 32, height: '100%', overflowY: 'auto' }}>
      <H2 sub={isEdit ? `スケジュール「${initial.name}」を編集します` : '時刻ごとにペダル開度とボタン押下を設定します'}>
        {isEdit ? 'スケジュール編集' : 'スケジュール新規作成'}
      </H2>

      <Box style={{ padding: 20, marginTop: 16 }}>
        <div style={{ display: 'flex', gap: 16, flexWrap: 'wrap' }}>
          <Input label="名前" value={name} onChange={v => setName(v)} width={260} />
          <Input label="説明" value={desc} onChange={v => setDesc(v)} width={360} />
        </div>
      </Box>

      <Box style={{ padding: 16, marginTop: 16 }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 10 }}>
          <div style={{ fontSize: 14, color: INK_SOFT }}>タイムライン（{rows.length} 行 / 総時間 {totalDuration.toFixed(1)}s）</div>
          {availableChannels.length > 0 && (
            <select
              value={addSelectValue}
              onChange={e => { if (e.target.value) addColumn(Number(e.target.value)); }}
              style={selectStyle}
            >
              <option value="">＋ 列追加…</option>
              {availableChannels.map(ch => (
                <option key={ch} value={ch}>{CHANNEL_NAMES[ch]}</option>
              ))}
            </select>
          )}
        </div>

        <div style={{ overflowX: 'auto' }}>
          <div style={{ minWidth: 'fit-content' }}>
            <div style={{
              display: 'grid', gridTemplateColumns: gridCols, gap: 6,
              borderBottom: `1px solid ${INK}`, padding: '4px 4px 8px', fontSize: 13,
              fontWeight: 700, color: INK_SOFT,
            }}>
              <div>時刻[s]</div><div>Acc[%]</div><div>Brk[%]</div>
              {visibleChannels.map(ch => (
                <div key={ch} style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 4 }}>
                  <span>{CHANNEL_NAMES[ch]}</span>
                  {ch >= 4 && (
                    <span onClick={() => removeColumn(ch)} title="列を削除"
                          style={{ cursor: 'pointer', opacity: 0.55, fontSize: 12, userSelect: 'none' }}>✕</span>
                  )}
                </div>
              ))}
              <div>操作</div>
            </div>

            {rows.map((r, idx) => (
              <div key={idx} style={{
                display: 'grid', gridTemplateColumns: gridCols, gap: 6,
                padding: '6px 4px', borderBottom: `1px dashed ${HATCH}`, alignItems: 'center',
              }}>
                <input type="number" step="0.1" value={r.time_s}
                       onChange={e => updateRow(idx, { time_s: Number(e.target.value) })}
                       style={cellStyle(idx, 'time_s')} />
                <input type="number" min="0" max="100" step="1" value={r.accel}
                       onChange={e => updateRow(idx, { accel: Number(e.target.value) })}
                       style={cellStyle(idx, 'accel')} />
                <input type="number" min="0" max="100" step="1" value={r.brake}
                       onChange={e => updateRow(idx, { brake: Number(e.target.value) })}
                       style={cellStyle(idx, 'brake')} />
                {visibleChannels.map(ch => {
                  const on = !!r.buttons[ch];
                  return (
                    <div key={ch} onClick={() => toggleButton(idx, ch)} style={{
                      cursor: 'pointer', textAlign: 'center', padding: '5px 4px', borderRadius: 4,
                      userSelect: 'none', fontSize: 13, fontWeight: on ? 700 : 400,
                      background: on ? 'rgba(200,146,42,0.16)' : 'transparent',
                      border: `1px solid ${on ? '#c8922a' : 'transparent'}`,
                      color: on ? '#c8922a' : INK_MUTE,
                    }}>{on ? '● ON' : '—'}</div>
                  );
                })}
                <div style={{ display: 'flex', gap: 6 }}>
                  <Btn onClick={() => insertRowBelow(idx)} style={{ padding: '4px 10px' }}>＋</Btn>
                  <Btn danger disabled={rows.length <= 1} onClick={() => deleteRow(idx)} style={{ padding: '4px 10px' }}>✕</Btn>
                </div>
              </div>
            ))}
          </div>
        </div>
      </Box>

      <Box style={{ padding: 16, marginTop: 16 }}>
        <div style={{ fontSize: 14, color: INK_SOFT, marginBottom: 10 }}>ペダル開度・ボタン押下プレビュー</div>
        {[{ key: 'accel', label: 'Acc', color: '#78c8f0' }, { key: 'brake', label: 'Brk', color: '#f07070' }].map(({ key, label, color }) => {
          const span = Math.max(totalDuration, 0.001);
          const startKey = `${key}Start`, endKey = `${key}End`;
          return (
            <div key={key} style={{ display: 'flex', alignItems: 'center', gap: 10, marginBottom: 6 }}>
              <div style={{ width: 84, fontSize: 13, color: INK_SOFT, flexShrink: 0 }}>{label}</div>
              <div style={{ position: 'relative', flex: 1, height: 18, background: PAPER, border: `1px solid ${HATCH}`, borderRadius: 3 }}>
                {pedalSegments.map((seg, i) => {
                  const left = (seg.start / span) * 100;
                  const width = ((seg.end - seg.start) / span) * 100;
                  const opStart = Math.max(0, Math.min(1, seg[startKey] / 100));
                  const opEnd = Math.max(0, Math.min(1, seg[endKey] / 100));
                  return (
                    <div key={i} title={`${seg.start.toFixed(1)}s: ${seg[startKey].toFixed(0)}% → ${seg.end.toFixed(1)}s: ${seg[endKey].toFixed(0)}%`} style={{
                      position: 'absolute', left: `${left}%`, width: `${Math.max(width, 0.6)}%`, top: 0, bottom: 0,
                      background: `linear-gradient(to right, ${color}${Math.round(opStart * 255).toString(16).padStart(2, '0')}, ${color}${Math.round(opEnd * 255).toString(16).padStart(2, '0')})`,
                    }} />
                  );
                })}
              </div>
              <div style={{ width: 190, fontSize: 12, color: INK_SOFT, flexShrink: 0 }}>
                {rows.length ? `${Math.min(...rows.map(r => r[key])).toFixed(0)}〜${Math.max(...rows.map(r => r[key])).toFixed(0)}%` : '—'}
              </div>
            </div>
          );
        })}
        {visibleChannels.map(ch => {
          const chIntervals = intervals.filter(iv => iv.channel === ch);
          const span = Math.max(totalDuration, 0.001);
          return (
            <div key={ch} style={{ display: 'flex', alignItems: 'center', gap: 10, marginBottom: 6 }}>
              <div style={{ width: 84, fontSize: 13, color: INK_SOFT, flexShrink: 0 }}>{CHANNEL_NAMES[ch]}</div>
              <div style={{ position: 'relative', flex: 1, height: 18, background: PAPER, border: `1px solid ${HATCH}`, borderRadius: 3 }}>
                {chIntervals.map((iv, i) => {
                  const left = (iv.start / span) * 100;
                  const width = iv.end === null ? (100 - left) : ((iv.end - iv.start) / span) * 100;
                  return (
                    <div key={i} title={`${iv.start.toFixed(1)}s〜${iv.end === null ? '?' : iv.end.toFixed(1) + 's'}`} style={{
                      position: 'absolute', left: `${left}%`, width: `${Math.max(width, 0.6)}%`, top: 0, bottom: 0,
                      background: iv.end === null ? '#b04040' : '#c8922a', opacity: 0.85, borderRadius: 2,
                    }} />
                  );
                })}
              </div>
              <div style={{ width: 190, fontSize: 12, color: INK_SOFT, flexShrink: 0 }}>
                {chIntervals.length === 0
                  ? '—'
                  : chIntervals.map(iv => `${iv.start.toFixed(1)}〜${iv.end === null ? '未終了' : iv.end.toFixed(1) + 's'}`).join(', ')}
              </div>
            </div>
          );
        })}
      </Box>

      {errors.length > 0 && (
        <Note style={{ marginTop: 16 }}>{errors[0].message}</Note>
      )}

      <div style={{ display: 'flex', gap: 12, marginTop: 20 }}>
        <Btn primary disabled={saving} onClick={handleSave}>{saving ? '保存中…' : '保存'}</Btn>
        <Btn ghost onClick={onCancel}>キャンセル</Btn>
      </div>
    </div>
  );
}

const numberInputStyle = {
  width: '100%', padding: '5px 6px', textAlign: 'right',
  border: `1px solid ${window.HATCH}`, borderRadius: 4,
  fontFamily: "'JetBrains Mono', monospace", fontSize: 13,
  background: window.PAPER, color: window.INK,
  outline: 'none',
};

const selectStyle = {
  padding: '5px 10px', fontSize: 13,
  border: `1px solid ${window.HATCH}`, borderRadius: 4,
  fontFamily: 'inherit', background: window.PAPER, color: window.INK,
  outline: 'none',
};

function SequenceScreen() {
  const { Box, H2, Note } = window;
  return (
    <div style={{ padding: 32 }}>
      <H2>シーケンス</H2>
      <Box style={{ padding: 32, marginTop: 20, textAlign: 'center' }}>
        <Note>シーケンス機能は Post-MVP で実装予定です。</Note>
      </Box>
    </div>
  );
}

window.ScheduleScreen = ScheduleScreen;
window.ScheduleEditForm = ScheduleEditForm;
window.SequenceScreen = SequenceScreen;
