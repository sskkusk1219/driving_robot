// ── Schedule (タイムスケジュール) & Sequence screens ────────

// タイムスケジュール: ペダル開度とボタン押下を統合タイムラインで再生する。
// ペダル点・ボタンイベントは JSON で入力する（フルのグラフィカルエディタは Post-MVP）。
function ScheduleScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, robotState, setNavLock, activeProfileId } = useContext(window.AppContext);
  const { INK, INK_SOFT, PAPER_2, HATCH, Box, Btn, H2, Note, Pill, Row, RowActions, Input,
          ConfirmStopPopup } = window;

  const [schedules, setSchedules] = useState([]);
  const [loading, setLoading] = useState(true);
  const [mode, setMode] = useState('list'); // list | create
  const [selectedId, setSelectedId] = useState(null);
  const [confirmStop, setConfirmStop] = useState(false);

  const running = robotState === 'RUNNING';

  async function refresh() {
    setLoading(true);
    const data = await apiFetch('GET', '/api/v1/schedules/');
    setSchedules(data || []);
    setLoading(false);
  }

  useEffect(() => { refresh(); }, []);

  async function handleDelete(s) {
    const r = await apiFetch('DELETE', `/api/v1/schedules/${s.id}`);
    if (r) {
      window.showToast('スケジュールを削除しました', 'success');
      if (selectedId === s.id) setSelectedId(null);
      refresh();
    }
  }

  async function handleStart() {
    if (!selectedId) { window.showToast('スケジュールを選択してください', 'error'); return; }
    if (!activeProfileId) { window.showToast('車両プロファイルを選択してください', 'error'); return; }
    const r = await apiFetch('POST', '/api/v1/drive/schedule/start', { schedule_id: selectedId });
    if (r) {
      window.showToast('タイムスケジュール走行を開始しました', 'success');
      setNavLock(true);
    }
  }

  async function handleStop() {
    setConfirmStop(false);
    const r = await apiFetch('POST', '/api/v1/drive/schedule/stop');
    if (r) {
      window.showToast('タイムスケジュール走行を停止しました', 'success');
      setNavLock(false);
    }
  }

  if (mode === 'create') {
    return React.createElement(ScheduleCreateForm, {
      onCancel: () => setMode('list'),
      onCreated: () => { setMode('list'); refresh(); },
    });
  }

  return React.createElement('div', { style: { padding: 32 } },
    React.createElement(H2, { sub: 'ペダル開度とボタン押下を時系列で自動再生します' }, 'タイムスケジュール'),

    // 走行制御
    React.createElement(Box, { style: { padding: 20, marginTop: 16, display: 'flex', gap: 12, alignItems: 'center' } },
      React.createElement(Pill, { accent: running }, running ? '走行中' : '待機'),
      React.createElement('div', { style: { flex: 1 } }),
      React.createElement(Btn, {
        primary: true, disabled: running || !selectedId, onClick: handleStart,
      }, '選択スケジュールで走行'),
      React.createElement(Btn, {
        danger: true, disabled: !running, onClick: () => setConfirmStop(true),
      }, '停止'),
    ),

    // 一覧
    React.createElement('div', { style: { marginTop: 20, display: 'flex', justifyContent: 'space-between', alignItems: 'center' } },
      React.createElement('div', { style: { fontSize: 14, color: INK_SOFT } },
        `登録済み: ${schedules.length} 件`),
      React.createElement(Btn, { onClick: () => setMode('create'), disabled: running }, '＋ 新規作成'),
    ),

    loading
      ? React.createElement(Note, { style: { marginTop: 20 } }, '読み込み中…')
      : schedules.length === 0
        ? React.createElement(Box, { style: { padding: 32, marginTop: 16, textAlign: 'center' } },
            React.createElement(Note, null, 'スケジュールがありません。「新規作成」から追加してください。'))
        : React.createElement('div', { style: { marginTop: 12 } },
            React.createElement(Row, {
              header: true,
              cells: ['', '名前', '総時間[s]', 'ペダル点', 'ボタン', 'ループ', ''],
            }),
            ...schedules.map(s =>
              React.createElement(Row, {
                key: s.id,
                style: selectedId === s.id ? { background: PAPER_2, borderColor: INK } : {},
                cells: [
                  React.createElement('input', {
                    type: 'radio', checked: selectedId === s.id,
                    onChange: () => setSelectedId(s.id), disabled: running,
                  }),
                  s.name,
                  s.total_duration.toFixed(1),
                  String(s.pedal_point_count),
                  String(s.button_event_count),
                  s.loop ? '有' : '—',
                  React.createElement(RowActions, {
                    onSelect: () => setSelectedId(s.id),
                    isActive: selectedId === s.id,
                    onDelete: running ? null : () => handleDelete(s),
                  }),
                ],
              })
            ),
          ),

    confirmStop && React.createElement(ConfirmStopPopup, {
      message: 'タイムスケジュール走行を停止しますか？',
      onYes: handleStop,
      onNo: () => setConfirmStop(false),
    }),
  );
}

// 新規作成フォーム: 名前・説明・ループ + ペダル点/ボタンイベントを JSON で入力
function ScheduleCreateForm({ onCancel, onCreated }) {
  const { useState } = React;
  const { INK_SOFT, HATCH, PAPER_2, Box, Btn, H2, Note, Input } = window;

  const [name, setName] = useState('');
  const [desc, setDesc] = useState('');
  const [loop, setLoop] = useState(false);
  const [pedalJson, setPedalJson] = useState(
    '[\n  {"time_s": 0, "accel_opening": 0, "brake_opening": 0},\n  {"time_s": 5, "accel_opening": 40, "brake_opening": 0},\n  {"time_s": 10, "accel_opening": 0, "brake_opening": 30}\n]');
  const [buttonJson, setButtonJson] = useState(
    '[\n  {"time_s": 0.5, "channel": 0, "press_duration_s": 1.0}\n]');
  const [saving, setSaving] = useState(false);

  function parseOrNull(text, label) {
    try {
      const v = JSON.parse(text || '[]');
      if (!Array.isArray(v)) throw new Error('配列である必要があります');
      return v;
    } catch (e) {
      window.showToast(`${label} の JSON が不正です: ${e.message}`, 'error');
      return null;
    }
  }

  async function handleSave() {
    if (!name.trim()) { window.showToast('スケジュール名を入力してください', 'error'); return; }
    const pedal_points = parseOrNull(pedalJson, 'ペダル点');
    if (pedal_points === null) return;
    const button_events = parseOrNull(buttonJson, 'ボタンイベント');
    if (button_events === null) return;
    setSaving(true);
    const r = await window.apiFetch('POST', '/api/v1/schedules/', {
      name: name.trim(), description: desc, pedal_points, button_events, loop,
    });
    setSaving(false);
    if (r) {
      window.showToast(`「${r.name}」を作成しました`, 'success');
      onCreated();
    }
  }

  const taStyle = {
    width: '100%', minHeight: 120, background: PAPER_2, color: window.INK,
    border: `1.4px solid ${HATCH}`, borderRadius: 4, padding: 10,
    fontFamily: 'monospace', fontSize: 12, resize: 'vertical',
  };

  return React.createElement('div', { style: { padding: 32 } },
    React.createElement(H2, { sub: 'ペダル点とボタンイベントを JSON で定義します' }, 'スケジュール新規作成'),
    React.createElement(Box, { style: { padding: 20, marginTop: 16 } },
      React.createElement('div', { style: { display: 'flex', gap: 16, flexWrap: 'wrap' } },
        React.createElement(Input, { label: '名前', value: name, onChange: e => setName(e.target.value), width: 260 }),
        React.createElement(Input, { label: '説明', value: desc, onChange: e => setDesc(e.target.value), width: 360 }),
      ),
      React.createElement('label', { style: { display: 'flex', gap: 8, alignItems: 'center', marginTop: 16, color: INK_SOFT, fontSize: 14 } },
        React.createElement('input', { type: 'checkbox', checked: loop, onChange: e => setLoop(e.target.checked) }),
        'ループ再生する',
      ),
      React.createElement('div', { style: { marginTop: 16 } },
        React.createElement('div', { style: { fontSize: 13, color: INK_SOFT, marginBottom: 6 } },
          'ペダル点 [{ time_s, accel_opening(0-100), brake_opening(0-100) }] — time_s 昇順'),
        React.createElement('textarea', { style: taStyle, value: pedalJson, onChange: e => setPedalJson(e.target.value) }),
      ),
      React.createElement('div', { style: { marginTop: 16 } },
        React.createElement('div', { style: { fontSize: 13, color: INK_SOFT, marginBottom: 6 } },
          'ボタンイベント [{ time_s, channel(0-15), press_duration_s>0 }] — ch0=始動, ch1-3=P/N/D'),
        React.createElement('textarea', { style: taStyle, value: buttonJson, onChange: e => setButtonJson(e.target.value) }),
      ),
      React.createElement('div', { style: { display: 'flex', gap: 12, marginTop: 20 } },
        React.createElement(Btn, { primary: true, disabled: saving, onClick: handleSave }, saving ? '保存中…' : '保存'),
        React.createElement(Btn, { ghost: true, onClick: onCancel }, 'キャンセル'),
      ),
    ),
    React.createElement(Note, { style: { marginTop: 16 } },
      'チャンネル割当: ch0=エンジンスタート, ch1=シフトP, ch2=シフトN, ch3=シフトD, ch4-15=オプション1-12'),
  );
}

function SequenceScreen() {
  const { Box, H2, Note } = window;
  return React.createElement('div', { style: { padding: 32 } },
    React.createElement(H2, null, 'シーケンス'),
    React.createElement(Box, { style: { padding: 32, marginTop: 20, textAlign: 'center' } },
      React.createElement(Note, null, 'シーケンス機能は Post-MVP で実装予定です。'),
    ),
  );
}

window.ScheduleScreen = ScheduleScreen;
window.ScheduleCreateForm = ScheduleCreateForm;
window.SequenceScreen = SequenceScreen;
