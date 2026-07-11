// ── Profiles screens ──────────────────────────────────────

function ProfilesScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeProfileId, setActiveProfileId, activeProfileName, setActiveProfileName, setNav } = useContext(window.AppContext);
  const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH, Box, Btn, Pill, H2, Row, RowActions } = window;

  const [profiles, setProfiles] = useState([]);
  const [mode, setMode] = useState('list'); // list | create | edit
  const [editTarget, setEditTarget] = useState(null);
  const [search, setSearch] = useState('');
  const [sortKey, setSortKey] = useState(null);  // 'name' | 'max_speed'
  const [sortAsc, setSortAsc] = useState(true);
  const [editingNameId, setEditingNameId] = useState(null);
  const [editingNameValue, setEditingNameValue] = useState('');

  useEffect(() => { loadProfiles(); }, []);

  async function loadProfiles() {
    const data = await apiFetch('GET', '/api/v1/profiles/');
    if (data) setProfiles(data);
  }

  async function handleSelect(p) {
    const r = await apiFetch('POST', '/api/v1/drive/select-profile', { profile_id: p.id });
    if (!r) return;
    setActiveProfileId(p.id);
    setActiveProfileName(p.name);
    window.showToast(`「${p.name}」を選択しました`, 'success');
    loadProfiles();
  }

  function startEditName(p) {
    setEditingNameId(p.id);
    setEditingNameValue(p.name);
  }

  function cancelEditName() {
    setEditingNameId(null);
    setEditingNameValue('');
  }

  async function handleSaveName(p) {
    const trimmed = editingNameValue.trim();
    if (!trimmed) { window.showToast('プロファイル名を入力してください', 'error'); return; }
    if (trimmed === p.name) { cancelEditName(); return; }
    const r = await apiFetch('PUT', `/api/v1/profiles/${p.id}`, { name: trimmed });
    if (r) {
      window.showToast('名前を更新しました', 'success');
      if (p.id === activeProfileId) setActiveProfileName(trimmed);
      setEditingNameId(null);
      setEditingNameValue('');
      loadProfiles();
    }
  }

  async function handleCopy(p) {
    const payload = {
      name: `${p.name} のコピー`,
      max_accel_opening: p.max_accel_opening,
      max_brake_opening: p.max_brake_opening,
      max_speed: p.max_speed,
      max_decel_g: p.max_decel_g,
      pid_gains: { kp: p.pid_gains?.kp ?? 1.0, ki: p.pid_gains?.ki ?? 0.0, kd: p.pid_gains?.kd ?? 0.0 },
      stop_config: {
        deviation_threshold_kmh: p.stop_config?.deviation_threshold_kmh ?? 2.0,
        deviation_duration_s: p.stop_config?.deviation_duration_s ?? 4.0,
      },
      model_path: null,
      dynamics_params: { pid_preview_s: p.dynamics_params?.pid_preview_s ?? 0.0 },
    };
    const r = await apiFetch('POST', '/api/v1/profiles/', payload);
    if (r) {
      window.showToast(`「${r.name}」を作成しました`, 'success');
      loadProfiles();
    }
  }

  async function handleDelete(p) {
    if (!confirm(`「${p.name}」を削除しますか？`)) return;
    const r = await apiFetch('DELETE', `/api/v1/profiles/${p.id}`);
    if (r !== null) {
      window.showToast('プロファイルを削除しました', 'success');
      setMode('list');
      setEditTarget(null);
      loadProfiles();
    }
  }

  if (mode === 'create') {
    return React.createElement(ProfileForm, {
      onSave: async payload => {
        const r = await apiFetch('POST', '/api/v1/profiles/', payload);
        if (r) { window.showToast(`「${r.name}」を作成しました`, 'success'); setMode('list'); loadProfiles(); }
      },
      onCancel: () => setMode('list'),
    });
  }

  if (mode === 'edit' && editTarget) {
    return React.createElement(ProfileForm, {
      initial: editTarget,
      onSave: async payload => {
        const r = await apiFetch('PUT', `/api/v1/profiles/${editTarget.id}`, payload);
        if (r) { window.showToast(`「${r.name}」を更新しました`, 'success'); setMode('list'); setEditTarget(null); loadProfiles(); }
      },
      onCancel: () => { setMode('list'); setEditTarget(null); },
      onDelete: () => handleDelete(editTarget),
    });
  }

  // ── List view (ProfilesA) ──────────────────────────────────
  const activeProfile = profiles.find(p => p.id === activeProfileId);

  function handleSortClick(key) {
    if (sortKey === key) { setSortAsc(a => !a); } else { setSortKey(key); setSortAsc(true); }
  }
  function sortIcon(key) {
    if (sortKey !== key) return ' ↕';
    return sortAsc ? ' ↑' : ' ↓';
  }

  const q = search.trim().toLowerCase();
  const filtered = profiles.filter(p => p.name.toLowerCase().includes(q));
  const sorted = sortKey
    ? [...filtered].sort((a, b) => {
        const va = a[sortKey], vb = b[sortKey];
        const cmp = typeof va === 'string' ? va.localeCompare(vb, 'ja') : va - vb;
        return sortAsc ? cmp : -cmp;
      })
    : filtered;

  return React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14, height: '100%' } },

    // Header
    React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 12 } },
      React.createElement('div', { style: { flex: 1 } }),
      React.createElement('input', {
        type: 'text',
        value: search,
        onChange: e => setSearch(e.target.value),
        placeholder: '名前で検索…',
        style: {
          padding: '6px 12px', fontSize: 14,
          border: `1.3px solid ${INK}`, borderRadius: 4,
          fontFamily: 'inherit', background: PAPER, color: INK, outline: 'none',
          width: 200,
        },
      }),
      React.createElement(Btn, { primary: true, big: true, onClick: () => setMode('create') }, '＋ 新規作成'),
    ),

    // Table
    React.createElement(Box, { style: { padding: 0, display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 } },
      // ヘッダ行
      React.createElement('div', {
        style: {
          display: 'grid',
          gridTemplateColumns: '2fr 1.4fr 1fr 1.4fr 0.7fr 0.7fr 2fr',
          borderBottom: `1px solid ${INK}`,
          padding: '10px 14px',
          background: PAPER_2,
          fontSize: 14, fontWeight: 700,
          flexShrink: 0,
        },
      },
        React.createElement('div', {
          onClick: () => handleSortClick('name'),
          style: { cursor: 'pointer', userSelect: 'none' },
        }, '名前' + sortIcon('name')),
        React.createElement('div', { style: { fontFamily: 'inherit' } }, '最大開度 (acc/brk)'),
        React.createElement('div', {
          onClick: () => handleSortClick('max_speed'),
          style: { cursor: 'pointer', userSelect: 'none', fontFamily: 'inherit' },
        }, '最高車速' + sortIcon('max_speed')),
        React.createElement('div', { style: { fontFamily: 'inherit' } }, 'Kp / Ki / Kd'),
        React.createElement('div', null, 'キャリブ'),
        React.createElement('div', null, 'モデル'),
        React.createElement('div', null, '操作'),
      ),

      React.createElement('div', { style: { overflowY: 'auto', flex: 1, minHeight: 0 } },
      sorted.length === 0
        ? React.createElement('div', { style: { padding: '20px 14px', color: INK_SOFT, fontSize: 14 } },
            profiles.length === 0
              ? 'プロファイルがありません。「+ 新規プロファイル」で作成してください。'
              : `「${search}」に一致するプロファイルがありません。`)
        : sorted.map((p, i) => {
            const isActive = p.id === activeProfileId;
            const hasCalib = p.calibration?.is_valid;
            const hasModel = !!p.model_path;
            const kp = p.pid_gains?.kp ?? '—';
            const ki = p.pid_gains?.ki ?? '—';
            const kd = p.pid_gains?.kd ?? '—';

            return React.createElement(Row, {
              key: p.id,
              cells: [
                [editingNameId === p.id
                  ? React.createElement('div', { key: 'ne', style: { display: 'flex', gap: 4, alignItems: 'center' } },
                      React.createElement('input', {
                        autoFocus: true,
                        value: editingNameValue,
                        onChange: e => setEditingNameValue(e.target.value),
                        onKeyDown: e => {
                          if (e.key === 'Enter') handleSaveName(p);
                          if (e.key === 'Escape') cancelEditName();
                        },
                        style: {
                          fontSize: 14, padding: '2px 6px',
                          border: `1.3px solid ${INK}`, borderRadius: 3,
                          fontFamily: 'inherit', background: PAPER, color: INK,
                          outline: 'none', width: '100%',
                        },
                      }),
                      React.createElement('span', {
                        onClick: () => handleSaveName(p),
                        title: '保存',
                        style: { cursor: 'pointer', fontSize: 16, userSelect: 'none' },
                      }, '✓'),
                      React.createElement('span', {
                        onClick: cancelEditName,
                        title: 'キャンセル',
                        style: { cursor: 'pointer', fontSize: 16, userSelect: 'none' },
                      }, '✗'),
                    )
                  : React.createElement('div', { key: 'n', style: { display: 'flex', gap: 6, alignItems: 'center' } },
                      React.createElement('b', null, p.name),
                      React.createElement('span', {
                        onClick: () => startEditName(p),
                        title: '名前を変更',
                        style: { cursor: 'pointer', opacity: 0.5, fontSize: 13, userSelect: 'none' },
                      }, '✎'),
                    ),
                  '2fr'],
                [`${p.max_accel_opening} / ${p.max_brake_opening} %`, '1.4fr', 'mono'],
                [`${p.max_speed} km/h`, '1fr', 'mono'],
                [`${kp} / ${ki} / ${kd}`, '1.4fr', 'mono'],
                [hasCalib
                  ? React.createElement(Pill, { key: 'c', accent: 'READY' }, 'OK')
                  : React.createElement(Pill, { key: 'c' }, '未'),
                  '0.7fr'],
                [hasModel
                  ? React.createElement(Pill, { key: 'm', accent: 'READY' }, 'あり')
                  : React.createElement(Pill, { key: 'm' }, 'なし'),
                  '0.7fr'],
                [React.createElement(RowActions, {
                  key: 'b',
                  isActive,
                  onSelect: () => handleSelect(p),
                  onEdit:   () => { setEditTarget(p); setMode('edit'); },
                  onCopy:   () => handleCopy(p),
                  onDelete: () => handleDelete(p),
                }), '2fr'],
              ],
              style: {
                padding: '12px 14px',
                background: isActive ? '#201e16' : 'transparent',
                borderBottom: `1px dashed ${INK_MUTE}`,
              },
            });
          }),
      ),
    ),

    // Footer
    React.createElement('div', { style: { display: 'flex', gap: 12, fontSize: 13, color: INK_SOFT } },
      activeProfile
        ? React.createElement('span', null, '選択中: ', React.createElement('b', null, activeProfile.name))
        : React.createElement('span', null, '(未選択)'),
      React.createElement('div', { style: { flex: 1 } }),
      React.createElement('span', null,
        q ? `${sorted.length} / ${profiles.length}件` : `${profiles.length}件 / プロファイル数は無制限`
      ),
    ),
  );
}

// ── ProfileForm — 新規作成・編集共用 ────────────────────────────
function ProfileForm({ initial, onSave, onCancel, onDelete }) {
  const { useState } = React;
  const { INK, INK_SOFT, INK_MUTE, Box, Btn, H2, Input, Row } = window;
  const isEdit = !!initial;

  const [form, setForm] = useState({
    name: initial?.name ?? '',
    max_accel_opening: initial?.max_accel_opening ?? 80,
    max_brake_opening: initial?.max_brake_opening ?? 80,
    max_speed: initial?.max_speed ?? 120,
    max_decel_g: initial?.max_decel_g ?? 0.3,
    kp: initial?.pid_gains?.kp ?? 1.0,
    ki: initial?.pid_gains?.ki ?? 0.0,
    kd: initial?.pid_gains?.kd ?? 0.0,
    deviation_threshold_kmh: initial?.stop_config?.deviation_threshold_kmh ?? 2.0,
    deviation_duration_s: initial?.stop_config?.deviation_duration_s ?? 4.0,
    model_path: initial?.model_path ?? '',
    creep_speed_kmh: initial?.feedforward_params?.creep_speed_kmh ?? 7.0,
    creep_rate_kmhs: initial?.feedforward_params?.creep_rate_kmhs ?? 0.5,
    engine_brake_decel_kmhs: initial?.feedforward_params?.engine_brake_decel_kmhs ?? 1.0,
    stop_brake_opening_pct: initial?.feedforward_params?.stop_brake_opening_pct ?? 20.0,
    brake_deadband_pct: initial?.feedforward_params?.brake_deadband_pct ?? 1.0,
    accel_deadband_pct: initial?.feedforward_params?.accel_deadband_pct ?? 1.0,
    pid_preview_s: initial?.dynamics_params?.pid_preview_s ?? 0.0,
  });

  const set = (k, v) => setForm(f => ({ ...f, [k]: v }));

  async function handleSave() {
    if (!form.name.trim()) { window.showToast('プロファイル名を入力してください', 'error'); return; }
    await onSave({
      name: form.name.trim(),
      max_accel_opening: Number(form.max_accel_opening),
      max_brake_opening: Number(form.max_brake_opening),
      max_speed: Number(form.max_speed),
      max_decel_g: Number(form.max_decel_g),
      pid_gains: { kp: Number(form.kp), ki: Number(form.ki), kd: Number(form.kd) },
      stop_config: {
        deviation_threshold_kmh: Number(form.deviation_threshold_kmh),
        deviation_duration_s: Number(form.deviation_duration_s),
      },
      model_path: form.model_path.trim() || null,
      feedforward_params: {
        creep_speed_kmh: Number(form.creep_speed_kmh),
        creep_rate_kmhs: Number(form.creep_rate_kmhs),
        engine_brake_decel_kmhs: Number(form.engine_brake_decel_kmhs),
        stop_brake_opening_pct: Number(form.stop_brake_opening_pct),
        brake_deadband_pct: Number(form.brake_deadband_pct),
        accel_deadband_pct: Number(form.accel_deadband_pct),
      },
      dynamics_params: {
        pid_preview_s: Number(form.pid_preview_s),
        // FOPDT同定値は学習サイクルが書き込む表示用メタデータ。フォーム保存で消さないよう
        // 既存値をそのまま送り返す（本フォームに編集UIはない）。
        fopdt_k: initial?.dynamics_params?.fopdt_k ?? null,
        fopdt_tau: initial?.dynamics_params?.fopdt_tau ?? null,
        fopdt_theta: initial?.dynamics_params?.fopdt_theta ?? null,
      },
    });
  }

  const inp = (label, key, opts = {}) =>
    React.createElement(Input, {
      key,
      label,
      value: String(form[key]),
      onChange: v => set(key, v),
      type: opts.type ?? 'text',
      mono: opts.mono,
      width: '100%',
    });

  // calibration data
  const calib = initial?.calibration;
  const calibDate = calib?.calibrated_at
    ? new Date(calib.calibrated_at).toLocaleString('ja-JP')
    : null;
  const accelZero   = calib?.accel_zero_pos ?? '—';
  const accelFull   = calib?.accel_full_pos ?? '—';
  const accelStroke = calib?.accel_stroke ?? '—';
  const brakeZero   = calib?.brake_zero_pos ?? '—';
  const brakeFull   = calib?.brake_full_pos ?? '—';
  const brakeStroke = calib?.brake_stroke ?? '—';

  return React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14, height: '100%' } },

    // Header
    React.createElement('div', { style: { display: 'flex', alignItems: 'center', gap: 12 } },
      React.createElement(Btn, { onClick: onCancel }, '← 一覧に戻る'),
      React.createElement(H2, {
        sub: isEdit ? `プロファイル「${initial.name}」を編集します` : '新しい車両プロファイルを作成します'
      }, isEdit ? 'プロファイル編集' : '新規プロファイル作成'),
    ),

    // 2-column body
    React.createElement('div', {
      style: { display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 12, flex: 1, minHeight: 0, paddingTop: 12 }
    },

      // ── Left column ──────────────────────────────────────
      React.createElement('div', { style: { display: 'flex', flexDirection: 'column', justifyContent: 'space-between', height: '100%', minHeight: 0 } },

        // 基本情報
        React.createElement(Box, { label: '基本情報', style: { padding: 14 } },
          React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 10 } },
            inp('プロファイル名', 'name'),
            React.createElement('div', { style: { display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 10 } },
              inp('アクセル最大開度 [%]', 'max_accel_opening', { type: 'number', mono: true }),
              inp('ブレーキ最大開度 [%]', 'max_brake_opening', { type: 'number', mono: true }),
              inp('最高車速 [km/h]', 'max_speed', { type: 'number', mono: true }),
              inp('最大減速G [G]', 'max_decel_g', { type: 'number', mono: true }),
            ),
          ),
        ),

        // 停止判定
        React.createElement(Box, { label: '停止判定', style: { padding: 14 } },
          React.createElement('div', { style: { display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 10 } },
            inp('逸脱閾値 [km/h]', 'deviation_threshold_kmh', { type: 'number', mono: true }),
            inp('逸脱継続 [s]', 'deviation_duration_s', { type: 'number', mono: true }),
          ),
        ),

        // フィードフォワード（学習で自動更新／手動調整可）
        React.createElement(Box, { label: 'フィードフォワード', style: { padding: 14 } },
          React.createElement('div', { style: { display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 10 } },
            inp('クリープ車速 [km/h]', 'creep_speed_kmh', { type: 'number', mono: true }),
            inp('クリープ加速率 [km/h/s]', 'creep_rate_kmhs', { type: 'number', mono: true }),
            inp('コースト減速量 [km/h/s]', 'engine_brake_decel_kmhs', { type: 'number', mono: true }),
            inp('停車時ブレーキ開度 [%]', 'stop_brake_opening_pct', { type: 'number', mono: true }),
            inp('ブレーキ不感帯 [%]', 'brake_deadband_pct', { type: 'number', mono: true }),
            inp('アクセル不感帯 [%]', 'accel_deadband_pct', { type: 'number', mono: true }),
          ),
        ),
      ),

      // ── Right column ─────────────────────────────────────
      React.createElement('div', { style: { display: 'flex', flexDirection: 'column', justifyContent: 'space-between', height: '100%', minHeight: 0 } },

        // フィードバック（PIDゲイン）
        React.createElement(Box, { label: 'フィードバック', style: { padding: 14 } },
          React.createElement('div', { style: { display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 10 } },
            inp('Kp', 'kp', { type: 'number', mono: true }),
            inp('Ki', 'ki', { type: 'number', mono: true }),
            inp('Kd', 'kd', { type: 'number', mono: true }),
          ),
        ),

        // PID先読み（FBループのむだ時間補償。PID自動適合で自動算出／手動調整可。FFはnow-frame）
        React.createElement(Box, { label: 'PID先読み', style: { padding: 14 } },
          React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 8 } },
            inp('PID先読み補償 [s]', 'pid_preview_s', { type: 'number', mono: true }),
            React.createElement('div', { style: { fontSize: 12, color: INK_SOFT, lineHeight: 1.4 } },
              'PIDフィードバックのみ前倒しします（FFはnow-frame）。PID自動適合（学習サイクル）で自動算出されます。'
            ),
            React.createElement('div', { style: { display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 10, fontSize: 13 } },
              React.createElement('div', null, 'K: ', React.createElement('b', null, initial?.dynamics_params?.fopdt_k ?? '—')),
              React.createElement('div', null, 'τ [s]: ', React.createElement('b', null, initial?.dynamics_params?.fopdt_tau ?? '—')),
              React.createElement('div', null, 'θ [s]: ', React.createElement('b', null, initial?.dynamics_params?.fopdt_theta ?? '—')),
            ),
          ),
        ),

        // キャリブレーション
        React.createElement(Box, { label: 'キャリブレーション', style: { padding: 14 } },
          (!isEdit || !calib?.is_valid)
            ? React.createElement('div', { style: { fontSize: 14, color: INK_SOFT } }, '未実施')
            : React.createElement('div', null,
                React.createElement(Row, {
                  cells: [['', '1fr'], ['ZERO', '1fr', 'mono'], ['FULL', '1fr', 'mono'], ['STROKE', '1fr', 'mono']],
                  header: true,
                }),
                React.createElement(Row, {
                  cells: [
                    ['アクセル', '1fr'],
                    [String(accelZero), '1fr', 'mono'],
                    [String(accelFull), '1fr', 'mono'],
                    [String(accelStroke), '1fr', 'mono'],
                  ],
                }),
                React.createElement(Row, {
                  cells: [
                    ['ブレーキ', '1fr'],
                    [String(brakeZero), '1fr', 'mono'],
                    [String(brakeFull), '1fr', 'mono'],
                    [String(brakeStroke), '1fr', 'mono'],
                  ],
                }),
                calibDate && React.createElement('div', { style: { fontSize: 12, color: INK_SOFT, marginTop: 6 } },
                  `校正日: ${calibDate}`
                ),
              ),
        ),

        // モデル
        React.createElement(Box, { label: 'モデル', style: { padding: 14 } },
          (!isEdit || !initial?.model_path)
            ? React.createElement('div', { style: { fontSize: 14, color: INK_SOFT } }, '未実施')
            : React.createElement('div', { style: { fontSize: 13, color: INK_SOFT, fontFamily: 'inherit', wordBreak: 'break-all' } },
                initial.model_path,
              ),
        ),
      ),
    ),

    // Action buttons (outside grid)
    React.createElement('div', { style: { display: 'flex', gap: 12, justifyContent: 'flex-end' } },
      isEdit && React.createElement(Btn, { danger: true, onClick: onDelete }, '削除'),
      React.createElement(Btn, { onClick: onCancel }, 'キャンセル'),
      React.createElement(Btn, { primary: true, big: true, onClick: handleSave }, isEdit ? '保存' : '作成'),
    ),
  );
}

window.ProfilesScreen = ProfilesScreen;
