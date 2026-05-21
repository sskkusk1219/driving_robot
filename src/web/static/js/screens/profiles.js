// ── Profiles screens ──────────────────────────────────────

function ProfilesScreen() {
  const { useState, useEffect, useContext } = React;
  const { apiFetch, activeProfileId, setActiveProfileId, setActiveProfileName, setNav } = useContext(window.AppContext);
  const { INK, INK_SOFT, INK_MUTE, PAPER, PAPER_2, HATCH, Box, Btn, Pill, H2, Row } = window;

  const [profiles, setProfiles] = useState([]);
  const [mode, setMode] = useState('list'); // list | create | edit
  const [editTarget, setEditTarget] = useState(null);
  const [search, setSearch] = useState('');
  const [sortKey, setSortKey] = useState(null);  // 'name' | 'max_speed'
  const [sortAsc, setSortAsc] = useState(true);

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
      onGoCalib: () => setNav('calibration'),
      onGoLearning: () => setNav('learning'),
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
      onGoCalib: () => setNav('calibration'),
      onGoLearning: () => setNav('learning'),
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
      React.createElement(H2, { sub: '車両ごとのキャリブレーション・モデル・PIDゲインを管理します' }, '車両プロファイル'),
      React.createElement('div', { style: { flex: 1 } }),
      React.createElement('input', {
        type: 'text',
        value: search,
        onChange: e => setSearch(e.target.value),
        placeholder: '名前で検索…',
        style: {
          padding: '6px 12px', fontSize: 14,
          border: `1.3px solid ${INK}`, borderRadius: 4,
          fontFamily: 'inherit', background: PAPER, outline: 'none',
          width: 200,
        },
      }),
      React.createElement(Btn, { primary: true, big: true, onClick: () => setMode('create') }, '+ 新規プロファイル'),
    ),

    // Table
    React.createElement(Box, { style: { padding: 0 } },
      // ヘッダ行
      React.createElement('div', {
        style: {
          display: 'grid',
          gridTemplateColumns: '40px 2fr 1.4fr 1fr 1.4fr 0.7fr 0.7fr 1fr',
          borderBottom: `1px solid ${INK}`,
          padding: '10px 14px',
          background: PAPER_2,
          fontSize: 14, fontWeight: 700,
        },
      },
        React.createElement('div', null, ''),
        React.createElement('div', {
          onClick: () => handleSortClick('name'),
          style: { cursor: 'pointer', userSelect: 'none' },
        }, '名前' + sortIcon('name')),
        React.createElement('div', { style: { fontFamily: 'monospace' } }, '最大開度 (acc/brk)'),
        React.createElement('div', {
          onClick: () => handleSortClick('max_speed'),
          style: { cursor: 'pointer', userSelect: 'none', fontFamily: 'monospace' },
        }, '最高車速' + sortIcon('max_speed')),
        React.createElement('div', { style: { fontFamily: 'monospace' } }, 'Kp / Ki / Kd'),
        React.createElement('div', null, 'キャリブ'),
        React.createElement('div', null, 'モデル'),
        React.createElement('div', null, '操作'),
      ),

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
                [isActive ? '●' : '', '40px'],
                [React.createElement('b', { key: 'n' }, p.name), '2fr'],
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
                [React.createElement('div', { key: 'b', style: { display: 'flex', gap: 6 } },
                  !isActive && React.createElement(Btn, { onClick: () => handleSelect(p) }, '選択'),
                  React.createElement(Btn, { onClick: () => { setEditTarget(p); setMode('edit'); } }, '編集'),
                ), '1fr'],
              ],
              style: {
                padding: '12px 14px',
                background: isActive ? '#f4f1e6' : 'transparent',
                borderBottom: `1px dashed ${INK_MUTE}`,
              },
            });
          }),
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
function ProfileForm({ initial, onSave, onCancel, onDelete, onGoCalib, onGoLearning }) {
  const { useState } = React;
  const { INK_SOFT, INK_MUTE, Box, Btn, H2, Input, Row, Note, Hatch } = window;
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
  const accelZero   = calib?.accel_zero ?? '—';
  const accelFull   = calib?.accel_full ?? '—';
  const accelStroke = (typeof calib?.accel_zero === 'number' && typeof calib?.accel_full === 'number')
    ? calib.accel_full - calib.accel_zero : '—';
  const brakeZero   = calib?.brake_zero ?? '—';
  const brakeFull   = calib?.brake_full ?? '—';
  const brakeStroke = (typeof calib?.brake_zero === 'number' && typeof calib?.brake_full === 'number')
    ? calib.brake_full - calib.brake_zero : '—';

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
      style: { display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 14, flex: 1, minHeight: 0, overflow: 'auto' }
    },

      // ── Left column ──────────────────────────────────────
      React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14 } },

        // 基本情報
        React.createElement(Box, { label: '基本情報', style: { padding: 18 } },
          React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 12 } },
            inp('プロファイル名', 'name'),
            React.createElement('div', { style: { display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 12 } },
              inp('アクセル最大開度 [%]', 'max_accel_opening', { type: 'number', mono: true }),
              inp('ブレーキ最大開度 [%]', 'max_brake_opening', { type: 'number', mono: true }),
              inp('最高車速 [km/h]', 'max_speed', { type: 'number', mono: true }),
              inp('最大減速G [G]', 'max_decel_g', { type: 'number', mono: true }),
            ),
          ),
        ),

        // PIDゲイン
        React.createElement(Box, { label: 'PIDゲイン', style: { padding: 18 } },
          React.createElement('div', { style: { display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 12 } },
            inp('Kp', 'kp', { type: 'number', mono: true }),
            inp('Ki', 'ki', { type: 'number', mono: true }),
            inp('Kd', 'kd', { type: 'number', mono: true }),
          ),
        ),

        // 停止判定
        React.createElement(Box, { label: '停止判定', style: { padding: 18 } },
          React.createElement('div', { style: { display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 12 } },
            inp('逸脱閾値 [km/h]', 'deviation_threshold_kmh', { type: 'number', mono: true }),
            inp('逸脱継続 [s]', 'deviation_duration_s', { type: 'number', mono: true }),
          ),
        ),
      ),

      // ── Right column ─────────────────────────────────────
      React.createElement('div', { style: { display: 'flex', flexDirection: 'column', gap: 14 } },

        // キャリブレーションデータ
        React.createElement(Box, { label: 'キャリブレーションデータ', style: { padding: 18 } },
          !isEdit || !calib?.is_valid
            ? React.createElement('div', null,
                React.createElement('div', { style: { fontSize: 14, color: INK_SOFT, lineHeight: 1.8, marginBottom: 10 } },
                  isEdit
                    ? 'キャリブレーションが未実施です。キャリブレーション画面でゼロ/フル位置を設定してください。'
                    : 'プロファイル保存後にキャリブレーション画面でゼロ/フル位置を設定してください。'
                ),
                React.createElement(Btn, { onClick: onGoCalib }, 'キャリブレーション画面へ →'),
              )
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
                React.createElement('div', { style: { marginTop: 10 } },
                  React.createElement(Btn, { onClick: onGoCalib }, 'キャリブレーション画面へ →'),
                ),
              ),
        ),

        // 運転モデル
        React.createElement(Box, { label: '運転モデル', style: { padding: 18 } },
          !isEdit || !initial?.model_path
            ? React.createElement('div', null,
                React.createElement('div', { style: { fontSize: 14, color: INK_SOFT, lineHeight: 1.8, marginBottom: 10 } },
                  isEdit
                    ? 'モデルがありません。学習運転を行いモデルを取得してください。'
                    : 'キャリブレーション完了後に学習運転を行いモデルを取得してください。'
                ),
                React.createElement(Btn, { onClick: onGoLearning }, '学習運転画面へ →'),
              )
            : React.createElement('div', null,
                React.createElement(Hatch, { width: '100%', height: 110, label: 'learned model heatmap (speed × accel)' }),
                React.createElement('div', { style: { fontSize: 12, color: INK_SOFT, marginTop: 6 } },
                  initial.model_path,
                ),
                React.createElement('div', { style: { marginTop: 10 } },
                  React.createElement(Btn, { onClick: onGoLearning }, '学習運転画面へ →'),
                ),
              ),
        ),

        // Note
        React.createElement(Note, null,
          isEdit
            ? 'PIDゲイン変更後は自動走行の動作を確認してください。キャリブレーションデータは保持されます。'
            : 'プロファイル保存後、キャリブレーション → 学習運転の順で設定を完了させてください。'
        ),

        // Action buttons (right-aligned)
        React.createElement('div', { style: { display: 'flex', gap: 12, justifyContent: 'flex-end', marginTop: 'auto' } },
          isEdit && React.createElement(Btn, { danger: true, onClick: onDelete }, '削除'),
          React.createElement(Btn, { onClick: onCancel }, 'キャンセル'),
          React.createElement(Btn, { primary: true, big: true, onClick: handleSave }, isEdit ? '保存' : '作成'),
        ),
      ),
    ),
  );
}

window.ProfilesScreen = ProfilesScreen;
